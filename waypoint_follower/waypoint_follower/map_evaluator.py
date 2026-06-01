#!/usr/bin/env python3
"""Evaluate a SLAM occupancy grid against the Gazebo reference map."""

import argparse
import json
import math
import os

import cv2
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt
from scipy.optimize import differential_evolution, minimize


def load_map(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    image_path = metadata['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f'Unable to read map image: {image_path}')
    occupancy = (255.0 - image.astype(np.float32)) / 255.0
    occupied = occupancy >= float(metadata.get('occupied_thresh', 0.65))
    free = occupancy <= float(metadata.get('free_thresh', 0.25))
    known = occupied | free
    return metadata, image, occupied, known


def grid_points(mask, metadata):
    rows, cols = np.nonzero(mask)
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = metadata['origin']
    x = origin_x + (cols + 0.5) * resolution
    y = origin_y + (mask.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y))


def transform_points(points, pose):
    tx, ty, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return (rotation @ points.T).T + np.array([tx, ty])


def sample_distance(points, distance_image, metadata):
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = metadata['origin']
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    rows = distance_image.shape[0] - 1 - np.floor(
        (points[:, 1] - origin_y) / resolution).astype(int)
    valid = (
        (rows >= 0) & (rows < distance_image.shape[0]) &
        (cols >= 0) & (cols < distance_image.shape[1])
    )
    distances = np.full(len(points), 2.0, dtype=float)
    distances[valid] = distance_image[rows[valid], cols[valid]] * resolution
    return distances


def rasterize(points, shape, metadata):
    resolution = float(metadata['resolution'])
    origin_x, origin_y, _ = metadata['origin']
    cols = np.floor((points[:, 0] - origin_x) / resolution).astype(int)
    rows = shape[0] - 1 - np.floor(
        (points[:, 1] - origin_y) / resolution).astype(int)
    valid = (
        (rows >= 0) & (rows < shape[0]) &
        (cols >= 0) & (cols < shape[1])
    )
    mask = np.zeros(shape, dtype=bool)
    mask[rows[valid], cols[valid]] = True
    return mask


def evaluate(reference_yaml, slam_yaml, output_dir, alignment_json=None):
    ref_meta, ref_image, ref_occupied, _ = load_map(reference_yaml)
    slam_meta, _, slam_occupied, slam_known = load_map(slam_yaml)
    ref_points = grid_points(ref_occupied, ref_meta)
    slam_points = grid_points(slam_occupied, slam_meta)
    slam_known_points = grid_points(slam_known, slam_meta)
    if len(ref_points) == 0 or len(slam_points) == 0:
        raise RuntimeError('Both maps must contain occupied cells')

    distance_image = distance_transform_edt(~ref_occupied)
    ref_center = ref_points.mean(axis=0)
    slam_center = slam_points.mean(axis=0)

    def objective(params):
        transformed = transform_points(slam_points, params)
        distances = sample_distance(transformed, distance_image, ref_meta)
        return float(np.mean(np.minimum(distances, 0.75) ** 2))

    initial_translation = ref_center - slam_center
    if alignment_json:
        with open(alignment_json, 'r', encoding='utf-8') as source:
            trajectory_metrics = json.load(source)
        hint = trajectory_metrics['alignment']
        alignment = np.asarray([
            hint['x_m'], hint['y_m'], math.radians(hint['yaw_deg'])])
        alignment_source = 'trajectory_metrics'
    else:
        bounds = [
            (initial_translation[0] - 5.0, initial_translation[0] + 5.0),
            (initial_translation[1] - 5.0, initial_translation[1] + 5.0),
            (-math.pi, math.pi),
        ]
        coarse = differential_evolution(
            objective, bounds=bounds, seed=42, workers=1, maxiter=80, polish=False)
        refined = minimize(objective, coarse.x, method='Nelder-Mead')
        alignment = refined.x
        alignment_source = 'map_geometry_optimization'

    transformed_points = transform_points(slam_points, alignment)
    transformed_known_points = transform_points(slam_known_points, alignment)
    wall_distances = sample_distance(transformed_points, distance_image, ref_meta)
    predicted = rasterize(transformed_points, ref_occupied.shape, ref_meta)
    explored = rasterize(transformed_known_points, ref_occupied.shape, ref_meta)
    tolerance_cells = max(1, int(round(0.15 / float(ref_meta['resolution']))))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * tolerance_cells + 1, 2 * tolerance_cells + 1),
    )
    ref_dilated = cv2.dilate(ref_occupied.astype(np.uint8), kernel).astype(bool)
    pred_dilated = cv2.dilate(predicted.astype(np.uint8), kernel).astype(bool)
    explored = cv2.dilate(explored.astype(np.uint8), kernel).astype(bool)
    evaluated_reference = ref_occupied & explored
    true_positive_precision = predicted & ref_dilated
    true_positive_recall = evaluated_reference & pred_dilated
    union = predicted | evaluated_reference

    metrics = {
        'reference_yaml': os.path.abspath(reference_yaml),
        'slam_yaml': os.path.abspath(slam_yaml),
        'alignment_source': alignment_source,
        'alignment': {
            'x_m': float(alignment[0]),
            'y_m': float(alignment[1]),
            'yaw_deg': float(math.degrees(alignment[2])),
        },
        'occupied_cell_iou_exact': (
            float(np.sum(predicted & evaluated_reference) / np.sum(union))
            if np.sum(union) else 0.0
        ),
        'wall_precision_15cm': float(
            np.sum(true_positive_precision) / max(1, np.sum(predicted))),
        'wall_recall_15cm': float(
            np.sum(true_positive_recall) / max(1, np.sum(evaluated_reference))),
        'wall_rmse_m': float(np.sqrt(np.mean(wall_distances ** 2))),
        'wall_mean_error_m': float(np.mean(wall_distances)),
        'estimated_occupied_cells': int(np.sum(predicted)),
        'reference_occupied_cells': int(np.sum(ref_occupied)),
        'evaluated_reference_occupied_cells': int(np.sum(evaluated_reference)),
    }

    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, 'mapping_metrics.json'),
              'w', encoding='utf-8') as output:
        json.dump(metrics, output, indent=2)

    overlay = cv2.cvtColor(ref_image, cv2.COLOR_GRAY2BGR)
    overlay[ref_occupied] = (0, 0, 255)
    overlay[predicted] = (255, 0, 0)
    overlay[ref_occupied & predicted] = (0, 255, 0)
    cv2.imwrite(os.path.join(output_dir, 'mapping_overlay.png'), overlay)
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference-yaml', required=True)
    parser.add_argument('--slam-yaml', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--alignment-json')
    args = parser.parse_args()
    metrics = evaluate(
        os.path.expanduser(args.reference_yaml),
        os.path.expanduser(args.slam_yaml),
        os.path.expanduser(args.output_dir),
        os.path.expanduser(args.alignment_json) if args.alignment_json else None,
    )
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
