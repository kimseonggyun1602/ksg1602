#!/usr/bin/env python3
"""Evaluate raw wheel odometry against Gazebo ground truth inside one rosbag."""

import argparse
import csv
import json
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


GT_TOPIC = '/gz_world_poses'
ODOM_TOPIC = '/mecanum_drive_controller/odom'
CMD_TOPIC = '/mecanum_drive_controller/cmd_vel'
SCAN_TOPIC = '/scan'
STATIC_TF_TOPIC = '/tf_static'


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def align_2d(source_xy, target_xy):
    source_center = source_xy.mean(axis=0)
    target_center = target_xy.mean(axis=0)
    centered_source = source_xy - source_center
    centered_target = target_xy - target_center
    u, _, vt = np.linalg.svd(centered_source.T @ centered_target)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def nearest_pairs(gt_rows, odom_rows, max_delta_sec=0.03):
    gt_stamps = np.asarray([row[0] for row in gt_rows])
    pairs = []
    for odom in odom_rows:
        index = int(np.searchsorted(gt_stamps, odom[0]))
        candidates = [min(index, len(gt_rows) - 1)]
        if index > 0:
            candidates.append(index - 1)
        if index + 1 < len(gt_rows):
            candidates.append(index + 1)
        best = min(candidates, key=lambda item: abs(gt_stamps[item] - odom[0]))
        if abs(gt_stamps[best] - odom[0]) <= max_delta_sec:
            pairs.append((gt_rows[best], odom))
    return pairs


def calculate_metrics(pairs):
    gt = np.asarray([[pair[0][1], pair[0][2]] for pair in pairs])
    odom = np.asarray([[pair[1][1], pair[1][2]] for pair in pairs])
    gt_yaw = np.asarray([pair[0][3] for pair in pairs])
    odom_yaw = np.asarray([pair[1][3] for pair in pairs])
    stamps = np.asarray([pair[1][0] for pair in pairs])

    rotation, translation = align_2d(odom, gt)
    aligned = (rotation @ odom.T).T + translation
    alignment_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    aligned_yaw = np.asarray([
        normalize_angle(value + alignment_yaw) for value in odom_yaw])

    errors = np.linalg.norm(aligned - gt, axis=1)
    yaw_errors = np.asarray([
        normalize_angle(a - b) for a, b in zip(aligned_yaw, gt_yaw)])

    rpe_translation = []
    rpe_yaw = []
    for index, stamp in enumerate(stamps):
        next_index = int(np.searchsorted(stamps, stamp + 1.0))
        if next_index >= len(stamps):
            continue
        gt_delta = gt[next_index] - gt[index]
        odom_delta = aligned[next_index] - aligned[index]
        rpe_translation.append(float(np.linalg.norm(odom_delta - gt_delta)))
        gt_delta_yaw = normalize_angle(gt_yaw[next_index] - gt_yaw[index])
        odom_delta_yaw = normalize_angle(
            aligned_yaw[next_index] - aligned_yaw[index])
        rpe_yaw.append(normalize_angle(odom_delta_yaw - gt_delta_yaw))

    return {
        'paired_samples': len(pairs),
        'duration_sec': float(stamps[-1] - stamps[0]),
        'ate_rmse_m': float(np.sqrt(np.mean(errors ** 2))),
        'ate_mean_m': float(np.mean(errors)),
        'yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(yaw_errors ** 2)))),
        'final_drift_m': float(errors[-1]),
        'max_position_error_m': float(np.max(errors)),
        'rpe_translation_rmse_m_1s': (
            float(np.sqrt(np.mean(np.square(rpe_translation))))
            if rpe_translation else None),
        'rpe_yaw_rmse_deg_1s': (
            float(np.degrees(np.sqrt(np.mean(np.square(rpe_yaw)))))
            if rpe_yaw else None),
        'alignment': {
            'x_m': float(translation[0]),
            'y_m': float(translation[1]),
            'yaw_deg': float(math.degrees(alignment_yaw)),
        },
        '_aligned_xy': aligned,
        '_gt_xy': gt,
        '_errors': errors,
    }


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    reader.open(storage_options, rosbag2_py.ConverterOptions('', ''))
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in {GT_TOPIC, ODOM_TOPIC, CMD_TOPIC, SCAN_TOPIC, STATIC_TF_TOPIC}
    }

    gt_rows = []
    odom_rows = []
    commands = []
    scan_count = 0
    scan_frames = set()
    static_frames = set()

    while reader.has_next():
        topic, data, received_stamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        if topic == GT_TOPIC:
            if not msg.transforms:
                continue
            transform = msg.transforms[0]
            p = transform.transform.translation
            gt_stamp = stamp_sec(transform.header.stamp)
            if gt_stamp == 0.0:
                gt_stamp = received_stamp_ns * 1e-9
            gt_rows.append((
                gt_stamp,
                p.x, p.y, yaw_from_quaternion(transform.transform.rotation)))
        elif topic == ODOM_TOPIC:
            p = msg.pose.pose.position
            odom_rows.append((
                stamp_sec(msg.header.stamp),
                p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)))
        elif topic == CMD_TOPIC:
            commands.append((
                stamp_sec(msg.header.stamp),
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.angular.z))
        elif topic == SCAN_TOPIC:
            scan_count += 1
            scan_frames.add(msg.header.frame_id)
        elif topic == STATIC_TF_TOPIC:
            for transform in msg.transforms:
                static_frames.add(
                    f'{transform.header.frame_id} -> {transform.child_frame_id}')

    return gt_rows, odom_rows, commands, scan_count, scan_frames, static_frames


def write_csv(path, pairs, metrics):
    aligned = metrics['_aligned_xy']
    errors = metrics['_errors']
    with open(path, 'w', newline='', encoding='utf-8') as output:
        writer = csv.writer(output)
        writer.writerow([
            'stamp_sec', 'gt_x_m', 'gt_y_m', 'gt_yaw_rad',
            'wheel_odom_x_m', 'wheel_odom_y_m', 'wheel_odom_yaw_rad',
            'aligned_wheel_odom_x_m', 'aligned_wheel_odom_y_m',
            'position_error_m'])
        for index, pair in enumerate(pairs):
            gt, odom = pair
            writer.writerow([
                odom[0], gt[1], gt[2], gt[3],
                odom[1], odom[2], odom[3],
                aligned[index, 0], aligned[index, 1], errors[index]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()

    bag_path = os.path.abspath(os.path.expanduser(args.bag))
    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(output_dir, exist_ok=True)

    gt, odom, commands, scan_count, scan_frames, static_frames = read_bag(
        bag_path)
    pairs = nearest_pairs(gt, odom)
    if len(pairs) < 3:
        raise RuntimeError('Not enough timestamp-matched GT and wheel odom samples')

    metrics = calculate_metrics(pairs)
    nonzero_commands = [
        row for row in commands
        if math.hypot(row[1], row[2]) > 1e-3 or abs(row[3]) > 1e-3]
    lateral_commands = [
        row for row in nonzero_commands
        if abs(row[2]) > 0.10 and abs(row[2]) > abs(row[1]) * 0.5]

    report = {
        'bag_path': bag_path,
        'raw_wheel_odom_vs_ground_truth': {
            key: value for key, value in metrics.items()
            if not key.startswith('_')
        },
        'motion_profile': {
            'command_messages': len(commands),
            'moving_command_messages': len(nonzero_commands),
            'lateral_command_messages': len(lateral_commands),
            'lateral_command_ratio': (
                len(lateral_commands) / len(nonzero_commands)
                if nonzero_commands else 0.0),
        },
        'sensor_and_tf_probe': {
            'scan_messages': scan_count,
            'scan_frames': sorted(scan_frames),
            'static_transforms': sorted(static_frames),
        },
    }
    with open(os.path.join(output_dir, 'raw_bag_metrics.json'), 'w',
              encoding='utf-8') as output:
        json.dump(report, output, indent=2)
    write_csv(os.path.join(output_dir, 'raw_wheel_odom_vs_gt.csv'), pairs, metrics)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
