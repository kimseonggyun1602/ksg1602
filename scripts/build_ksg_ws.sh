#!/usr/bin/env bash
set -euo pipefail

workspace="${KSG_WS:-${HOME}/ksg_ws}"
source_dir="${workspace}/src/yahboom_rosmaster"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy가 설치되어 있지 않습니다." >&2
  exit 1
fi

if [[ ! -d "${source_dir}" ]]; then
  echo "ERROR: 저장소를 찾을 수 없습니다: ${source_dir}" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
cd "${workspace}"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --allow-overriding mecanum_drive_controller

echo
echo "Build complete. 새 터미널에서 다음을 실행하세요:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${workspace}/install/setup.bash"
