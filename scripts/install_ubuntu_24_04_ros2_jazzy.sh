#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: /etc/os-release를 읽을 수 없습니다." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ERROR: 이 스크립트는 Ubuntu 24.04 전용입니다." >&2
  echo "Detected: ${PRETTY_NAME:-unknown}" >&2
  exit 1
fi

sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

ros_apt_source_version="$({
  curl -fsSL \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F 'tag_name' | awk -F\" '{print $4}'
})"

if [[ -z "${ros_apt_source_version}" ]]; then
  echo "ERROR: ros-apt-source 최신 버전을 확인하지 못했습니다." >&2
  exit 1
fi

ros_source_deb="/tmp/ros2-apt-source.deb"
curl -fL \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${VERSION_CODENAME}_all.deb" \
  -o "${ros_source_deb}"
sudo dpkg -i "${ros_source_deb}"

sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  git \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-nav2-map-server

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo
echo "ROS 2 Jazzy와 프로젝트 의존성 설치 완료"
echo "다음 단계:"
echo "  mkdir -p ~/ksg_ws/src"
echo "  git clone https://github.com/kimseonggyun1602/ksg1602.git ~/ksg_ws/src/yahboom_rosmaster"
echo "  bash ~/ksg_ws/src/yahboom_rosmaster/scripts/build_ksg_ws.sh"
