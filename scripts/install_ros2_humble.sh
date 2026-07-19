#!/usr/bin/env bash
# Install ROS 2 Humble on NVIDIA Jetson Orin (Ubuntu 22.04 / JetPack 6.x).
# Run on the Orin base station as a user with sudo.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
LOCALE_TARGET="${LOCALE_TARGET:-en_US.UTF-8}"

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this script as a normal user with sudo privileges (not as root)."
  exit 1
fi

if [[ ! -f /etc/os-release ]]; then
  echo "Cannot detect OS. Aborting."
  exit 1
fi

# shellcheck source=/dev/null
. /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "WARNING: ROS 2 Humble binary packages target Ubuntu 22.04."
  echo "Detected VERSION_ID=${VERSION_ID:-unknown}."
  echo "On JetPack 5 / Ubuntu 20.04, prefer Docker (see docker/docker-compose.microros.yml)."
  read -r -p "Continue anyway? [y/N] " reply
  if [[ ! "${reply}" =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

echo "==> Setting locale"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen "${LOCALE_TARGET}"
sudo update-locale LC_ALL="${LOCALE_TARGET}" LANG="${LOCALE_TARGET}"
export LANG="${LOCALE_TARGET}"

echo "==> Enabling Ubuntu Universe and prerequisites"
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

echo "==> Adding ROS 2 apt repository"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> Installing ROS 2 ${ROS_DISTRO}"
sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-rmw-fastrtps-cpp" \
  "ros-${ROS_DISTRO}-demo-nodes-cpp" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-argcomplete \
  build-essential \
  cmake \
  git \
  python3-pip \
  libasio-dev \
  libtinyxml2-dev

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

BASHRC="${HOME}/.bashrc"
MARKER="# >>> petcam ros2 humble >>>"
if ! grep -Fq "${MARKER}" "${BASHRC}" 2>/dev/null; then
  {
    echo ""
    echo "${MARKER}"
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "# <<< petcam ros2 humble <<<"
  } >> "${BASHRC}"
  echo "==> Appended ROS 2 source to ${BASHRC}"
fi

# shellcheck source=/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/source_ros.sh"
petcam_source "/opt/ros/${ROS_DISTRO}/setup.bash"
echo "==> ROS 2 ${ROS_DISTRO} installed."
echo "    Open a new shell or: source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "    Next: ./scripts/install_microros_agent.sh"
