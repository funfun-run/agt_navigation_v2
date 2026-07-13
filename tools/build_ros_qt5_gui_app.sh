#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SOURCE_DIR="${WS_ROOT}/third_party/ros_qt5_gui_app"
BUILD_DIR="${WS_ROOT}/build/ros_qt5_gui_app"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
JOBS="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"
MISSING_PACKAGES=()

if [[ ! -f "${SOURCE_DIR}/CMakeLists.txt" ]]; then
  echo "Ros_Qt5_Gui_App source not found: ${SOURCE_DIR}" >&2
  exit 1
fi

for package in qtbase5-private-dev libqt5svg5-dev libsdl2-image-dev; do
  if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q "install ok installed"; then
    MISSING_PACKAGES+=("${package}")
  fi
done
if (( ${#MISSING_PACKAGES[@]} > 0 )); then
  echo "Missing Ros_Qt5_Gui_App build dependencies: ${MISSING_PACKAGES[*]}" >&2
  echo "Install them with:" >&2
  echo "  sudo apt-get install -y ${MISSING_PACKAGES[*]}" >&2
  exit 3
fi

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${BUILD_DIR}/install"
cmake --build "${BUILD_DIR}" \
  --target channel_ros2 ros_qt5_gui_app \
  --parallel "${JOBS}"

echo "Built ${BUILD_DIR}/ros_qt5_gui_app and ${BUILD_DIR}/lib/libchannel_ros2.so"
