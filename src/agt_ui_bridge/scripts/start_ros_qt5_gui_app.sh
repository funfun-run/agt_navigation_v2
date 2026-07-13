#!/usr/bin/env bash

set -euo pipefail

PREFIX="$(ros2 pkg prefix agt_ui_bridge)"
WS_ROOT="$(cd "${PREFIX}/../.." && pwd)"
PACKAGE_SHARE="${PREFIX}/share/agt_ui_bridge"
BUILD_DIR="${ROS_QT5_GUI_BUILD_DIR:-${WS_ROOT}/build/ros_qt5_gui_app}"
RUNTIME_DIR="${ROS_QT5_GUI_RUNTIME_DIR:-${WS_ROOT}/runtime/gui/ros_qt5_gui_app}"
BINARY="${BUILD_DIR}/ros_qt5_gui_app"
RESET_CONFIG="${1:-}"

if [[ ! -x "${BINARY}" ]]; then
  LEGACY_BUILD_DIR="/home/yangxuan/ros2_ws/src/Ros_Qt5_Gui_App/build"
  if [[ -x "${LEGACY_BUILD_DIR}/ros_qt5_gui_app" ]]; then
    echo "Using the existing legacy GUI binary until the vendored source is rebuilt." >&2
    BUILD_DIR="${LEGACY_BUILD_DIR}"
    BINARY="${BUILD_DIR}/ros_qt5_gui_app"
  fi
fi

if [[ ! -x "${BINARY}" ]]; then
  echo "Ros_Qt5_Gui_App has not been built: ${BINARY}" >&2
  echo "Run: ${WS_ROOT}/tools/build_ros_qt5_gui_app.sh" >&2
  exit 2
fi

mkdir -p "${RUNTIME_DIR}"
if [[ ! -f "${RUNTIME_DIR}/config.json" || "${RESET_CONFIG}" == "--reset-config" ]]; then
  cp "${PACKAGE_SHARE}/config/ros_qt5_gui_app.json" "${RUNTIME_DIR}/config.json"
fi

cd "${RUNTIME_DIR}"
export LD_LIBRARY_PATH="${BUILD_DIR}/lib:${LD_LIBRARY_PATH:-}"
exec "${BINARY}"
