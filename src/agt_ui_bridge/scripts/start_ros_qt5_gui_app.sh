#!/usr/bin/env bash

set -euo pipefail

PREFIX="$(ros2 pkg prefix agt_ui_bridge)"
WS_ROOT="$(cd "${PREFIX}/../.." && pwd)"
PACKAGE_SHARE="${PREFIX}/share/agt_ui_bridge"
BUILD_DIR="${ROS_QT5_GUI_BUILD_DIR:-${WS_ROOT}/build/ros_qt5_gui_app}"
RUNTIME_ROOT="${ROS_QT5_GUI_RUNTIME_DIR:-${WS_ROOT}/runtime/gui/ros_qt5_gui_app}"
BINARY="${BUILD_DIR}/ros_qt5_gui_app"
PROFILE="navigation"
RESET_CONFIG=false

usage() {
  echo "Usage: $0 [--profile mapping|navigation] [--reset-config]" >&2
}

while (( $# > 0 )); do
  case "$1" in
    --profile)
      if (( $# < 2 )); then
        echo "--profile requires mapping or navigation" >&2
        usage
        exit 64
      fi
      PROFILE="$2"
      shift 2
      ;;
    --reset-config)
      RESET_CONFIG=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 64
      ;;
  esac
done

if [[ "${PROFILE}" != "mapping" && "${PROFILE}" != "navigation" ]]; then
  echo "Invalid profile '${PROFILE}'; expected mapping or navigation" >&2
  exit 64
fi

RUNTIME_DIR="${RUNTIME_ROOT}/${PROFILE}"
CONFIG_TEMPLATE="${PACKAGE_SHARE}/config/ros_qt5_gui_${PROFILE}.json"

if [[ ! -x "${BINARY}" ]]; then
  echo "Ros_Qt5_Gui_App build artifact not found in this workspace: ${BINARY}" >&2
  echo "Run: ${WS_ROOT}/tools/build_ros_qt5_gui_app.sh" >&2
  exit 2
fi

mkdir -p "${RUNTIME_DIR}"
if [[ ! -f "${RUNTIME_DIR}/config.json" || "${RESET_CONFIG}" == true ]]; then
  cp "${CONFIG_TEMPLATE}" "${RUNTIME_DIR}/config.json"
fi

cd "${RUNTIME_DIR}"
export LD_LIBRARY_PATH="${BUILD_DIR}/lib:${LD_LIBRARY_PATH:-}"
exec "${BINARY}"
