#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 <map_name>" >&2
}

if (( $# != 1 )); then
  usage
  exit 64
fi

MAP_NAME="$1"
if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "map_name may contain only letters, numbers, '_' and '-'" >&2
  exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_TOPIC="/agt/map/mapping_occupancy"
OUTPUT_DIR="${REPOSITORY_ROOT}/runtime/maps/${MAP_NAME}"
OUTPUT_PREFIX="${OUTPUT_DIR}/${MAP_NAME}"

if ! ros2 topic list | rg -x --fixed-strings "${MAP_TOPIC}" >/dev/null; then
  echo "Mapping topic is not available: ${MAP_TOPIC}" >&2
  exit 2
fi

TOPIC_TYPE="$(ros2 topic type "${MAP_TOPIC}")"
if [[ "${TOPIC_TYPE}" != "nav_msgs/msg/OccupancyGrid" ]]; then
  echo "Unexpected type for ${MAP_TOPIC}: ${TOPIC_TYPE}" >&2
  echo "Expected nav_msgs/msg/OccupancyGrid" >&2
  exit 3
fi

mkdir -p "${OUTPUT_DIR}"
ros2 run nav2_map_server map_saver_cli \
  -t "${MAP_TOPIC}" \
  -f "${OUTPUT_PREFIX}" \
  --ros-args -p map_subscribe_transient_local:=true

if [[ ! -f "${OUTPUT_PREFIX}.pgm" || ! -f "${OUTPUT_PREFIX}.yaml" ]]; then
  echo "Map saver returned without both expected output files" >&2
  exit 4
fi

echo "Saved occupancy image: ${OUTPUT_PREFIX}.pgm"
echo "Saved map metadata:   ${OUTPUT_PREFIX}.yaml"
echo "Use Ctrl+C for a normal mapping shutdown so FAST-LIVO2 can write its PCD."
echo "This helper does not terminate FAST-LIVO2."
