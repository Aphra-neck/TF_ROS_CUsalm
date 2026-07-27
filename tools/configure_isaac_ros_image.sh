#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPOSITORY_DIR="$(cd "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"
DOCKER_DIR="${REPOSITORY_DIR}/docker"
CONFIG_FILE="${HOME}/.isaac_ros_common-config"
BEGIN_MARKER="# BEGIN TF_ROS_CUsalm yopo_mocap image"
END_MARKER="# END TF_ROS_CUsalm yopo_mocap image"

if [[ ! -f "${DOCKER_DIR}/Dockerfile.yopo_mocap" ]]; then
  echo "[STOP] Dockerfile not found: ${DOCKER_DIR}/Dockerfile.yopo_mocap" >&2
  exit 1
fi

TEMP_FILE="$(mktemp "${CONFIG_FILE}.tmp.XXXXXX")"
cleanup() {
  rm -f "${TEMP_FILE}"
}
trap cleanup EXIT

if [[ -f "${CONFIG_FILE}" ]]; then
  awk -v begin="${BEGIN_MARKER}" -v end="${END_MARKER}" '
    $0 == begin { inside = 1; found_begin = 1; next }
    $0 == end {
      if (!inside) exit 2
      inside = 0
      next
    }
    !inside { print }
    END { if (inside || (found_begin && inside)) exit 2 }
  ' "${CONFIG_FILE}" > "${TEMP_FILE}" || {
    echo "[STOP] malformed managed block in ${CONFIG_FILE}; file was not changed" >&2
    exit 1
  }
fi

if [[ -s "${TEMP_FILE}" ]]; then
  printf '\n' >> "${TEMP_FILE}"
fi

{
  printf '%s\n' "${BEGIN_MARKER}"
  cat <<'EOF'
if [[ -z "${CONFIG_IMAGE_KEY:-}" ]]; then
  CONFIG_IMAGE_KEY=ros2_humble.yopo_mocap
elif [[ ".${CONFIG_IMAGE_KEY}." != *.yopo_mocap.* ]]; then
  CONFIG_IMAGE_KEY="${CONFIG_IMAGE_KEY}.yopo_mocap"
fi
EOF
  printf 'CONFIG_DOCKER_SEARCH_DIRS+=(%q)\n' "${DOCKER_DIR}"
  printf '%s\n' "${END_MARKER}"
} >> "${TEMP_FILE}"

mv "${TEMP_FILE}" "${CONFIG_FILE}"
trap - EXIT

echo "Configured Isaac ROS image layer in ${CONFIG_FILE}"
echo "image_key_component=yopo_mocap"
echo "docker_search_dir=${DOCKER_DIR}"
echo "Next: stop the old container and run run_dev.sh once without -b."
echo "Also unset SKIP_DOCKER_BUILD and remove any non-empty CONFIG_SKIP_IMAGE_BUILD setting."
