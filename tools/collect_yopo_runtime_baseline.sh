#!/usr/bin/env bash

# Read-only inventory for the isolated YOPO runtime. Run this from the same
# host environment or container that will execute YOPO.

set -u

yopo_repo="${YOPO_REPO:-/home/nvidia/catkin_ws/src/YOPO_ROS2}"
yopo_python="${YOPO_PYTHON:-/usr/bin/python3}"

section()
{
  printf '\n========== %s ==========\n' "$1"
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'yopo_repo=%s\n' "$yopo_repo"
printf 'yopo_python=%s\n' "$yopo_python"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"
printf 'CONDA_DEFAULT_ENV=%s\n' "${CONDA_DEFAULT_ENV:-UNSET}"
printf 'CONDA_PREFIX=%s\n' "${CONDA_PREFIX:-UNSET}"
if command -v conda >/dev/null 2>&1; then
  conda info --envs 2>&1 || true
fi

section "YOPO repository"
if [ ! -d "${yopo_repo}/.git" ]; then
  echo "status=MISSING"
else
  echo "status=FOUND"
  printf 'branch='
  git -C "$yopo_repo" rev-parse --abbrev-ref HEAD 2>&1 || true
  printf 'commit='
  git -C "$yopo_repo" rev-parse HEAD 2>&1 || true
  echo "remotes:"
  git -C "$yopo_repo" remote -v 2>&1 || true
  echo "working_tree:"
  if [ -z "$(git -C "$yopo_repo" status --porcelain 2>/dev/null)" ]; then
    echo "CLEAN"
  else
    git -C "$yopo_repo" status --short 2>&1 || true
  fi
fi

section "YOPO immutable files"
for relative_path in \
  "YOPO/saved/YOPO_1/epoch50.pth" \
  "YOPO/config/traj_opt.yaml" \
  "YOPO/requirements.txt"
do
  if git -C "$yopo_repo" cat-file -e "HEAD:${relative_path}" 2>/dev/null; then
    blob_id="$(git -C "$yopo_repo" rev-parse "HEAD:${relative_path}")"
    canonical_sha256="$(git -C "$yopo_repo" show "HEAD:${relative_path}" | \
      sha256sum | cut -d ' ' -f 1)"
    printf '%s  git:HEAD:%s blob=%s\n' \
      "$canonical_sha256" "$relative_path" "$blob_id"
  else
    echo "MISSING git:HEAD:${relative_path}"
  fi
done

section "YOPO Python runtime"
if [ -x "$yopo_python" ]; then
  "$yopo_python" --version 2>&1 || true
  "$yopo_python" -c '
import sys
import torch
import numpy
print("python_executable=" + sys.executable)
print("torch_version=" + torch.__version__)
print("torch_file=" + torch.__file__)
print("torch_cuda_version=" + str(torch.version.cuda))
print("torch_cuda_available=" + str(torch.cuda.is_available()))
print("cuda_device=" + (
    torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NONE"))
print("numpy_version=" + numpy.__version__)
' 2>&1 || true
else
  echo "python=MISSING $yopo_python"
fi

section "YOPO ROS installation"
if command -v ros2 >/dev/null 2>&1; then
  yopo_prefix="$(ros2 pkg prefix yopo_planner 2>/dev/null || true)"
  printf 'yopo_planner_prefix=%s\n' "${yopo_prefix:-UNAVAILABLE}"
  yopo_node="${yopo_prefix}/lib/yopo_planner/yopo_node"
  if [ -n "$yopo_prefix" ] && [ -f "$yopo_node" ]; then
    printf 'yopo_node=%s\n' "$yopo_node"
    printf 'yopo_node_shebang='
    head -n 1 "$yopo_node" 2>&1 || true
  else
    echo "yopo_node=UNAVAILABLE"
  fi
  ros2 pkg prefix quadrotor_msgs 2>&1 || true
else
  echo "ros2=UNAVAILABLE"
fi

section "Completion"
echo "scope=yopo_runtime"
echo "analysis=READ_ONLY_INVENTORY_COMPLETE"
echo "authorization=NONE"
