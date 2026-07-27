#!/usr/bin/env bash

# Read-only inventory for localization-runtime. YOPO and MAVROS/PX4 are
# intentionally collected by separate scripts in their own environments.

set -u

workspace="${WORKSPACE:-/workspaces/isaac_ros-dev}"

section()
{
  printf '\n========== %s ==========\n' "$1"
}

repo_snapshot()
{
  label="$1"
  path="$2"

  printf '\n[%s]\npath=%s\n' "$label" "$path"
  if [ ! -d "${path}/.git" ]; then
    echo "status=MISSING"
    return
  fi

  echo "status=FOUND"
  printf 'branch='
  git -C "$path" rev-parse --abbrev-ref HEAD 2>&1 || true
  printf 'commit='
  git -C "$path" rev-parse HEAD 2>&1 || true
  echo "remotes:"
  git -C "$path" remote -v 2>&1 || true
  echo "working_tree:"
  if [ -z "$(git -C "$path" status --porcelain 2>/dev/null)" ]; then
    echo "CLEAN"
  else
    git -C "$path" status --short 2>&1 || true
  fi
}

package_snapshot()
{
  package="$1"
  echo "[${package}]"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2=UNAVAILABLE"
    return
  fi
  ros2 pkg prefix "$package" 2>&1 || true
  ros2 pkg xml "$package" 2>&1 | sed -n '/<name>/p; /<version>/p' || true
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'workspace=%s\n' "$workspace"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"

section "Operating system"
uname -a 2>&1 || true
cat /etc/os-release 2>&1 || true
if [ -r /etc/nv_tegra_release ]; then
  cat /etc/nv_tegra_release
fi
dpkg-query -W nvidia-jetpack 2>&1 || true
if command -v nvcc >/dev/null 2>&1; then
  nvcc --version 2>&1 || true
elif [ -x /usr/local/cuda/bin/nvcc ]; then
  /usr/local/cuda/bin/nvcc --version 2>&1 || true
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi 2>&1 || true
fi

section "ROS environment"
set +u
if [ -r /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -r "${workspace}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${workspace}/install/setup.bash"
fi
set -u
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-UNSET}"
printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-UNSET}"

section "Git repositories"
repo_snapshot "TF_ROS_CUsalm" "${workspace}/src/TF_ROS_CUsalm"
repo_snapshot "cuVSLAM bringup" "${workspace}/src/cuvslam-yopo-adapter"
repo_snapshot "isaac_ros_visual_slam" "${workspace}/src/isaac_ros_visual_slam"
repo_snapshot "vrpn_client_ros2" "${workspace}/src/vrpn_client_ros2"

section "Visual SLAM wrapper patch"
vslam_repo="${workspace}/src/isaac_ros_visual_slam"
vslam_patch="${workspace}/src/cuvslam-yopo-adapter/integrations/isaac_ros_3_2_yopo/patches/isaac_ros_visual_slam_v3_2_15_imu_timestamp.patch"
vslam_source="${vslam_repo}/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp"
if [ -f "$vslam_patch" ]; then
  sha256sum "$vslam_patch"
else
  echo "patch=MISSING"
fi
if [ -d "${vslam_repo}/.git" ] && [ -f "$vslam_patch" ]; then
  printf 'tracked_status='
  git -C "$vslam_repo" status --short --untracked-files=no 2>&1 || true
  if git -C "$vslam_repo" apply --reverse --check "$vslam_patch" 2>/dev/null; then
    echo "patch_state=APPLIED_AND_REVERSE_CHECK_PASSED"
  else
    echo "patch_state=NOT_APPLIED_OR_CONTENT_MISMATCH"
  fi
  git -C "$vslam_repo" diff --check 2>&1 || true
fi
if [ -f "$vslam_source" ] && \
  grep -Fq ISAAC_ROS_YOPO_IMU_TIMESTAMP_PATCH_V1 "$vslam_source"
then
  echo "source_marker=PRESENT"
else
  echo "source_marker=MISSING"
fi

section "ROS and Debian packages"
package_snapshot "isaac_ros_yopo_bringup"
package_snapshot "isaac_ros_visual_slam"
package_snapshot "realsense2_camera"
package_snapshot "vrpn_client_ros"
dpkg-query -W ros-humble-realsense2-camera 2>&1 || true

section "RealSense device"
if command -v lsusb >/dev/null 2>&1; then
  lsusb -d 8086:0b3a 2>&1 || true
  lsusb -t 2>&1 || true
else
  echo "lsusb=UNAVAILABLE_IN_THIS_ENVIRONMENT"
fi
pkg-config --modversion realsense2 2>&1 || true
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  timeout 10s rs-enumerate-devices -s 2>&1 || true
else
  echo "rs-enumerate-devices=UNAVAILABLE"
fi

section "Completion"
echo "scope=localization_runtime"
echo "analysis=READ_ONLY_INVENTORY_COMPLETE"
echo "authorization=NONE"
