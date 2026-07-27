#!/usr/bin/env bash

# Read-only host inventory for USB, container images, MAVROS and PX4 identity.
# It never reads or changes PX4 parameters and never arms or changes mode.

set -u

section()
{
  printf '\n========== %s ==========\n' "$1"
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"

section "Host operating system"
uname -a 2>&1 || true
cat /etc/os-release 2>&1 || true
if [ -r /etc/nv_tegra_release ]; then
  cat /etc/nv_tegra_release
fi
dpkg-query -W nvidia-jetpack 2>&1 || true

section "USB"
if command -v lsusb >/dev/null 2>&1; then
  lsusb -d 8086:0b3a 2>&1 || true
  lsusb -t 2>&1 || true
else
  echo "lsusb=UNAVAILABLE"
fi

section "Container images"
if command -v docker >/dev/null 2>&1; then
  docker version 2>&1 || true
  docker image inspect \
    isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-jp6.2 \
    isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-vslam3.2-dev-20260717 \
    --format 'tags={{json .RepoTags}} id={{.Id}}' 2>&1 || true
else
  echo "docker=UNAVAILABLE"
fi

section "MAVROS packages"
set +u
if [ -r /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -u
dpkg-query -W ros-humble-mavros ros-humble-mavros-msgs 2>&1 || true
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix mavros 2>&1 || true
  ros2 pkg prefix mavros_msgs 2>&1 || true
fi

section "MAVROS and PX4 identity"
if command -v ros2 >/dev/null 2>&1; then
  timeout 5s ros2 topic echo /mavros/state --once 2>&1 || true
  vehicle_info_type="$(timeout 5s ros2 service type \
    /mavros/vehicle_info_get 2>/dev/null || true)"
  printf 'vehicle_info_service_type=%s\n' "${vehicle_info_type:-UNAVAILABLE}"
  if [ -n "$vehicle_info_type" ] && \
    ros2 interface show "$vehicle_info_type" >/dev/null 2>&1
  then
    timeout 10s ros2 service call /mavros/vehicle_info_get \
      "$vehicle_info_type" '{}' 2>&1 || true
  elif [ -n "$vehicle_info_type" ]; then
    echo "vehicle_info_call=BLOCKED_LOCAL_INTERFACE_UNAVAILABLE"
  fi
else
  echo "ros2=UNAVAILABLE"
fi

section "Completion"
echo "scope=host_flight_stack"
echo "px4_parameter_reads=DEFERRED"
echo "analysis=READ_ONLY_INVENTORY_COMPLETE"
echo "authorization=NONE"
