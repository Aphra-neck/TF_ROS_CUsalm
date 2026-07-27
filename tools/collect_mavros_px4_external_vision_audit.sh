#!/usr/bin/env bash

# Read-only audit of the installed MAVROS external-vision endpoints and the
# minimal PX4 EKF2 external-vision parameter set. It never creates external-
# vision or control topic publishers, writes parameters, changes mode, arms,
# or enters OFFBOARD.

set -u

error_count=0
warning_count=0
expected_px4_custom_revision=99c40407ff000000

section()
{
  printf '\n========== %s ==========\n' "$1"
}

show_topic_endpoint()
{
  topic="$1"
  requirement="$2"
  expected_type="$3"
  expected_endpoint="$4"
  expected_node="$5"
  expected_publisher_count="$6"
  printf '\n----- %s -----\n' "$topic"
  endpoint_output="$(timeout 5s ros2 topic info --verbose "$topic" 2>&1)"
  endpoint_rc=$?
  printf '%s\n' "$endpoint_output"
  if [ "$endpoint_rc" -ne 0 ]; then
    printf 'endpoint_status=MISSING requirement=%s\n' "$requirement"
    if [ "$requirement" = "required" ]; then
      error_count=$((error_count + 1))
    else
      warning_count=$((warning_count + 1))
    fi
  else
    publisher_count="$(printf '%s\n' "$endpoint_output" | awk \
      '/^Publisher count:/ {print $3; exit}')"
    if [ "$publisher_count" = "$expected_publisher_count" ]; then
      echo "publisher_authority=VERIFIED"
    else
      printf 'publisher_authority=NOT_VERIFIED expected=%s actual=%s\n' \
        "$expected_publisher_count" "${publisher_count:-UNAVAILABLE}"
      error_count=$((error_count + 1))
    fi

    if printf '%s\n' "$endpoint_output" | awk \
    -v expected_type="$expected_type" \
    -v expected_endpoint="$expected_endpoint" \
    -v expected_node="$expected_node" '
      /^Node name:/ {name = $3; node_namespace = ""; type = ""; endpoint = ""}
      /^Node namespace:/ {node_namespace = $3}
      /^Topic type:/ {type = $3}
      /^Endpoint type:/ {endpoint = $3}
      /^QoS profile:/ {
        if (name == expected_node && node_namespace == "/mavros" &&
            type == expected_type && endpoint == expected_endpoint) {
          found = 1
        }
      }
      END {exit found ? 0 : 1}
    '
    then
      echo "endpoint_contract=VERIFIED"
    else
      echo "endpoint_contract=NOT_VERIFIED"
      if [ "$requirement" = "required" ]; then
        error_count=$((error_count + 1))
      else
        warning_count=$((warning_count + 1))
      fi
    fi
  fi
}

read_px4_parameter()
{
  parameter="$1"
  printf '\n----- %s -----\n' "$parameter"
  parameter_output="$(timeout 10s ros2 param get \
    /mavros/param "$parameter" 2>&1)"
  parameter_rc=$?
  printf '%s\n' "$parameter_output"
  if [ "$parameter_rc" -ne 0 ] || \
    [[ "$parameter_output" == *"Parameter not set"* ]] || \
    [[ "$parameter_output" == *"not declared"* ]] || \
    ! printf '%s\n' "$parameter_output" | \
      grep -Eq '^(Integer|Double) value is:'
  then
    error_count=$((error_count + 1))
    echo "parameter_read=NOT_VERIFIED"
  else
    echo "parameter_read=VERIFIED"
  fi
}

wait_for_px4_parameter_cache()
{
  attempt=1
  while [ "$attempt" -le 15 ]; do
    cache_probe="$(timeout 5s ros2 param get \
      /mavros/param EKF2_EV_CTRL 2>&1)"
    cache_probe_rc=$?
    if [ "$cache_probe_rc" -eq 0 ] && \
      printf '%s\n' "$cache_probe" | \
        grep -Eq '^(Integer|Double) value is:'
    then
      printf 'px4_parameter_cache=READY attempts=%d\n' "$attempt"
      return 0
    fi
    attempt=$((attempt + 1))
    sleep 1
  done
  printf '%s\n' "$cache_probe"
  echo "px4_parameter_cache=NOT_READY"
  return 1
}

section "Capture context"
date --iso-8601=seconds 2>&1 || date
printf 'user=%s\n' "$(id -un)"
printf 'hostname=%s\n' "$(hostname)"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-UNSET}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-UNSET}"
if [ "${ROS_DOMAIN_ID:-UNSET}" != "42" ]; then
  error_count=$((error_count + 1))
  echo "ros_domain_contract=MISMATCH_EXPECTED_42"
fi
if [ "${RMW_IMPLEMENTATION:-UNSET}" != "rmw_fastrtps_cpp" ]; then
  error_count=$((error_count + 1))
  echo "rmw_contract=MISMATCH_EXPECTED_rmw_fastrtps_cpp"
fi

section "Installed MAVROS identity"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2=UNAVAILABLE"
  exit 2
fi
if ! dpkg-query -W \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs 2>&1
then
  error_count=$((error_count + 1))
fi
apt-cache policy \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs 2>&1 || true
printf 'mavros_prefix='
if ! ros2 pkg prefix mavros 2>&1; then
  error_count=$((error_count + 1))
fi
printf 'mavros_extras_prefix='
if ! ros2 pkg prefix mavros_extras 2>&1; then
  error_count=$((error_count + 1))
fi
printf 'mavros_msgs_prefix='
if ! ros2 pkg prefix mavros_msgs 2>&1; then
  error_count=$((error_count + 1))
fi

section "Relevant ROS message contracts"
for interface in \
  geometry_msgs/msg/PoseStamped \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  nav_msgs/msg/Odometry \
  mavros_msgs/msg/TimesyncStatus \
  mavros_msgs/srv/VehicleInfoGet \
  rcl_interfaces/srv/GetParameters
do
  printf '\n----- %s -----\n' "$interface"
  if ! ros2 interface show "$interface" 2>&1; then
    error_count=$((error_count + 1))
  fi
done

section "MAVROS node and plugin parameters"
ros2 node list 2>&1 || true
for node in /mavros/mavros_node /mavros/mavros /mavros/vision_pose
do
  printf '\n----- %s -----\n' "$node"
  if ! timeout 10s ros2 node info "$node" 2>&1; then
    error_count=$((error_count + 1))
  fi
  if ! timeout 10s ros2 param list "$node" 2>&1; then
    error_count=$((error_count + 1))
  fi
done
printf '\n----- /mavros/param -----\n'
if ! timeout 10s ros2 node info /mavros/param 2>&1; then
  error_count=$((error_count + 1))
fi
printf '\n----- vision_pose tf/listen -----\n'
tf_listen_output="$(timeout 5s ros2 param get \
  /mavros/vision_pose tf/listen 2>&1)"
tf_listen_rc=$?
printf '%s\n' "$tf_listen_output"
if [ "$tf_listen_rc" -eq 0 ] && \
  printf '%s\n' "$tf_listen_output" | \
    grep -Fxq 'Boolean value is: False'
then
  echo "vision_pose_tf_listener=DISABLED_EXPLICIT"
elif [[ "$tf_listen_output" == *"Boolean value is: True"* ]]; then
  error_count=$((error_count + 1))
  echo "vision_pose_tf_listener=ENABLED_NOT_ALLOWED"
elif [ "$tf_listen_rc" -ne 0 ]; then
  error_count=$((error_count + 1))
  echo "vision_pose_tf_listener=NOT_VERIFIED_DISABLED"
else
  error_count=$((error_count + 1))
  echo "vision_pose_tf_listener=UNEXPECTED_PARAMETER_RESPONSE"
fi
for parameter in \
  fcu_url \
  gcs_url \
  tgt_system \
  tgt_component \
  base_link_frame \
  map_frame \
  odom_frame
do
  printf '\n----- %s -----\n' "$parameter"
  if ! timeout 5s ros2 param get \
    /mavros/mavros_node "$parameter" 2>&1
  then
    error_count=$((error_count + 1))
  fi
done

section "External-vision endpoint graph"
show_topic_endpoint \
  /mavros/state required mavros_msgs/msg/State PUBLISHER sys 1
show_topic_endpoint \
  /mavros/local_position/odom required nav_msgs/msg/Odometry PUBLISHER \
  local_position 1
show_topic_endpoint \
  /mavros/timesync_status required mavros_msgs/msg/TimesyncStatus PUBLISHER time 1
show_topic_endpoint \
  /mavros/vision_pose/pose_cov required \
  geometry_msgs/msg/PoseWithCovarianceStamped SUBSCRIPTION vision_pose 0
show_topic_endpoint \
  /mavros/vision_pose/pose optional geometry_msgs/msg/PoseStamped SUBSCRIPTION \
  vision_pose 0
show_topic_endpoint \
  /mavros/odometry/out optional nav_msgs/msg/Odometry SUBSCRIPTION odometry 0
show_topic_endpoint \
  /mavros/mocap/pose optional geometry_msgs/msg/PoseStamped SUBSCRIPTION mocap 0
show_topic_endpoint \
  /mavros/odometry/in optional nav_msgs/msg/Odometry PUBLISHER odometry 1

section "External-vision interface decision"
echo "selected_input=/mavros/vision_pose/pose_cov"
echo "selected_type=geometry_msgs/msg/PoseWithCovarianceStamped"
echo "selected_covariance=ALL_NAN_UNKNOWN_PENDING_BENCH_PASS_THROUGH"
echo "pose_input=REJECTED_ZERO_INITIALIZED_COVARIANCE"
echo "odometry_out_input=REJECTED_FOR_POSE_ONLY_SOURCES"
echo "reason=NO_FAKE_ZERO_COVARIANCE_OR_TWIST"

section "MAVROS and PX4 state"
state_output="$(timeout 5s ros2 topic echo /mavros/state \
  mavros_msgs/msg/State --once 2>&1)"
state_rc=$?
printf '%s\n' "$state_output"
if [ "$state_rc" -ne 0 ]; then
  error_count=$((error_count + 1))
fi

printf '\n----- PX4 vehicle revision -----\n'
vehicle_info_type="$(timeout 5s ros2 service type \
  /mavros/vehicle_info_get 2>/dev/null)"
vehicle_info_type_rc=$?
printf 'vehicle_info_service_type=%s\n' "${vehicle_info_type:-UNAVAILABLE}"
if [ "$vehicle_info_type_rc" -eq 0 ] && \
  [ "$vehicle_info_type" = "mavros_msgs/srv/VehicleInfoGet" ]
then
  vehicle_info_output="$(timeout 10s ros2 service call \
    /mavros/vehicle_info_get "$vehicle_info_type" '{}' 2>&1)"
  vehicle_info_rc=$?
  printf '%s\n' "$vehicle_info_output"
  if [ "$vehicle_info_rc" -ne 0 ] || \
    { [[ "$vehicle_info_output" != *"success=True"* ]] && \
      [[ "$vehicle_info_output" != *"success: true"* ]]; } || \
    [[ "$vehicle_info_output" != \
      *"flight_custom_version='${expected_px4_custom_revision}'"* ]]
  then
    error_count=$((error_count + 1))
    echo "px4_revision=NOT_VERIFIED"
  else
    echo "px4_revision=VERIFIED_${expected_px4_custom_revision}"
  fi
else
  error_count=$((error_count + 1))
  echo "px4_revision=NOT_VERIFIED"
fi

section "Read-only runtime samples"
if ! timeout 5s ros2 topic echo /mavros/timesync_status \
  mavros_msgs/msg/TimesyncStatus --once 2>&1
then
  error_count=$((error_count + 1))
fi
if ! timeout 5s ros2 topic echo /mavros/local_position/odom \
  nav_msgs/msg/Odometry --once --no-arr 2>&1
then
  error_count=$((error_count + 1))
fi

section "Minimal PX4 external-vision parameters"
if [[ "$state_output" == *"connected: true"* ]] && \
  [[ "$state_output" == *"armed: false"* ]]
then
  if wait_for_px4_parameter_cache; then
    for parameter in \
      EKF2_EV_CTRL \
      EKF2_EV_DELAY \
      EKF2_EV_NOISE_MD \
      EKF2_EV_QMIN \
      EKF2_EV_POS_X \
      EKF2_EV_POS_Y \
      EKF2_EV_POS_Z \
      EKF2_EVP_NOISE \
      EKF2_EVP_GATE \
      EKF2_EVV_NOISE \
      EKF2_EVV_GATE \
      EKF2_EVA_NOISE \
      EKF2_HGT_REF \
      EKF2_GPS_CTRL \
      EKF2_BARO_CTRL \
      EKF2_RNG_CTRL \
      EKF2_MAG_TYPE \
      EKF2_NOAID_TOUT
    do
      read_px4_parameter "$parameter"
    done
  else
    error_count=$((error_count + 1))
  fi
else
  echo "parameter_reads=SKIPPED_REQUIRES_CONNECTED_AND_DISARMED_PX4"
  error_count=$((error_count + 1))
fi

section "Completion"
echo "scope=mavros_px4_external_vision"
printf 'errors=%d\n' "$error_count"
printf 'warnings=%d\n' "$warning_count"
echo "external_vision_topic_publishers_created=0"
echo "parameter_writes=0"
echo "mode_changes=0"
echo "arming_commands=0"
echo "offboard_commands=0"
echo "authorization=NONE"
if [ "$error_count" -eq 0 ]; then
  echo "analysis=READ_ONLY_AUDIT_COMPLETE"
  exit 0
fi
echo "analysis=INCOMPLETE"
exit 2
