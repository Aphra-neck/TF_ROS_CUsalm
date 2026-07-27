#!/usr/bin/env bash

# Container-only supervisor for D435 depth and the mocap-primary localization path.
# MAVROS must already be running on the Jetson host. This script never arms,
# changes PX4 mode, starts YOPO, or publishes control commands.
# It keeps all container-side launch processes under one foreground terminal.

set -Eeuo pipefail

readonly SCRIPT_NAME="$(basename "$0")"
readonly ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
readonly WORKSPACE_SETUP_FILE="${WORKSPACE_SETUP_FILE:-/workspaces/isaac_ros-dev/install/setup.bash}"
readonly WORKSPACE_ROOT="${WORKSPACE_ROOT:-/workspaces/isaac_ros-dev}"
readonly STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-30}"
readonly PROBE_TIMEOUT_SEC="${PROBE_TIMEOUT_SEC:-6}"
readonly DIAGNOSTIC_PROBE_TIMEOUT_SEC="${DIAGNOSTIC_PROBE_TIMEOUT_SEC:-6}"
readonly DEPTH_RATE_PROBE_SEC="${DEPTH_RATE_PROBE_SEC:-8}"
readonly MINIMUM_DEPTH_RATE_HZ="${MINIMUM_DEPTH_RATE_HZ:-80}"
readonly MAXIMUM_DEPTH_RATE_HZ="${MAXIMUM_DEPTH_RATE_HZ:-100}"
readonly SHUTDOWN_TIMEOUT_SEC="${SHUTDOWN_TIMEOUT_SEC:-8}"
readonly LOCK_FILE="${MOCAP_PRIMARY_LOCK_FILE:-/tmp/yopo_mocap_primary_container.lock}"
readonly D435_SERIAL="${D435_SERIAL-243622070369}"
readonly YOPO_DEPTH_TOPIC="${YOPO_DEPTH_TOPIC:-/depth_image}"
readonly EXPECTED_SELECTOR_CONTRACT_ID="yopo_mocap_primary_selector_20260727_v2"
readonly EXPECTED_GATEWAY_CONTRACT_ID="yopo_mocap_primary_output_gateway_20260727_v2"

declare -a child_names=()
declare -a child_pids=()
declare -a child_logs=()
declare -a child_exit_reported=()
tail_pid=""
lock_fd=""
shutdown_started=0
pending_signal=""
external_vision_active=0
armed_stop_warning_issued=0
last_gateway_published=""
last_depth_rate_hz=""
runtime_health_failure="not_checked"

usage()
{
  cat <<EOF
Usage: bash tools/${SCRIPT_NAME}

Run this once inside the Isaac ROS container after MAVROS is connected on the
Jetson host. It starts one depth-only D435 node plus the mocap-primary
localization chain. Keep this terminal open; press Ctrl-C to stop the
container-side chain in reverse order.
EOF
}

log()
{
  printf '[mocap-primary] %s\n' "$*"
}

stop()
{
  log "STOP: $*" >&2
  exit 1
}

validate_positive_integer()
{
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^[1-9][0-9]*$ ]]; then
    stop "${name} must be a positive integer; got '${value}'"
  fi
}

process_group_is_alive()
{
  local pid="${1:-}"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
  kill -0 -- "-$pid" 2>/dev/null
}

shutdown_children()
{
  local index
  local deadline
  local any_alive

  if [ "$shutdown_started" -eq 1 ]; then
    return
  fi
  shutdown_started=1
  set +e

  if [ -n "$tail_pid" ]; then
    kill "$tail_pid" 2>/dev/null || true
    wait "$tail_pid" 2>/dev/null || true
  fi

  if [ "${#child_pids[@]}" -eq 0 ]; then
    return
  fi
  log "Stopping container-side localization chain"

  for ((index = ${#child_pids[@]} - 1; index >= 0; index--)); do
    if process_group_is_alive "${child_pids[$index]}"; then
      log "SIGINT -> ${child_names[$index]}"
      kill -INT -- "-${child_pids[$index]}" 2>/dev/null || true
    fi
  done

  deadline=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while [ "$SECONDS" -lt "$deadline" ]; do
    any_alive=0
    for index in "${!child_pids[@]}"; do
      if process_group_is_alive "${child_pids[$index]}"; then
        any_alive=1
        break
      fi
    done
    if [ "$any_alive" -eq 0 ]; then
      break
    fi
    sleep 1
  done

  for ((index = ${#child_pids[@]} - 1; index >= 0; index--)); do
    if process_group_is_alive "${child_pids[$index]}"; then
      log "SIGTERM -> ${child_names[$index]}"
      kill -TERM -- "-${child_pids[$index]}" 2>/dev/null || true
    fi
  done

  deadline=$((SECONDS + SHUTDOWN_TIMEOUT_SEC))
  while [ "$SECONDS" -lt "$deadline" ]; do
    any_alive=0
    for index in "${!child_pids[@]}"; do
      if process_group_is_alive "${child_pids[$index]}"; then
        any_alive=1
        break
      fi
    done
    if [ "$any_alive" -eq 0 ]; then
      break
    fi
    sleep 1
  done

  for ((index = ${#child_pids[@]} - 1; index >= 0; index--)); do
    if process_group_is_alive "${child_pids[$index]}"; then
      log "SIGKILL -> ${child_names[$index]}"
      kill -KILL -- "-${child_pids[$index]}" 2>/dev/null || true
    fi
  done
  for index in "${!child_pids[@]}"; do
    if [ -n "${child_pids[$index]}" ]; then
      wait "${child_pids[$index]}" 2>/dev/null || true
    fi
  done
  for index in "${!child_pids[@]}"; do
    if process_group_is_alive "${child_pids[$index]}"; then
      log "WARNING: process group still exists for ${child_names[$index]}" >&2
    fi
  done
}

on_exit()
{
  local exit_code=$?
  trap - EXIT
  trap '' INT TERM HUP
  shutdown_children
  exit "$exit_code"
}

on_signal()
{
  local signal_name="$1"
  local state_output
  local stop_is_safe=0

  if [ "$external_vision_active" -eq 1 ]; then
    state_output="$(capture_one /mavros/state mavros_msgs/msg/State || true)"
    if printf '%s\n' "$state_output" |
      grep -Eq '^[[:space:]]*armed: false[[:space:]]*$'; then
      stop_is_safe=1
    fi
    if [ "$stop_is_safe" -eq 0 ]; then
      if [ "$armed_stop_warning_issued" -eq 0 ]; then
        armed_stop_warning_issued=1
        log "STOP: PX4 disarmed state was not confirmed" >&2
        log "Disarm PX4 before requesting shutdown again" >&2
        log "A repeated signal forces shutdown for emergency recovery" >&2
        return
      fi
      log "WARNING: forcing shutdown without confirmed PX4 disarmed state" >&2
    fi
  fi
  log "Received ${signal_name}"
  exit 130
}

trap on_exit EXIT
trap 'on_signal SIGINT' INT
trap 'on_signal SIGTERM' TERM
trap 'on_signal SIGHUP' HUP

acquire_lock()
{
  command -v flock >/dev/null 2>&1 || stop "required command not found: flock"
  exec {lock_fd}>"$LOCK_FILE" || stop "cannot open launcher lock ${LOCK_FILE}"
  flock -n "$lock_fd" || stop "another ${SCRIPT_NAME} instance owns ${LOCK_FILE}"
}

source_ros_environment()
{
  [ -r "$ROS_SETUP_FILE" ] || stop "missing ${ROS_SETUP_FILE}"
  [ -r "$WORKSPACE_SETUP_FILE" ] || stop "missing ${WORKSPACE_SETUP_FILE}"

  unset PYTHONHOME
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP_FILE"
  # shellcheck disable=SC1090
  source "$WORKSPACE_SETUP_FILE"
  set -Eeuo pipefail

  if [ -n "${ROS_DOMAIN_ID:-}" ] && [ "$ROS_DOMAIN_ID" != "42" ]; then
    stop "ROS_DOMAIN_ID must be 42; got ${ROS_DOMAIN_ID}"
  fi
  if [ -n "${RMW_IMPLEMENTATION:-}" ] &&
    [ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]
  then
    stop "RMW_IMPLEMENTATION must be rmw_fastrtps_cpp; got ${RMW_IMPLEMENTATION}"
  fi
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export PYTHONUNBUFFERED=1
}

require_command()
{
  command -v "$1" >/dev/null 2>&1 || stop "required command not found: $1"
}

bounded()
{
  timeout --signal=INT --kill-after=1s "${PROBE_TIMEOUT_SEC}s" "$@"
}

diagnostic_bounded()
{
  timeout --signal=INT --kill-after=1s \
    "${DIAGNOSTIC_PROBE_TIMEOUT_SEC}s" "$@"
}

topic_count_from_info()
{
  local output="$1"
  local endpoint_kind="$2"
  printf '%s\n' "$output" | awk -v endpoint_kind="$endpoint_kind" '
    $1 == endpoint_kind && $2 == "count:" {print $3; exit}
  '
}

endpoint_exists_in_info()
{
  local output="$1"
  local expected_name="$2"
  local expected_namespace="$3"
  local expected_type="$4"
  local expected_endpoint="$5"

  printf '%s\n' "$output" | awk \
    -v expected_name="$expected_name" \
    -v expected_namespace="$expected_namespace" \
    -v expected_type="$expected_type" \
    -v expected_endpoint="$expected_endpoint" '
      /^Node name:/ {
        name = $3
        node_namespace = ""
        type = ""
        endpoint = ""
      }
      /^Node namespace:/ {node_namespace = $3}
      /^Topic type:/ {type = $3}
      /^Endpoint type:/ {endpoint = $3}
      /^QoS profile:/ {
        if (name == expected_name &&
            node_namespace == expected_namespace &&
            type == expected_type &&
            endpoint == expected_endpoint) {
          found = 1
        }
      }
      END {exit found ? 0 : 1}
    '
}

preflight_packages()
{
  local package
  for package in \
    realsense2_camera \
    vrpn_client_ros \
    mocap_localization_adapter \
    localization_source_selector \
    localization_output_gateway \
    localization_adapter_interfaces \
    mavros_msgs
  do
    ros2 pkg prefix "$package" >/dev/null 2>&1 ||
      stop "ROS package not found after sourcing overlay: ${package}"
  done
}

topic_publisher_count()
{
  local topic="$1"
  local output
  local count
  output="$(bounded ros2 topic info "$topic" 2>/dev/null)" || return 1
  count="$(topic_count_from_info "$output" Publisher)"
  [[ "$count" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$count"
}

alternate_mavros_inputs_are_clear()
{
  local topic
  local topic_list
  local count
  topic_list="$(bounded ros2 topic list 2>/dev/null)" || return 1
  for topic in \
    /mavros/vision_pose/pose \
    /mavros/mocap/pose \
    /mavros/odometry/out \
    /mavros/vision_speed/speed_twist \
    /mavros/vision_speed/speed_twist_cov \
    /mavros/vision_speed/speed_vector
  do
    if ! printf '%s\n' "$topic_list" | grep -Fxq "$topic"; then
      continue
    fi
    count="$(topic_publisher_count "$topic")" || return 1
    [ "$count" = "0" ] || return 1
  done
}

assert_no_existing_runtime()
{
  local node
  local topic
  local count
  local node_list
  local topic_list
  node_list="$(bounded ros2 node list 2>/dev/null)" ||
    stop "unable to inspect the ROS node graph"
  topic_list="$(bounded ros2 topic list 2>/dev/null)" ||
    stop "unable to inspect the ROS topic graph"

  for node in \
    /camera/camera \
    /aligned_fcu_imu_relay \
    /d435i_cuvslam_runtime_health_monitor \
    /visual_slam_launch_container \
    /visual_slam_node \
    /vrpn_client_node \
    /mocap_localization_adapter \
    /localization_source_selector \
    /localization_output_gateway
  do
    if printf '%s\n' "$node_list" | grep -Fxq "$node"; then
      stop "ROS node is already running: ${node}"
    fi
  done

  for topic in \
    "$YOPO_DEPTH_TOPIC" \
    /droneyee207/pose \
    /localization/candidates/mocap/base_pose \
    /localization/selected/pose \
    /mavros/vision_pose/pose_cov \
    /mavros/vision_pose/pose \
    /mavros/mocap/pose \
    /mavros/odometry/out \
    /mavros/vision_speed/speed_twist \
    /mavros/vision_speed/speed_twist_cov \
    /mavros/vision_speed/speed_vector
  do
    if ! printf '%s\n' "$topic_list" | grep -Fxq "$topic"; then
      continue
    fi
    count="$(topic_publisher_count "$topic")" ||
      stop "unable to inspect publishers for protected topic ${topic}"
    if [ "$count" != "0" ]; then
      stop "protected topic already has ${count} publisher(s): ${topic}"
    fi
  done
}

capture_one()
{
  local topic="$1"
  local type="$2"
  bounded ros2 topic echo "$topic" "$type" --once --no-arr 2>/dev/null
}

depth_message_is_valid()
{
  local output="$1"
  local height
  local width
  local encoding
  local frame_id
  local step

  height="$(printf '%s\n' "$output" | awk '/^height:/ {print $2; exit}')"
  width="$(printf '%s\n' "$output" | awk '/^width:/ {print $2; exit}')"
  encoding="$(printf '%s\n' "$output" | awk '/^encoding:/ {print $2; exit}')"
  frame_id="$(printf '%s\n' "$output" |
    awk '$1 == "frame_id:" {print $2; exit}')"
  step="$(printf '%s\n' "$output" | awk '/^step:/ {print $2; exit}')"
  [ "$height" = "360" ] &&
    [ "$width" = "640" ] &&
    [ "$encoding" = "16UC1" ] &&
    [ "$frame_id" = "camera_depth_optical_frame" ] &&
    [ "$step" = "1280" ]
}

camera_parameters_are_valid()
{
  local value

  value="$(bounded ros2 param get /camera/camera enable_depth 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: True" ] || return 1
  value="$(bounded ros2 param get /camera/camera enable_color 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: False" ] || return 1
  value="$(bounded ros2 param get /camera/camera enable_infra1 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: False" ] || return 1
  value="$(bounded ros2 param get /camera/camera enable_infra2 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: False" ] || return 1
  value="$(bounded ros2 param get /camera/camera enable_gyro 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: False" ] || return 1
  value="$(bounded ros2 param get /camera/camera enable_accel 2>/dev/null)" || return 1
  [ "$value" = "Boolean value is: False" ] || return 1
  value="$(bounded ros2 param get \
    /camera/camera depth_module.emitter_enabled 2>/dev/null)" || return 1
  [ "$value" = "Integer value is: 0" ] || return 1
  value="$(bounded ros2 param get \
    /camera/camera depth_module.profile 2>/dev/null)" || return 1
  [ "$value" = "String value is: 640x360x90" ]
}

depth_rate_is_valid()
{
  local output
  local rate

  output="$(timeout --signal=INT --kill-after=1s \
    "${DEPTH_RATE_PROBE_SEC}s" ros2 topic hz "$YOPO_DEPTH_TOPIC" \
    2>/dev/null || true)"
  rate="$(printf '%s\n' "$output" |
    awk '/^average rate:/ {rate = $3} END {print rate}')"
  last_depth_rate_hz="${rate:-unavailable}"
  [[ "$rate" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 1
  awk \
    -v rate="$rate" \
    -v minimum="$MINIMUM_DEPTH_RATE_HZ" \
    -v maximum="$MAXIMUM_DEPTH_RATE_HZ" \
    'BEGIN {exit !(rate >= minimum && rate <= maximum)}' || return 1
}

wait_for_depth_ready()
{
  local child_index="$1"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local message
  local endpoint

  log "Waiting for D435 depth: ${YOPO_DEPTH_TOPIC}"
  while [ "$SECONDS" -lt "$deadline" ]; do
    ensure_child_alive \
      "${child_pids[$child_index]}" \
      "${child_names[$child_index]}" \
      "${child_logs[$child_index]}"
    endpoint="$(bounded ros2 topic info -v "$YOPO_DEPTH_TOPIC" 2>/dev/null || true)"
    message="$(capture_one "$YOPO_DEPTH_TOPIC" sensor_msgs/msg/Image || true)"
    if [ "$(topic_count_from_info "$endpoint" Publisher)" = "1" ] &&
      endpoint_exists_in_info \
        "$endpoint" camera /camera sensor_msgs/msg/Image PUBLISHER &&
      depth_message_is_valid "$message" &&
      camera_parameters_are_valid &&
      depth_rate_is_valid
    then
      log "D435 depth is ready (640x360, 16UC1, ${last_depth_rate_hz} Hz," \
        "emitter disabled)"
      return
    fi
    sleep 1
  done
  show_child_log \
    "${child_names[$child_index]}" "${child_logs[$child_index]}"
  stop "D435 depth did not satisfy the runtime contract within ${STARTUP_TIMEOUT_SEC}s"
}

wait_for_mavros_graph()
{
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local state_info
  local timesync_info
  local output_info
  local state_publishers
  local timesync_publishers
  local output_publishers
  local output_subscriptions

  log "Waiting for the host MAVROS graph"
  while [ "$SECONDS" -lt "$deadline" ]; do
    state_info="$(bounded ros2 topic info -v /mavros/state 2>/dev/null || true)"
    timesync_info="$(bounded ros2 topic info -v \
      /mavros/timesync_status 2>/dev/null || true)"
    output_info="$(bounded ros2 topic info -v \
      /mavros/vision_pose/pose_cov 2>/dev/null || true)"
    state_publishers="$(topic_count_from_info "$state_info" Publisher)"
    timesync_publishers="$(topic_count_from_info "$timesync_info" Publisher)"
    output_publishers="$(topic_count_from_info "$output_info" Publisher)"
    output_subscriptions="$(topic_count_from_info "$output_info" Subscription)"

    if [ "$state_publishers" = "1" ] &&
      endpoint_exists_in_info \
        "$state_info" sys /mavros mavros_msgs/msg/State PUBLISHER &&
      [ "$timesync_publishers" = "1" ] &&
      endpoint_exists_in_info \
        "$timesync_info" time /mavros mavros_msgs/msg/TimesyncStatus PUBLISHER &&
      [ "$output_publishers" = "0" ] &&
      [ "$output_subscriptions" = "1" ] &&
      endpoint_exists_in_info \
        "$output_info" vision_pose /mavros \
        geometry_msgs/msg/PoseWithCovarianceStamped SUBSCRIPTION
    then
      log "Host MAVROS graph is ready"
      return
    fi
    sleep 1
  done
  stop "host MAVROS graph did not become ready within ${STARTUP_TIMEOUT_SEC}s"
}

mavros_tf_input_is_disabled()
{
  local output
  output="$(bounded ros2 param get /mavros/vision_pose tf/listen 2>/dev/null)" ||
    return 1
  printf '%s\n' "$output" | grep -Fxq 'Boolean value is: False'
}

verify_mavros_input_configuration()
{
  mavros_tf_input_is_disabled ||
    stop "/mavros/vision_pose tf/listen must exist and be false"
  log "MAVROS alternate vision-pose TF input is disabled"
}

wait_for_mavros_state()
{
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local state_output
  local mode

  log "Waiting for MAVROS connected, PX4 disarmed, and non-OFFBOARD mode"
  while [ "$SECONDS" -lt "$deadline" ]; do
    state_output="$(capture_one /mavros/state mavros_msgs/msg/State || true)"
    if printf '%s\n' "$state_output" |
      grep -Eq '^[[:space:]]*connected: true[[:space:]]*$'
    then
      mode="$(printf '%s\n' "$state_output" |
        awk '/^[[:space:]]*mode:/ {print $2; exit}')"
      if [ -z "$mode" ]; then
        sleep 1
        continue
      fi
      if printf '%s\n' "$state_output" |
        grep -Eq '^[[:space:]]*armed: true[[:space:]]*$'
      then
        stop "PX4 is armed; disarm before starting the localization chain"
      fi
      if [ "$mode" = "OFFBOARD" ]; then
        stop "PX4 is already in OFFBOARD; change mode before startup"
      fi
      if printf '%s\n' "$state_output" |
        grep -Eq '^[[:space:]]*armed: false[[:space:]]*$'
      then
        log "MAVROS state is ready"
        return
      fi
    fi
    sleep 1
  done
  stop "MAVROS did not report a connected, disarmed state within ${STARTUP_TIMEOUT_SEC}s"
}

wait_for_valid_timesync()
{
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local output
  local rtt

  log "Waiting for a valid MAVROS timesync sample"
  while [ "$SECONDS" -lt "$deadline" ]; do
    output="$(capture_one /mavros/timesync_status \
      mavros_msgs/msg/TimesyncStatus || true)"
    rtt="$(printf '%s\n' "$output" |
      awk '/^[[:space:]]*round_trip_time_ms:/ {print $2; exit}')"
    if [[ "$rtt" =~ ^[+]?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$ ]]; then
      log "MAVROS timesync is ready (RTT ${rtt} ms)"
      return
    fi
    sleep 1
  done
  stop "no finite non-negative MAVROS timesync RTT received within ${STARTUP_TIMEOUT_SEC}s"
}

show_child_log()
{
  local name="$1"
  local file="$2"
  printf '\n========== %s log tail ==========\n' "$name" >&2
  tail -n 80 "$file" >&2 || true
}

start_launch()
{
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  local pid

  : > "$log_file"
  pending_signal=""
  trap 'pending_signal=SIGINT' INT
  trap 'pending_signal=SIGTERM' TERM
  trap 'pending_signal=SIGHUP' HUP
  setsid env --default-signal=INT,QUIT "$@" > "$log_file" 2>&1 &
  pid=$!
  child_names+=("$name")
  child_pids+=("$pid")
  child_logs+=("$log_file")
  child_exit_reported+=(0)
  trap 'on_signal SIGINT' INT
  trap 'on_signal SIGTERM' TERM
  trap 'on_signal SIGHUP' HUP
  if [ -n "$pending_signal" ]; then
    on_signal "$pending_signal"
  fi
  log "Started ${name} (PID ${pid}, log ${log_file})"

  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    show_child_log "$name" "$log_file"
    stop "${name} exited during startup"
  fi
}

ensure_child_alive()
{
  local pid="$1"
  local name="$2"
  local log_file="$3"
  if kill -0 "$pid" 2>/dev/null; then
    return
  fi
  show_child_log "$name" "$log_file"
  stop "${name} exited before its output became ready"
}

wait_for_topic_message()
{
  local label="$1"
  local topic="$2"
  local type="$3"
  local child_index="$4"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))

  log "Waiting for ${label}: ${topic}"
  while [ "$SECONDS" -lt "$deadline" ]; do
    ensure_child_alive \
      "${child_pids[$child_index]}" \
      "${child_names[$child_index]}" \
      "${child_logs[$child_index]}"
    if capture_one "$topic" "$type" >/dev/null; then
      log "Ready: ${label}"
      return
    fi
    sleep 1
  done
  show_child_log "${child_names[$child_index]}" "${child_logs[$child_index]}"
  stop "${label} did not become ready within ${STARTUP_TIMEOUT_SEC}s"
}

wait_for_gateway_healthy()
{
  local child_index="$1"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local output
  local selector_output
  local state
  local reason
  local selector_contract_id
  local gateway_contract_id

  log "Waiting for active gateway diagnostics"
  while [ "$SECONDS" -lt "$deadline" ]; do
    ensure_child_alive \
      "${child_pids[$child_index]}" \
      "${child_names[$child_index]}" \
      "${child_logs[$child_index]}"
    selector_output="$(capture_diagnostic localization_source_selector || true)"
    output="$(capture_diagnostic localization_output_gateway || true)"
    state="$(diagnostic_value "$output" state)"
    reason="$(diagnostic_value "$output" reason_code)"
    selector_contract_id="$(diagnostic_value "$selector_output" selector_contract_id)"
    gateway_contract_id="$(diagnostic_value "$output" gateway_contract_id)"
    if [ -n "$selector_contract_id" ] &&
      [ "$selector_contract_id" != "$EXPECTED_SELECTOR_CONTRACT_ID" ]; then
      printf '%s\n' "$selector_output" >&2
      stop "selector contract is not the required global-mocap v2 contract"
    fi
    if [ -n "$gateway_contract_id" ] &&
      [ "$gateway_contract_id" != "$EXPECTED_GATEWAY_CONTRACT_ID" ]; then
      printf '%s\n' "$output" >&2
      stop "gateway contract is not the required global-mocap v2 contract"
    fi
    if selector_global_identity_is_valid "$selector_output" &&
      gateway_v2_contract_is_valid "$output" &&
      [ "$state" = "active_healthy" ] &&
      [ "$reason" = "EXTERNAL_VISION_PUBLISHED" ]; then
      last_gateway_published="$(diagnostic_value "$output" published)"
      if [[ "$last_gateway_published" =~ ^[1-9][0-9]*$ ]]; then
        log "Global-mocap v2 selector and gateway are publishing external-vision pose"
        return
      fi
    fi
    if [ "$state" = "latched_fault" ]; then
      printf '%s\n' "$output" >&2
      stop "gateway contract fault is latched"
    fi
    sleep 1
  done
  printf '%s\n' "$selector_output" >&2
  printf '%s\n' "$output" >&2
  show_child_log "${child_names[$child_index]}" "${child_logs[$child_index]}"
  stop "global-mocap v2 selector/gateway did not become healthy within ${STARTUP_TIMEOUT_SEC}s"
}

diagnostic_value()
{
  local output="$1"
  local expected_key="$2"
  printf '%s\n' "$output" | awk -v expected_key="$expected_key" '
    $1 == "-" && $2 == "key:" {key = $3; next}
    $1 == "value:" && key == expected_key {
      value = $2
      gsub(/\047/, "", value)
      print value
      exit
    }
  '
}

diagnostic_value_is_zero()
{
  [[ "$1" =~ ^-?0+([.][0]+)?$ ]]
}

selector_global_identity_is_valid()
{
  local output="$1"
  [ "$(diagnostic_value "$output" state)" = "healthy" ] &&
    [ "$(diagnostic_value "$output" reason_code)" = "SOURCE_HEALTHY" ] &&
    [ "$(diagnostic_value "$output" selector_contract_id)" = \
      "$EXPECTED_SELECTOR_CONTRACT_ID" ] &&
    [ "$(diagnostic_value "$output" pose_reference_semantics)" = \
      "global_mocap_world" ] &&
    [ "$(diagnostic_value "$output" alignment_locked)" = "1" ] &&
    diagnostic_value_is_zero \
      "$(diagnostic_value "$output" alignment_yaw_map_from_source_rad)" &&
    diagnostic_value_is_zero \
      "$(diagnostic_value "$output" alignment_translation_x_m)" &&
    diagnostic_value_is_zero \
      "$(diagnostic_value "$output" alignment_translation_y_m)" &&
    diagnostic_value_is_zero \
      "$(diagnostic_value "$output" alignment_translation_z_m)"
}

gateway_v2_contract_is_valid()
{
  local output="$1"
  [ "$(diagnostic_value "$output" gateway_contract_id)" = \
    "$EXPECTED_GATEWAY_CONTRACT_ID" ] &&
    [ "$(diagnostic_value "$output" expected_selector_contract_id)" = \
      "$EXPECTED_SELECTOR_CONTRACT_ID" ]
}

capture_diagnostic()
{
  local node_fragment="$1"
  diagnostic_bounded ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
    --once \
    --filter "any('${node_fragment}' in s.name for s in m.status)" \
    2>/dev/null
}

runtime_health_fail()
{
  runtime_health_failure="$1"
  return 1
}

runtime_health_check()
{
  local node
  local node_list
  local adapter_output
  local selector_output
  local gateway_output
  local depth_endpoint
  local depth_message
  local endpoint_output
  local published

  runtime_health_failure="unknown"
  node_list="$(bounded ros2 node list 2>/dev/null)" ||
    runtime_health_fail "ROS node graph query timed out" || return 1
  for node in \
    /camera/camera \
    /vrpn_client_node \
    /mocap_localization_adapter \
    /localization_source_selector \
    /localization_output_gateway
  do
    [ "$(printf '%s\n' "$node_list" | grep -Fxc "$node")" = "1" ] ||
      runtime_health_fail "required node is missing or duplicated: ${node}" || return 1
  done

  for node in \
    /aligned_fcu_imu_relay \
    /d435i_cuvslam_runtime_health_monitor \
    /visual_slam_launch_container \
    /visual_slam_node
  do
    if printf '%s\n' "$node_list" | grep -Fxq "$node"; then
      runtime_health_fail "forbidden node is running: ${node}"
      return 1
    fi
  done

  depth_endpoint="$(bounded ros2 topic info -v \
    "$YOPO_DEPTH_TOPIC" 2>/dev/null)" ||
    runtime_health_fail "depth endpoint query timed out: ${YOPO_DEPTH_TOPIC}" || return 1
  [ "$(topic_count_from_info "$depth_endpoint" Publisher)" = "1" ] ||
    runtime_health_fail "depth publisher count is not one: ${YOPO_DEPTH_TOPIC}" || return 1
  endpoint_exists_in_info \
    "$depth_endpoint" camera /camera sensor_msgs/msg/Image PUBLISHER ||
    runtime_health_fail "depth publisher identity or type changed" || return 1
  depth_message="$(capture_one \
    "$YOPO_DEPTH_TOPIC" sensor_msgs/msg/Image)" ||
    runtime_health_fail "depth sample was not received before timeout" || return 1
  depth_message_is_valid "$depth_message" ||
    runtime_health_fail "depth format, dimensions, frame, or step changed" || return 1
  depth_rate_is_valid ||
    runtime_health_fail \
      "depth rate ${last_depth_rate_hz} Hz is outside configured range" ||
    return 1

  adapter_output="$(capture_diagnostic mocap_localization_adapter)" ||
    runtime_health_fail "mocap adapter diagnostic timed out" || return 1
  selector_output="$(capture_diagnostic localization_source_selector)" ||
    runtime_health_fail "selector diagnostic timed out" || return 1
  gateway_output="$(capture_diagnostic localization_output_gateway)" ||
    runtime_health_fail "gateway diagnostic timed out" || return 1

  [ "$(diagnostic_value "$adapter_output" health_state)" = "healthy" ] ||
    runtime_health_fail "mocap adapter is not healthy" || return 1
  [ "$(diagnostic_value "$selector_output" state)" = "healthy" ] ||
    runtime_health_fail "selector is not healthy" || return 1
  [ "$(diagnostic_value "$selector_output" reason_code)" = "SOURCE_HEALTHY" ] ||
    runtime_health_fail "selector reason is not SOURCE_HEALTHY" || return 1
  selector_global_identity_is_valid "$selector_output" ||
    runtime_health_fail "selector global-mocap v2 identity semantics are invalid" || return 1
  [ "$(diagnostic_value "$gateway_output" state)" = "active_healthy" ] ||
    runtime_health_fail "gateway is not active_healthy" || return 1
  [ "$(diagnostic_value "$gateway_output" reason_code)" = \
    "EXTERNAL_VISION_PUBLISHED" ] ||
    runtime_health_fail "gateway is not publishing external vision" || return 1
  gateway_v2_contract_is_valid "$gateway_output" ||
    runtime_health_fail "gateway global-mocap v2 contract is invalid" || return 1

  published="$(diagnostic_value "$gateway_output" published)"
  [[ "$published" =~ ^[1-9][0-9]*$ ]] ||
    runtime_health_fail "gateway published counter is invalid" || return 1
  [[ "$last_gateway_published" =~ ^[1-9][0-9]*$ ]] ||
    runtime_health_fail "previous gateway published counter is invalid" || return 1
  [ "$published" -gt "$last_gateway_published" ] ||
    runtime_health_fail "gateway published counter did not increase" || return 1
  last_gateway_published="$published"

  endpoint_output="$(bounded ros2 topic info -v \
    /mavros/vision_pose/pose_cov 2>/dev/null)" ||
    runtime_health_fail "MAVROS external-vision endpoint query timed out" || return 1
  [ "$(topic_count_from_info "$endpoint_output" Publisher)" = "1" ] ||
    runtime_health_fail "external-vision publisher count is not one" || return 1
  endpoint_exists_in_info \
    "$endpoint_output" localization_output_gateway / \
    geometry_msgs/msg/PoseWithCovarianceStamped PUBLISHER ||
    runtime_health_fail "external-vision publisher identity changed" || return 1
  endpoint_exists_in_info \
    "$endpoint_output" vision_pose /mavros \
    geometry_msgs/msg/PoseWithCovarianceStamped SUBSCRIPTION ||
    runtime_health_fail "MAVROS vision_pose subscription is missing" || return 1
  alternate_mavros_inputs_are_clear ||
    runtime_health_fail "an alternate MAVROS vision input has a publisher" || return 1
  mavros_tf_input_is_disabled ||
    runtime_health_fail "MAVROS vision_pose TF input is not disabled" || return 1

  runtime_health_failure="none"
}

verify_external_vision_endpoint()
{
  local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
  local output=""

  log "Verifying the unique MAVROS external-vision endpoint"
  while [ "$SECONDS" -lt "$deadline" ]; do
    output="$(bounded ros2 topic info -v \
      /mavros/vision_pose/pose_cov 2>/dev/null || true)"
    if [ "$(topic_count_from_info "$output" Publisher)" = "1" ] &&
      [ "$(topic_count_from_info "$output" Subscription)" = "1" ] &&
      endpoint_exists_in_info \
        "$output" localization_output_gateway / \
        geometry_msgs/msg/PoseWithCovarianceStamped PUBLISHER &&
      endpoint_exists_in_info \
        "$output" vision_pose /mavros \
        geometry_msgs/msg/PoseWithCovarianceStamped SUBSCRIPTION
    then
      log "External-vision endpoint is unique (1 publisher, 1 MAVROS subscriber)"
      return
    fi
    sleep 1 || true
  done
  printf '%s\n' "$output" >&2
  stop "external-vision endpoint authority was not verified"
}

monitor_children()
{
  local index
  local exit_code
  local health_failures=0
  local runtime_fault_reported=0

  set +e

  log "D435 depth and all container-side localization nodes are ready"
  log "Logs: ${LOG_DIR}"
  log "This confirms depth and localization transport, not YOPO or flight readiness"
  log "The deferred 60-second timesync RTT/jitter acceptance test is not performed"
  log "Keep this terminal open; press Ctrl-C only after PX4 is disarmed"
  setsid tail -n 0 -F "${child_logs[@]}" &
  tail_pid=$!

  while true; do
    for index in "${!child_pids[@]}"; do
      if [ -z "${child_pids[$index]}" ]; then
        continue
      fi
      if ! kill -0 "${child_pids[$index]}" 2>/dev/null; then
        if [ "${child_names[$index]}" = "d435_depth" ] &&
          [ "$external_vision_active" -eq 1 ]
        then
          if [ "${child_exit_reported[$index]}" -eq 0 ]; then
            child_exit_reported[$index]=1
            wait "${child_pids[$index]}"
            exit_code=$?
            child_pids[$index]=""
            show_child_log "${child_names[$index]}" "${child_logs[$index]}"
            log "FAULT: D435 depth exited with code ${exit_code};" \
              "localization remains running" >&2
          fi
          continue
        fi
        if wait "${child_pids[$index]}"; then
          exit_code=0
        else
          exit_code=$?
        fi
        show_child_log "${child_names[$index]}" "${child_logs[$index]}"
        stop "${child_names[$index]} exited unexpectedly with code ${exit_code}"
      fi
    done
    if runtime_health_check; then
      if [ "$runtime_fault_reported" -eq 1 ]; then
        log "Runtime health recovered"
      fi
      health_failures=0
      runtime_fault_reported=0
    else
      health_failures=$((health_failures + 1))
      log "WARNING: runtime health probe failed (${health_failures}/3):" \
        "${runtime_health_failure}" >&2
      if [ "$health_failures" -ge 3 ] && [ "$runtime_fault_reported" -eq 0 ]; then
        runtime_fault_reported=1
        log "FAULT: localization runtime is not healthy; land and disarm" >&2
        log "The ROS nodes remain running so this supervisor does not remove pose while armed" >&2
      fi
    fi
    sleep 5
  done
}

main()
{
  local camera_index
  local vrpn_index
  local adapter_index
  local gateway_index

  if [ "$#" -gt 0 ]; then
    case "$1" in
      -h|--help)
        usage
        return 0
        ;;
      *)
        usage >&2
        stop "unexpected argument: $1"
        ;;
    esac
  fi

  validate_positive_integer STARTUP_TIMEOUT_SEC "$STARTUP_TIMEOUT_SEC"
  validate_positive_integer PROBE_TIMEOUT_SEC "$PROBE_TIMEOUT_SEC"
  validate_positive_integer \
    DIAGNOSTIC_PROBE_TIMEOUT_SEC "$DIAGNOSTIC_PROBE_TIMEOUT_SEC"
  validate_positive_integer DEPTH_RATE_PROBE_SEC "$DEPTH_RATE_PROBE_SEC"
  validate_positive_integer MINIMUM_DEPTH_RATE_HZ "$MINIMUM_DEPTH_RATE_HZ"
  validate_positive_integer MAXIMUM_DEPTH_RATE_HZ "$MAXIMUM_DEPTH_RATE_HZ"
  [ "$MINIMUM_DEPTH_RATE_HZ" -lt "$MAXIMUM_DEPTH_RATE_HZ" ] ||
    stop "depth-rate range must be increasing"
  validate_positive_integer SHUTDOWN_TIMEOUT_SEC "$SHUTDOWN_TIMEOUT_SEC"
  [ -n "$D435_SERIAL" ] || stop "D435_SERIAL must not be empty"
  [[ "$YOPO_DEPTH_TOPIC" = /* ]] ||
    stop "YOPO_DEPTH_TOPIC must be absolute; got ${YOPO_DEPTH_TOPIC}"
  acquire_lock
  source_ros_environment

  require_command ros2
  require_command timeout
  require_command setsid
  require_command env
  require_command awk
  require_command grep
  require_command tail

  [ -d "$WORKSPACE_ROOT" ] || stop "not inside expected container workspace: ${WORKSPACE_ROOT}"
  cd "$WORKSPACE_ROOT"

  preflight_packages
  assert_no_existing_runtime
  wait_for_mavros_graph
  verify_mavros_input_configuration
  wait_for_mavros_state
  wait_for_valid_timesync

  readonly LOG_DIR="${MOCAP_PRIMARY_LOG_DIR:-${WORKSPACE_ROOT}/log/mocap_primary_launcher/$(date -u +%Y%m%dT%H%M%SZ)}"
  mkdir -p "$LOG_DIR"

  camera_index="${#child_pids[@]}"
  start_launch d435_depth \
    ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -r __node:=camera \
    -r __ns:=/camera \
    -r "/camera/depth/image_rect_raw:=${YOPO_DEPTH_TOPIC}" \
    -p "serial_no:='${D435_SERIAL}'" \
    -p enable_depth:=true \
    -p enable_color:=false \
    -p enable_infra1:=false \
    -p enable_infra2:=false \
    -p enable_gyro:=false \
    -p enable_accel:=false \
    -p unite_imu_method:=0 \
    -p depth_module.emitter_enabled:=0 \
    -p "depth_module.profile:='640x360x90'" \
    -p publish_tf:=true
  wait_for_depth_ready "$camera_index"

  vrpn_index="${#child_pids[@]}"
  start_launch vrpn \
    ros2 launch vrpn_client_ros sample.launch.py
  wait_for_topic_message \
    "raw mocap pose" \
    /droneyee207/pose \
    geometry_msgs/msg/PoseStamped \
    "$vrpn_index"

  adapter_index="${#child_pids[@]}"
  start_launch mocap_adapter \
    ros2 launch mocap_localization_adapter mocap_adapter_shadow.launch.py
  wait_for_topic_message \
    "mocap source candidate" \
    /localization/candidates/mocap/base_pose \
    localization_adapter_interfaces/msg/LocalizationSourceCandidate \
    "$adapter_index"

  gateway_index="${#child_pids[@]}"
  start_launch selector_gateway \
    ros2 launch localization_output_gateway mocap_primary_output.launch.py
  wait_for_topic_message \
    "selected pose" \
    /localization/selected/pose \
    localization_adapter_interfaces/msg/SelectedPoseCandidate \
    "$gateway_index"
  wait_for_gateway_healthy "$gateway_index"
  external_vision_active=1
  verify_external_vision_endpoint
  monitor_children
}

main "$@"
