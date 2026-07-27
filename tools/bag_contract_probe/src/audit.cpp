// Copyright 2026 u5-4
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "audit.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace bag_contract_probe
{
namespace detail
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr char kMocapDiagnosticName[] =
  "/mocap_localization_adapter: mocap shadow contract";
constexpr char kAlignedImuDiagnosticName[] =
  "/aligned_fcu_imu_relay: aligned FCU IMU";
constexpr char kRuntimeDiagnosticName[] =
  "/d435i_cuvslam_runtime_health_monitor: calibrated runtime";

using DiagnosticValues = std::map<std::string, std::string>;

bool IsRequiredDiagnosticName(const std::string & name)
{
  return name == kMocapDiagnosticName || name == kAlignedImuDiagnosticName ||
         name == kRuntimeDiagnosticName;
}

std::optional<std::int64_t> StampNanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return std::nullopt;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds > std::numeric_limits<std::int64_t>::max() / kNanosecondsPerSecond) {
    return std::nullopt;
  }
  const std::int64_t stamp_ns = seconds * kNanosecondsPerSecond +
    static_cast<std::int64_t>(stamp.nanosec);
  if (stamp_ns <= 0) {
    return std::nullopt;
  }
  return stamp_ns;
}

template<typename ContainerT>
bool AllFinite(const ContainerT & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

bool QuaternionValid(const std::array<double, 4> & quaternion_xyzw)
{
  if (!AllFinite(quaternion_xyzw)) {
    return false;
  }
  long double squared_norm = 0.0L;
  for (const double value : quaternion_xyzw) {
    squared_norm += static_cast<long double>(value) * static_cast<long double>(value);
  }
  return squared_norm > 1.0e-24L;
}

PoseFields ToPoseFields(const geometry_msgs::msg::Pose & pose)
{
  return PoseFields{
    {pose.position.x, pose.position.y, pose.position.z},
    {pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w}};
}

bool PoseValid(const PoseFields & pose)
{
  return AllFinite(pose.position) && QuaternionValid(pose.orientation_xyzw);
}

bool PoseEquivalent(const PoseFields & raw, const PoseFields & shadow)
{
  for (std::size_t index = 0U; index < raw.position.size(); ++index) {
    if (std::abs(raw.position[index] - shadow.position[index]) > 1.0e-12) {
      return false;
    }
  }

  long double raw_norm_squared = 0.0L;
  long double shadow_norm_squared = 0.0L;
  long double dot = 0.0L;
  for (std::size_t index = 0U; index < raw.orientation_xyzw.size(); ++index) {
    raw_norm_squared += static_cast<long double>(raw.orientation_xyzw[index]) *
      static_cast<long double>(raw.orientation_xyzw[index]);
    shadow_norm_squared += static_cast<long double>(shadow.orientation_xyzw[index]) *
      static_cast<long double>(shadow.orientation_xyzw[index]);
    dot += static_cast<long double>(raw.orientation_xyzw[index]) *
      static_cast<long double>(shadow.orientation_xyzw[index]);
  }
  const long double denominator = std::sqrt(raw_norm_squared * shadow_norm_squared);
  if (denominator <= 1.0e-24L) {
    return false;
  }
  const long double normalized_dot = std::clamp(
    std::abs(dot) / denominator, 0.0L, 1.0L);
  const long double angular_distance = 2.0L * std::acos(normalized_dot);
  return angular_distance <= 1.0e-10L;
}

ImuFields ToImuFields(const sensor_msgs::msg::Imu & message)
{
  ImuFields fields;
  fields.orientation = {
    message.orientation.x, message.orientation.y,
    message.orientation.z, message.orientation.w};
  fields.angular_velocity = {
    message.angular_velocity.x, message.angular_velocity.y,
    message.angular_velocity.z};
  fields.linear_acceleration = {
    message.linear_acceleration.x, message.linear_acceleration.y,
    message.linear_acceleration.z};
  std::copy(
    message.orientation_covariance.begin(), message.orientation_covariance.end(),
    fields.orientation_covariance.begin());
  std::copy(
    message.angular_velocity_covariance.begin(),
    message.angular_velocity_covariance.end(),
    fields.angular_velocity_covariance.begin());
  std::copy(
    message.linear_acceleration_covariance.begin(),
    message.linear_acceleration_covariance.end(),
    fields.linear_acceleration_covariance.begin());
  return fields;
}

bool ImuFieldsValid(const ImuFields & fields)
{
  return AllFinite(fields.orientation) && AllFinite(fields.angular_velocity) &&
         AllFinite(fields.linear_acceleration) &&
         AllFinite(fields.orientation_covariance) &&
         AllFinite(fields.angular_velocity_covariance) &&
         AllFinite(fields.linear_acceleration_covariance);
}

void ObserveHeader(
  StreamAudit * stream,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & parent_frame)
{
  const auto stamp_ns = StampNanoseconds(stamp);
  stream->header_stamps_ns.push_back(stamp_ns.value_or(0));
  ++stream->parent_frames[parent_frame];
}

void PairRawPose(
  PosePairAudit * pairs, const std::int64_t stamp_ns, const PoseFields & raw)
{
  const auto shadow = pairs->pending_shadow.find(stamp_ns);
  if (shadow == pairs->pending_shadow.end() || shadow->second.empty()) {
    pairs->pending_raw[stamp_ns].push_back(raw);
    return;
  }
  ++pairs->paired;
  if (!PoseEquivalent(raw, shadow->second.front())) {
    ++pairs->value_mismatches;
  }
  shadow->second.pop_front();
  if (shadow->second.empty()) {
    pairs->pending_shadow.erase(shadow);
  }
}

void PairShadowPose(
  PosePairAudit * pairs, const std::int64_t stamp_ns, const PoseFields & shadow)
{
  const auto raw = pairs->pending_raw.find(stamp_ns);
  if (raw == pairs->pending_raw.end() || raw->second.empty()) {
    pairs->pending_shadow[stamp_ns].push_back(shadow);
    return;
  }
  ++pairs->paired;
  if (!PoseEquivalent(raw->second.front(), shadow)) {
    ++pairs->value_mismatches;
  }
  raw->second.pop_front();
  if (raw->second.empty()) {
    pairs->pending_raw.erase(raw);
  }
}

DiagnosticValues CollectDiagnosticValues(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  std::uint64_t * duplicate_keys)
{
  DiagnosticValues values;
  for (const auto & value : status.values) {
    if (!values.emplace(value.key, value.value).second) {
      ++(*duplicate_keys);
    }
  }
  return values;
}

std::optional<std::uint64_t> ParseUnsigned(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  std::uint64_t result = 0U;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    return std::nullopt;
  }
  return result;
}

std::optional<std::int64_t> ParseSigned(const std::string & text)
{
  if (text.empty()) {
    return std::nullopt;
  }
  std::int64_t result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    return std::nullopt;
  }
  return result;
}

bool ValueEquals(
  const DiagnosticValues & values,
  const std::string & key,
  const std::string & expected)
{
  const auto value = values.find(key);
  return value != values.end() && value->second == expected;
}

bool ValuesEqual(
  const DiagnosticValues & values,
  const std::initializer_list<std::pair<const char *, const char *>> expected)
{
  return std::all_of(
    expected.begin(), expected.end(), [&values](const auto & entry) {
      return ValueEquals(values, entry.first, entry.second);
    });
}

bool MocapHealthTupleMatches(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const DiagnosticValues & values)
{
  const auto health = values.find("health_state");
  const auto reason = values.find("reason_code");
  const auto latched = values.find("latched");
  if (health == values.end() || reason == values.end() || latched == values.end() ||
    reason->second.empty())
  {
    return false;
  }

  if (health->second == "starting") {
    return status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE &&
           status.message == "waiting for complete VRPN pose evidence" &&
           latched->second == "0";
  }
  if (health->second == "healthy") {
    return status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
           status.message ==
           "mocap input is healthy; output remains a shadow-only pose candidate" &&
           reason->second == "INPUT_HEALTHY" && latched->second == "0";
  }
  if (health->second == "transient_fault") {
    return status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
           status.message == "transient mocap shadow contract fault" &&
           latched->second == "0";
  }
  if (health->second == "recovering") {
    return status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
           status.message == "mocap pose input is recovering" &&
           latched->second == "0";
  }
  if (health->second == "latched_fault") {
    return status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
           status.message == "latched mocap contract fault; restart required" &&
           latched->second == "1";
  }
  return false;
}

void CollectCounters(
  const DiagnosticValues & values,
  const std::vector<std::string> & keys,
  std::map<std::string, std::vector<std::uint64_t>> * destination,
  std::uint64_t * malformed_samples)
{
  bool malformed = false;
  for (const auto & key : keys) {
    const auto value = values.find(key);
    if (value == values.end()) {
      malformed = true;
      continue;
    }
    const auto parsed = ParseUnsigned(value->second);
    if (!parsed.has_value()) {
      malformed = true;
      continue;
    }
    (*destination)[key].push_back(parsed.value());
  }
  if (malformed) {
    ++(*malformed_samples);
  }
}

std::optional<std::uint64_t> CounterValue(
  const DiagnosticValues & values, const std::string & key)
{
  const auto value = values.find(key);
  if (value == values.end()) {
    return std::nullopt;
  }
  return ParseUnsigned(value->second);
}

std::optional<std::uint64_t> SumCounters(
  const DiagnosticValues & values, const std::vector<std::string> & keys)
{
  std::uint64_t sum = 0U;
  for (const auto & key : keys) {
    const auto value = CounterValue(values, key);
    if (!value.has_value() || value.value() > std::numeric_limits<std::uint64_t>::max() - sum) {
      return std::nullopt;
    }
    sum += value.value();
  }
  return sum;
}

bool OdometryFinite(const nav_msgs::msg::Odometry & message)
{
  const auto pose = ToPoseFields(message.pose.pose);
  const std::array<double, 6> twist{
    message.twist.twist.linear.x, message.twist.twist.linear.y,
    message.twist.twist.linear.z, message.twist.twist.angular.x,
    message.twist.twist.angular.y, message.twist.twist.angular.z};
  return PoseValid(pose) && AllFinite(twist) &&
         AllFinite(message.pose.covariance) && AllFinite(message.twist.covariance);
}

}  // namespace

void NumericMoments::Observe(const double value)
{
  ++observations;
  if (!std::isfinite(value)) {
    ++nonfinite;
    return;
  }
  minimum = minimum.has_value() ? std::min(minimum.value(), value) : value;
  maximum = maximum.has_value() ? std::max(maximum.value(), value) : value;
  sum += static_cast<long double>(value);
}

std::optional<double> NumericMoments::Mean() const
{
  const std::uint64_t finite_count = observations - nonfinite;
  if (finite_count == 0U) {
    return std::nullopt;
  }
  return static_cast<double>(sum / static_cast<long double>(finite_count));
}

bool ImuFields::operator==(const ImuFields & other) const
{
  return orientation == other.orientation &&
         angular_velocity == other.angular_velocity &&
         linear_acceleration == other.linear_acceleration &&
         orientation_covariance == other.orientation_covariance &&
         angular_velocity_covariance == other.angular_velocity_covariance &&
         linear_acceleration_covariance == other.linear_acceleration_covariance;
}

void AuditData::ObserveBagStamp(
  const std::string & topic, const std::int64_t bag_stamp_ns)
{
  ++total_messages;
  StreamAudit & stream = streams[topic];
  ++stream.messages;
  stream.bag_stamps_ns.push_back(bag_stamp_ns);
}

void AuditData::ObserveRawMocap(const geometry_msgs::msg::PoseStamped & message)
{
  StreamAudit & stream = streams["/droneyee207/pose"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  raw_mocap_stamps_ns.push_back(stamp_ns.value_or(0));
  const PoseFields pose = ToPoseFields(message.pose);
  if (!stamp_ns.has_value() || !PoseValid(pose)) {
    ++stream.invalid_payload;
    return;
  }
  PairRawPose(&pose_pairs, stamp_ns.value(), pose);
}

void AuditData::ObserveShadow(
  const localization_adapter_interfaces::msg::ShadowPoseCandidate & message)
{
  StreamAudit & stream = streams["/localization/shadow/mocap/assumed_base_pose"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  ++stream.child_frames[message.semantic_child_frame];
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  shadow_stamps_ns.push_back(stamp_ns.value_or(0));
  const PoseFields pose = ToPoseFields(message.pose);
  if (!stamp_ns.has_value() || !PoseValid(pose)) {
    ++stream.invalid_payload;
  } else {
    PairShadowPose(&pose_pairs, stamp_ns.value(), pose);
  }

  const bool contract_matches =
    message.header.frame_id == "mocap_world" &&
    message.semantic_child_frame == "base_link" &&
    message.contract_id == "droneyee207_mocap_shadow_20260722_v2" &&
    message.authorization == "shadow_candidate_only" &&
    message.source_topic == "/droneyee207/pose" &&
    message.source_parent_frame == "world" &&
    message.source_child_frame == "mocap_rigid_body" &&
    message.source_world_axes == "right_handed_x_reference_y_left_z_up_local_lab" &&
    message.source_body_axes == "x_forward_y_left_z_up" &&
    !message.geographic_alignment_validated &&
    message.world_alignment_status == "local_lab_identity_not_geographic" &&
    !message.world_alignment_approved &&
    message.extrinsic_status == "assumed_coincident_not_measured" &&
    message.extrinsic_assumption_id ==
    "four_markers_centered_at_fcu_imu_20260722" &&
    !message.extrinsic_approved &&
    message.expected_source_revision == "vrpn_client_ros2@1b9731c" &&
    message.expected_timestamp_semantics ==
    "jetson_ros_callback_time_use_server_time_false" &&
    !message.source_configuration_validated && !message.capture_time_validated;
  if (!contract_matches) {
    ++shadow_contract_mismatches;
  }
}

void AuditData::ObserveRawImu(const sensor_msgs::msg::Imu & message)
{
  StreamAudit & stream = streams["/mavros/imu/data_raw"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  const ImuFields fields = ToImuFields(message);
  if (!stamp_ns.has_value() || !ImuFieldsValid(fields) ||
    stamp_ns.value() > std::numeric_limits<std::int64_t>::max() - kExpectedImuOffsetNs)
  {
    ++stream.invalid_payload;
    return;
  }

  const std::int64_t aligned_stamp_ns = stamp_ns.value() + kExpectedImuOffsetNs;
  const auto aligned = imu_pairs.pending_aligned.find(aligned_stamp_ns);
  if (aligned != imu_pairs.pending_aligned.end()) {
    ++imu_pairs.paired;
    if (!(fields == aligned->second)) {
      ++imu_pairs.payload_mismatches;
    }
    imu_pairs.pending_aligned.erase(aligned);
    return;
  }
  if (!imu_pairs.pending_raw_by_aligned_stamp.emplace(aligned_stamp_ns, fields).second) {
    ++imu_pairs.duplicate_pending_keys;
  }
}

void AuditData::ObserveAlignedImu(const sensor_msgs::msg::Imu & message)
{
  StreamAudit & stream = streams["/fcu/imu/data_raw_aligned"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  const ImuFields fields = ToImuFields(message);
  if (!stamp_ns.has_value() || !ImuFieldsValid(fields)) {
    ++stream.invalid_payload;
    return;
  }

  const auto raw = imu_pairs.pending_raw_by_aligned_stamp.find(stamp_ns.value());
  if (raw != imu_pairs.pending_raw_by_aligned_stamp.end()) {
    ++imu_pairs.paired;
    if (!(raw->second == fields)) {
      ++imu_pairs.payload_mismatches;
    }
    imu_pairs.pending_raw_by_aligned_stamp.erase(raw);
    return;
  }
  if (!imu_pairs.pending_aligned.emplace(stamp_ns.value(), fields).second) {
    ++imu_pairs.duplicate_pending_keys;
  }
}

void AuditData::ObserveVisualStatus(
  const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus & message)
{
  StreamAudit & stream = streams["/visual_slam/status"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  visual_status_stamps_ns.push_back(stamp_ns.value_or(0));
  ++vo_states[message.vo_state];
  track_execution_time_sec.Observe(message.track_execution_time);
  const std::array<double, 4> timings{
    message.node_callback_execution_time, message.track_execution_time,
    message.track_execution_time_mean, message.track_execution_time_max};
  if (!stamp_ns.has_value() || !AllFinite(timings) ||
    std::any_of(timings.begin(), timings.end(), [](const double value) {return value < 0.0;}))
  {
    ++stream.invalid_payload;
  }
}

void AuditData::ObserveOdometry(const nav_msgs::msg::Odometry & message)
{
  StreamAudit & stream = streams["/visual_slam/tracking/odometry"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  ++stream.child_frames[message.child_frame_id];
  const auto stamp_ns = StampNanoseconds(message.header.stamp);
  odometry_stamps_ns.push_back(stamp_ns.value_or(0));
  if (!stamp_ns.has_value() || !OdometryFinite(message)) {
    ++stream.invalid_payload;
  }
}

void AuditData::ObserveTimesync(const mavros_msgs::msg::TimesyncStatus & message)
{
  StreamAudit & stream = streams["/mavros/timesync_status"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  if (message.remote_timestamp_ns == 0U ||
    message.remote_timestamp_ns >
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
  {
    timesync_remote_stamps_ns.push_back(0);
    ++stream.invalid_payload;
  } else {
    timesync_remote_stamps_ns.push_back(
      static_cast<std::int64_t>(message.remote_timestamp_ns));
  }
  timesync_observed_offset_ns.Observe(static_cast<double>(message.observed_offset_ns));
  timesync_estimated_offset_ns.Observe(static_cast<double>(message.estimated_offset_ns));
  const long double innovation_ns =
    static_cast<long double>(message.observed_offset_ns) -
    static_cast<long double>(message.estimated_offset_ns);
  timesync_offset_innovation_ns.Observe(static_cast<double>(innovation_ns));
  if (previous_timesync_estimated_offset_ns.has_value()) {
    const long double step_ns =
      static_cast<long double>(message.estimated_offset_ns) -
      static_cast<long double>(previous_timesync_estimated_offset_ns.value());
    timesync_estimated_step_ns.Observe(static_cast<double>(step_ns));
  }
  previous_timesync_estimated_offset_ns = message.estimated_offset_ns;
  timesync_round_trip_ms.Observe(static_cast<double>(message.round_trip_time_ms));
  if (!std::isfinite(message.round_trip_time_ms) || message.round_trip_time_ms < 0.0F) {
    ++stream.invalid_payload;
  }
}

void AuditData::ObserveDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & message,
  const std::int64_t bag_stamp_ns)
{
  StreamAudit & stream = streams["/diagnostics"];
  ObserveHeader(&stream, message.header.stamp, message.header.frame_id);
  std::set<std::string> names_in_message;
  for (const auto & status : message.status) {
    const bool duplicate_name = !names_in_message.insert(status.name).second;
    if (!IsRequiredDiagnosticName(status.name)) {
      continue;
    }
    if (duplicate_name) {
      ++duplicate_diagnostic_status_names;
    }
    DiagnosticStatusAudit & generic = diagnostic_statuses[status.name];
    ++generic.samples;
    generic.bag_stamps_ns.push_back(bag_stamp_ns);
    generic.header_stamps_ns.push_back(
      StampNanoseconds(message.header.stamp).value_or(0));
    ++generic.levels[status.level];
    ++generic.messages[status.message];
    ++generic.hardware_ids[status.hardware_id];
    std::uint64_t duplicate_keys = 0U;
    const DiagnosticValues values = CollectDiagnosticValues(status, &duplicate_keys);
    generic.duplicate_keys += duplicate_keys;

    if (status.name == kMocapDiagnosticName) {
      ++mocap_diagnostics.samples;
      if (duplicate_keys > 0U) {
        ++mocap_diagnostics.malformed_samples;
        continue;
      }
      const auto health = values.find("health_state");
      const auto reason = values.find("reason_code");
      bool malformed = false;
      if (health == values.end() || reason == values.end()) {
        malformed = true;
      } else {
        ++mocap_diagnostics.health_states[health->second];
        ++mocap_diagnostics.reasons[reason->second];
        mocap_diagnostics.health_observations.push_back(
          MocapHealthObservation{
              StampNanoseconds(message.header.stamp).value_or(0), bag_stamp_ns,
              health->second, reason->second});
      }

      const bool contract_matches = status.hardware_id == "droneyee207" &&
        MocapHealthTupleMatches(status, values) &&
        ValuesEqual(
        values,
        {{"contract_id", "droneyee207_mocap_shadow_20260722_v2"},
          {"expected_source_revision", "vrpn_client_ros2@1b9731c"},
          {"source_configuration_validated", "0"},
          {"mode", "shadow"},
          {"authorization", "shadow_candidate_only"},
          {"input_topic", "/droneyee207/pose"},
          {"input_type", "geometry_msgs/msg/PoseStamped"},
          {"expected_publisher", "/vrpn_client_node"},
          {"actual_publisher", "/vrpn_client_node"},
          {"publisher_count", "1"},
          {"publisher_identity_valid", "1"},
          {"publisher_type_valid", "1"},
          {"publisher_qos_valid", "1"},
          {"output_publisher_count", "1"},
          {"output_publisher_identity_valid", "1"},
          {"output_publisher_type_valid", "1"},
          {"expected_input_frame", "world"},
          {"semantic_input_child_frame", "mocap_rigid_body"},
          {"world_axes", "right_handed_x_reference_y_left_z_up_local_lab"},
          {"rigid_body_axes", "x_forward_y_left_z_up"},
          {"expected_timestamp_semantics",
            "jetson_ros_callback_time_use_server_time_false"},
          {"capture_time_validated", "0"},
          {"output_topic", "/localization/shadow/mocap/assumed_base_pose"},
          {"output_type",
            "localization_adapter_interfaces/msg/ShadowPoseCandidate"},
          {"output_parent_frame", "mocap_world"},
          {"output_child_frame_semantic", "base_link"},
          {"extrinsic_status", "assumed_coincident_not_measured"},
          {"extrinsic_assumption_id", "four_markers_centered_at_fcu_imu_20260722"},
          {"world_alignment_status", "local_lab_identity_not_geographic"},
          {"world_alignment_approved", "0"},
          {"standard_odometry_authorized", "0"},
          {"tf_authorized", "0"},
          {"mavros_authorized", "0"},
          {"vrpn_twist_consumed", "0"},
          {"vrpn_accel_consumed", "0"}});
      if (!contract_matches) {
        ++mocap_diagnostics.contract_mismatches;
      }

      const std::vector<std::string> keys{
        "received", "accepted", "published_shadow_candidates", "rejected",
        "zero_or_invalid_stamp", "duplicate", "nonmonotonic", "frame_mismatch",
        "nonfinite_position", "out_of_bounds_position", "invalid_quaternion",
        "clock_domain_mismatch", "stamp_gap_violation", "receive_gap_violation",
        "pose_reset_candidate", "publisher_authority_violation",
        "output_publisher_authority_violation"};
      std::uint64_t malformed_counters = 0U;
      CollectCounters(
        values, keys, &mocap_diagnostics.counters, &malformed_counters);
      if (malformed || malformed_counters > 0U) {
        ++mocap_diagnostics.malformed_samples;
      }
      const auto received = CounterValue(values, "received");
      const auto accepted = CounterValue(values, "accepted");
      const auto rejected = CounterValue(values, "rejected");
      const auto published = CounterValue(values, "published_shadow_candidates");
      if (received.has_value() && accepted.has_value() && rejected.has_value() &&
        (rejected.value() > std::numeric_limits<std::uint64_t>::max() -
        accepted.value() || received.value() != accepted.value() + rejected.value()))
      {
        ++mocap_diagnostics.invariant_violations;
      }
      if (published.has_value() && accepted.has_value() &&
        published.value() > accepted.value())
      {
        ++mocap_diagnostics.invariant_violations;
      }
    } else if (status.name == kAlignedImuDiagnosticName) {
      ++aligned_imu_diagnostics.samples;
      if (duplicate_keys > 0U) {
        ++aligned_imu_diagnostics.malformed_samples;
        continue;
      }
      const std::vector<std::string> keys{
        "received", "published", "zero_stamp", "invalid_stamp", "duplicate",
        "nonmonotonic", "frame_mismatch", "nonfinite_measurement",
        "clock_domain_mismatch", "aligned_out_of_range"};
      std::uint64_t malformed_counters = 0U;
      CollectCounters(
        values, keys, &aligned_imu_diagnostics.counters, &malformed_counters);
      if (malformed_counters > 0U) {
        ++aligned_imu_diagnostics.malformed_samples;
      }

      const auto offset = values.find("imu_to_camera_offset_ns");
      // Cumulative counters are collected and checked separately below.
      const bool contract_matches =
        status.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
        status.hardware_id == "px4-highres-imu-105-ttyTHS2" &&
        offset != values.end() && ParseSigned(offset->second) == kExpectedImuOffsetNs &&
        ValuesEqual(
        values,
        {{"input_topic", "/mavros/imu/data_raw"},
          {"output_topic", "/fcu/imu/data_raw_aligned"},
          {"expected_input_frame", "base_link"},
          {"output_frame", "fcu_imu"},
          {"offset_equation", "t_aligned = t_imu_raw + offset"}});
      if (!contract_matches) {
        ++aligned_imu_diagnostics.contract_mismatches;
      }

      const auto received = CounterValue(values, "received");
      const auto published = CounterValue(values, "published");
      const std::vector<std::string> rejection_keys(keys.begin() + 2, keys.end());
      const auto rejected = SumCounters(values, rejection_keys);
      if (received.has_value() && published.has_value() && rejected.has_value()) {
        if (rejected.value() > std::numeric_limits<std::uint64_t>::max() -
          published.value() ||
          received.value() != published.value() + rejected.value())
        {
          ++aligned_imu_diagnostics.invariant_violations;
        }
      } else if (malformed_counters == 0U) {
        ++aligned_imu_diagnostics.malformed_samples;
      }
    } else if (status.name == kRuntimeDiagnosticName) {
      ++runtime_diagnostics.samples;
      if (duplicate_keys > 0U) {
        ++runtime_diagnostics.malformed_samples;
        continue;
      }
      const std::vector<std::string> keys{
        "left_camera_info", "right_camera_info", "odometry", "forbidden_camera_imu"};
      std::uint64_t malformed_counters = 0U;
      CollectCounters(
        values, keys, &runtime_diagnostics.counters, &malformed_counters);
      if (malformed_counters > 0U) {
        ++runtime_diagnostics.malformed_samples;
      }

      const bool contract_matches =
        status.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
        status.hardware_id == "243622070369" &&
        ValuesEqual(
        values,
        {{"calibration_id", "d435i_243622070369_factory_rectified_px4_imu_20260720"},
          {"left_camera_info_topic", "/camera/infra1/camera_info"},
          {"right_camera_info_topic", "/camera/infra2/camera_info"},
          {"camera_imu_topic", "/camera/imu"},
          {"odometry_topic", "/visual_slam/tracking/odometry"},
          {"expected_odometry_frame", "odom"},
          {"expected_odometry_child_frame", "camera_link"},
          {"known_right_frame_reuse_observed", "True"}});
      if (!contract_matches) {
        ++runtime_diagnostics.contract_mismatches;
      }
    }
  }
}

std::size_t PendingPoseCount(
  const std::unordered_map<std::int64_t, std::deque<PoseFields>> & pending)
{
  std::size_t count = 0U;
  for (const auto & entry : pending) {
    count += entry.second.size();
  }
  return count;
}

std::string ToString(const FindingSeverity severity)
{
  switch (severity) {
    case FindingSeverity::kObserved:
      return "OBSERVED";
    case FindingSeverity::kReview:
      return "REVIEW";
    case FindingSeverity::kFail:
      return "FAIL";
  }
  return "FAIL";
}

}  // namespace detail
}  // namespace bag_contract_probe
