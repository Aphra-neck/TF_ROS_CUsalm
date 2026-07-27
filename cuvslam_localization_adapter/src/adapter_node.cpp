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

#include "cuvslam_localization_adapter/adapter_node.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/message_info.hpp>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/gate.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/validation.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using localization_contracts::AuthorizationState;
using localization_contracts::CameraPoseToBasePose;
using localization_contracts::ContractViolation;
using localization_contracts::HealthState;
using localization_contracts::OdometrySample;
using localization_contracts::PoseStep;
using localization_contracts::QuaternionXyzw;
using localization_contracts::RigidTransform;
using localization_contracts::StampOrder;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr char kOdometryMessageType[] = "nav_msgs/msg/Odometry";
constexpr char kStatusMessageType[] =
  "isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus";
constexpr char kCandidateTopic[] = "/localization/candidates/cuvslam/base_pose";
constexpr char kCandidateMessageType[] =
  "localization_adapter_interfaces/msg/LocalizationSourceCandidate";
constexpr char kCandidateAuthorization[] = "source_pose_candidate_only";
constexpr char kSourceId[] = "cuvslam";

rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

std::string DefaultContractFile()
{
  return ament_index_cpp::get_package_share_directory("cuvslam_localization_adapter") +
         "/config/contract_blocked.yaml";
}

std::optional<std::int64_t> StampToNanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds >
    (std::numeric_limits<std::int64_t>::max() - stamp.nanosec) / kNanosecondsPerSecond)
  {
    return std::nullopt;
  }
  return seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(stamp.nanosec);
}

std::array<double, 36> CopyCovariance(const std::array<double, 36> & covariance)
{
  return covariance;
}

std::string Join(const std::vector<std::string> & values)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    output << (index == 0U ? "" : ",") << values[index];
  }
  return output.str();
}

KeyValue Value(const std::string & key, const std::string & value)
{
  KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template<typename T>
KeyValue Value(const std::string & key, const T & value)
{
  std::ostringstream stream;
  stream << value;
  return Value(key, stream.str());
}

std::string FullyQualifiedNodeName(const rclcpp::TopicEndpointInfo & endpoint)
{
  const std::string node_namespace = endpoint.node_namespace();
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + endpoint.node_name();
  }
  return node_namespace +
         (node_namespace.back() == '/' ? "" : "/") + endpoint.node_name();
}

template<typename Gid>
bool GidsEqual(const Gid & left, const Gid & right)
{
  return std::equal(left.begin(), left.end(), right.begin());
}

template<typename Gid>
bool GidMatchesRmw(const Gid & endpoint_gid, const rmw_gid_t & publisher_gid)
{
  return std::equal(endpoint_gid.begin(), endpoint_gid.end(), publisher_gid.data);
}

template<typename Gid>
std::string GidToString(const Gid & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace

CuvslamLocalizationAdapter::CuvslamLocalizationAdapter(const rclcpp::NodeOptions & options)
: Node("cuvslam_localization_adapter", options),
  mode_(declare_parameter<std::string>(
      "mode", "shadow",
      ReadOnlyDescriptor("Stage-1 mode; this revision accepts only 'shadow'."))),
  contract_file_(declare_parameter<std::string>(
      "contract_file", DefaultContractFile(),
      ReadOnlyDescriptor("Version-controlled localization contract YAML."))),
  contract_(LoadContractConfig(contract_file_)),
  diagnostic_status_name_(
    std::string(get_fully_qualified_name()) + ": localization contract"),
  health_gate_(contract_.health.recovery_consecutive_samples),
  started_at_(std::chrono::steady_clock::now())
{
  if (mode_ != "shadow") {
    throw std::runtime_error(
            "this revision is shadow-only; mode must be exactly 'shadow'");
  }
  if (contract_.authorization != AuthorizationState::kShadowOnly) {
    throw std::runtime_error(
            "shadow node refuses a contract with active publication authorization");
  }
  for (const auto & required : contract_.input.required_diagnostics) {
    if (required.name == diagnostic_status_name_) {
      throw std::runtime_error(
              "required diagnostics must not depend on this adapter's own status");
    }
    diagnostic_sequence_guards_.try_emplace(required.name);
  }

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
  const auto diagnostics_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    contract_.input.diagnostics_topic, diagnostics_qos);
  source_candidate_publisher_ = create_publisher<
    localization_adapter_interfaces::msg::LocalizationSourceCandidate>(
    kCandidateTopic, diagnostics_qos);
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    contract_.input.odometry_topic,
    sensor_qos,
    std::bind(
      &CuvslamLocalizationAdapter::OnOdometry,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  status_subscription_ = create_subscription<
    isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
    contract_.input.visual_slam_status_topic,
    sensor_qos,
    std::bind(
      &CuvslamLocalizationAdapter::OnVisualSlamStatus,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  upstream_diagnostics_subscription_ = create_subscription<DiagnosticArray>(
    contract_.input.diagnostics_topic,
    diagnostics_qos,
    std::bind(
      &CuvslamLocalizationAdapter::OnUpstreamDiagnostics,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  if (std::string(diagnostics_publisher_->get_topic_name()) !=
    contract_.input.diagnostics_topic ||
    std::string(odometry_subscription_->get_topic_name()) !=
    contract_.input.odometry_topic ||
    std::string(source_candidate_publisher_->get_topic_name()) != kCandidateTopic ||
    std::string(status_subscription_->get_topic_name()) !=
    contract_.input.visual_slam_status_topic ||
    std::string(upstream_diagnostics_subscription_->get_topic_name()) !=
    contract_.input.diagnostics_topic)
  {
    throw std::runtime_error(
            "ROS remapping of contract-bound topics is forbidden in schema version one");
  }
  const auto diagnostic_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(contract_.health.diagnostic_period_sec));
  diagnostic_timer_ = create_wall_timer(
    diagnostic_period,
    std::bind(&CuvslamLocalizationAdapter::OnDiagnosticTimer, this));

  RCLCPP_WARN(
    get_logger(),
    "Shadow-only localization adapter loaded contract %s (%s); only the unprivileged "
    "source candidate may be published",
    contract_.contract_id.c_str(), contract_.source_sha256.c_str());
}

void CuvslamLocalizationAdapter::OnOdometry(
  const nav_msgs::msg::Odometry::SharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  last_actual_parent_frame_ = message->header.frame_id;
  last_actual_child_frame_ = message->child_frame_id;
  if (!ValidateMessagePublisherLocked(
      message_info,
      contract_.input.odometry_topic,
      contract_.input.expected_odometry_publisher,
      kOdometryMessageType,
      &bound_odometry_publisher_gid_,
      &odometry_publisher_count_,
      &odometry_publisher_identity_valid_,
      &odometry_publisher_type_valid_,
      &odometry_publisher_authority_violation_,
      "ODOMETRY"))
  {
    ++rejected_;
    last_odometry_valid_ = false;
    return;
  }

  const auto stamp_ns = StampToNanoseconds(message->header.stamp);
  OdometrySample sample{
    message->header.frame_id,
    message->child_frame_id,
    stamp_ns.value_or(0),
    Eigen::Vector3d(
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z),
    QuaternionXyzw{
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z,
      message->pose.pose.orientation.w},
    Eigen::Vector3d(
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z),
    Eigen::Vector3d(
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z),
    CopyCovariance(message->pose.covariance),
    CopyCovariance(message->twist.covariance)};
  const auto issues = localization_contracts::ValidateOdometrySample(
    sample,
    contract_.input.expected_parent_frame,
    contract_.input.expected_child_frame);
  if (!issues.empty()) {
    ++rejected_;
    last_odometry_valid_ = false;
    const auto latched = std::find_if(
      issues.begin(), issues.end(),
      [](const localization_contracts::ValidationIssue & issue) {return issue.latched;});
    if (latched != issues.end()) {
      health_gate_.MarkLatched(latched->code);
    } else {
      health_gate_.MarkTransient(issues.front().code);
    }
    return;
  }

  const auto previous_stamp = odometry_sequence_guard_.LastAcceptedNanoseconds();
  const StampOrder order = odometry_sequence_guard_.Classify(sample.stamp_ns);
  if (order == StampOrder::kDuplicate) {
    ++rejected_;
    ++duplicate_;
    last_odometry_valid_ = false;
    health_gate_.MarkTransient("DUPLICATE_TIMESTAMP");
    return;
  }
  if (order == StampOrder::kNonmonotonic) {
    ++rejected_;
    ++nonmonotonic_;
    last_odometry_valid_ = false;
    health_gate_.MarkLatched("NONMONOTONIC_TIMESTAMP");
    return;
  }

  double clock_residual_sec = 0.0;
  const bool clock_valid = ClockDomainMatches(sample.stamp_ns, &clock_residual_sec);
  last_odometry_clock_residual_sec_ = clock_residual_sec;
  if (!clock_valid) {
    ++rejected_;
    ++clock_domain_mismatch_;
    last_odometry_valid_ = false;
    health_gate_.MarkTransient("ODOMETRY_CLOCK_DOMAIN_MISMATCH");
    return;
  }

  odometry_sequence_guard_.Commit(sample.stamp_ns);
  UpdateOdometryRateLocked(sample.stamp_ns);

  bool receive_gap_valid = true;
  if (last_odometry_observed_at_.has_value()) {
    last_odometry_receive_gap_sec_ = std::chrono::duration<double>(
      now - last_odometry_observed_at_.value()).count();
    receive_gap_valid = last_odometry_receive_gap_sec_.value() <=
      contract_.health.maximum_odometry_receive_gap_sec;
    if (!receive_gap_valid) {
      ++odometry_receive_gap_violation_;
    }
  }
  last_odometry_observed_at_ = now;

  bool stamp_gap_valid = true;
  if (previous_stamp.has_value()) {
    const double stamp_gap_sec = localization_contracts::AbsoluteStampDifferenceSeconds(
      sample.stamp_ns, previous_stamp.value());
    stamp_gap_valid = stamp_gap_sec <=
      contract_.health.maximum_odometry_stamp_gap_sec;
    if (!stamp_gap_valid) {
      ++odometry_stamp_gap_violation_;
    }
  }

  std::optional<RigidTransform> odom_from_camera;
  try {
    odom_from_camera = RigidTransform::Create(
      sample.parent_frame,
      sample.child_frame,
      sample.position,
      sample.orientation_xyzw);
  } catch (const ContractViolation &) {
    ++rejected_;
    last_odometry_valid_ = false;
    health_gate_.MarkLatched("INPUT_POSE_CONSTRUCTION_FAILED");
    return;
  }
  if (last_input_pose_.has_value()) {
    const PoseStep step = localization_contracts::MeasurePoseStep(
      last_input_pose_.value(), odom_from_camera.value());
    if (step.translation_m > contract_.health.maximum_position_step_m ||
      step.rotation_rad > contract_.health.maximum_rotation_step_rad)
    {
      ++rejected_;
      ++pose_jump_violation_;
      last_odometry_valid_ = false;
      health_gate_.MarkLatched("LOCALIZATION_EPOCH_JUMP_DETECTED");
      return;
    }
  }
  if (!observed_odometry_rate_hz_.has_value()) {
    last_odometry_valid_ = false;
    return;
  }
  const bool rate_valid = observed_odometry_rate_hz_.value() >=
    contract_.health.minimum_odometry_rate_hz;
  if (!rate_valid) {
    ++odometry_rate_violation_;
  }
  if (!stamp_gap_valid || !receive_gap_valid || !rate_valid ||
    (maximum_recent_stamp_gap_sec_.has_value() &&
    maximum_recent_stamp_gap_sec_.value() >
    contract_.health.maximum_odometry_stamp_gap_sec))
  {
    ++rejected_;
    last_odometry_valid_ = false;
    if (!stamp_gap_valid ||
      maximum_recent_stamp_gap_sec_.value() >
      contract_.health.maximum_odometry_stamp_gap_sec)
    {
      health_gate_.MarkTransient("ODOMETRY_STAMP_GAP_EXCEEDED");
    } else if (!receive_gap_valid) {
      health_gate_.MarkTransient("ODOMETRY_RECEIVE_GAP_EXCEEDED");
    } else {
      health_gate_.MarkTransient("ODOMETRY_RATE_TOO_LOW");
    }
    return;
  }

  stamp_guard_.Commit(sample.stamp_ns);
  last_input_pose_ = odom_from_camera;
  ++accepted_;
  ++shadow_processed_;
  last_odometry_valid_ = true;
  last_odometry_received_at_ = now;

  if (contract_.extrinsic.Approved()) {
    try {
      last_shadow_pose_ = CameraPoseToBasePose(
        odom_from_camera.value(),
        contract_.extrinsic.transform.value());
      ++shadow_pose_computed_;
    } catch (const ContractViolation &) {
      ++rejected_;
      last_odometry_valid_ = false;
      health_gate_.MarkLatched("SHADOW_POSE_TRANSFORM_FAILED");
      return;
    }
  }
  MaybeAdvanceHealthLocked(now);
  if (health_gate_.State() == HealthState::kHealthy) {
    PublishSourceCandidateLocked(message->header.stamp);
  }
}

void CuvslamLocalizationAdapter::OnVisualSlamStatus(
  const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::SharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++status_received_;
  if (!ValidateMessagePublisherLocked(
      message_info,
      contract_.input.visual_slam_status_topic,
      contract_.input.expected_status_publisher,
      kStatusMessageType,
      &bound_status_publisher_gid_,
      &status_publisher_count_,
      &status_publisher_identity_valid_,
      &status_publisher_type_valid_,
      &status_publisher_authority_violation_,
      "TRACKING_STATUS"))
  {
    ++status_rejected_;
    last_status_valid_ = false;
    return;
  }
  const auto stamp_ns = StampToNanoseconds(message->header.stamp);
  const bool timing_valid =
    std::isfinite(message->node_callback_execution_time) &&
    std::isfinite(message->track_execution_time) &&
    std::isfinite(message->track_execution_time_mean) &&
    std::isfinite(message->track_execution_time_max) &&
    message->node_callback_execution_time >= 0.0 &&
    message->track_execution_time >= 0.0 &&
    message->track_execution_time_mean >= 0.0 &&
    message->track_execution_time_max >= 0.0;
  if (!stamp_ns.has_value() || stamp_ns.value() <= 0 ||
    message->header.frame_id != contract_.input.expected_status_frame || !timing_valid)
  {
    ++status_rejected_;
    last_status_valid_ = false;
    health_gate_.MarkTransient("TRACKING_STATUS_INVALID");
    return;
  }

  const StampOrder order = status_sequence_guard_.Classify(stamp_ns.value());
  if (order == StampOrder::kDuplicate) {
    ++status_rejected_;
    ++status_duplicate_;
    last_status_valid_ = false;
    health_gate_.MarkTransient("TRACKING_STATUS_DUPLICATE_TIMESTAMP");
    return;
  }
  if (order == StampOrder::kNonmonotonic) {
    ++status_rejected_;
    ++status_nonmonotonic_;
    last_status_valid_ = false;
    health_gate_.MarkLatched("TRACKING_STATUS_NONMONOTONIC_TIMESTAMP");
    return;
  }

  double clock_residual_sec = 0.0;
  if (!ClockDomainMatches(stamp_ns.value(), &clock_residual_sec)) {
    ++status_rejected_;
    ++clock_domain_mismatch_;
    last_status_clock_residual_sec_ = clock_residual_sec;
    last_status_valid_ = false;
    health_gate_.MarkTransient("TRACKING_STATUS_CLOCK_DOMAIN_MISMATCH");
    return;
  }
  last_status_clock_residual_sec_ = clock_residual_sec;
  status_sequence_guard_.Commit(stamp_ns.value());
  last_vo_state_ = message->vo_state;
  last_status_valid_ = std::find(
    contract_.input.accepted_vo_states.begin(),
    contract_.input.accepted_vo_states.end(),
    message->vo_state) != contract_.input.accepted_vo_states.end();
  if (!last_status_valid_) {
    health_gate_.MarkTransient("TRACKING_STATE_NOT_ACCEPTED");
    return;
  }
  status_stamp_guard_.Commit(stamp_ns.value());
  last_status_received_at_ = now;
  MaybeAdvanceHealthLocked(now);
}

void CuvslamLocalizationAdapter::OnUpstreamDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  struct Candidate
  {
    const DiagnosticStatus * status;
    const RequiredDiagnosticConfig * required;
    StampOrder stamp_order;
  };
  std::vector<Candidate> candidates;
  std::set<std::string> names_in_message;
  for (const auto & status : message->status) {
    const auto required = std::find_if(
      contract_.input.required_diagnostics.begin(),
      contract_.input.required_diagnostics.end(),
      [&status](const RequiredDiagnosticConfig & candidate) {
        return candidate.name == status.name;
      });
    if (required == contract_.input.required_diagnostics.end()) {
      continue;
    }
    ++diagnostic_evidence_received_;
    if (!names_in_message.insert(status.name).second) {
      ++diagnostic_evidence_rejected_;
      upstream_diagnostics_.erase(status.name);
      health_gate_.MarkLatched("DUPLICATE_REQUIRED_DIAGNOSTIC_STATUS");
      return;
    }
    candidates.push_back(Candidate{&status, &(*required), StampOrder::kAccept});
  }
  if (candidates.empty()) {
    return;
  }
  for (const auto & candidate : candidates) {
    upstream_diagnostics_.erase(candidate.required->name);
  }

  std::optional<std::string> source_node;
  const auto & publisher_gid =
    message_info.get_rmw_message_info().publisher_gid;
  const auto diagnostic_publishers = get_publishers_info_by_topic(
    contract_.input.diagnostics_topic);
  for (const auto & endpoint : diagnostic_publishers) {
    const auto & endpoint_gid = endpoint.endpoint_gid();
    if (std::equal(
        endpoint_gid.begin(), endpoint_gid.end(), publisher_gid.data))
    {
      if (source_node.has_value()) {
        diagnostic_evidence_rejected_ += candidates.size();
        health_gate_.MarkLatched("DIAGNOSTIC_SOURCE_GID_NOT_UNIQUE");
        return;
      }
      source_node = FullyQualifiedNodeName(endpoint);
    }
  }
  if (!source_node.has_value()) {
    diagnostic_evidence_rejected_ += candidates.size();
    health_gate_.MarkTransient("DIAGNOSTIC_SOURCE_NOT_DISCOVERED");
    return;
  }
  const auto same_name_publishers = std::count_if(
    diagnostic_publishers.begin(), diagnostic_publishers.end(),
    [&source_node](const rclcpp::TopicEndpointInfo & endpoint) {
      return FullyQualifiedNodeName(endpoint) == source_node.value();
    });
  if (same_name_publishers != 1) {
    diagnostic_evidence_rejected_ += candidates.size();
    health_gate_.MarkLatched("DIAGNOSTIC_PUBLISHER_NOT_UNIQUE");
    return;
  }
  for (const auto & candidate : candidates) {
    if (source_node.value() != candidate.required->expected_publisher) {
      diagnostic_evidence_rejected_ += candidates.size();
      health_gate_.MarkLatched("DIAGNOSTIC_PUBLISHER_IDENTITY_MISMATCH");
      return;
    }
  }

  const auto stamp_ns = StampToNanoseconds(message->header.stamp);
  if (!stamp_ns.has_value() || stamp_ns.value() <= 0) {
    diagnostic_evidence_rejected_ += candidates.size();
    health_gate_.MarkTransient("UPSTREAM_DIAGNOSTIC_TIMESTAMP_INVALID");
    return;
  }
  double clock_residual_sec = 0.0;
  if (!ClockDomainMatches(stamp_ns.value(), &clock_residual_sec)) {
    diagnostic_evidence_rejected_ += candidates.size();
    ++clock_domain_mismatch_;
    last_diagnostic_clock_residual_sec_ = clock_residual_sec;
    health_gate_.MarkTransient("UPSTREAM_DIAGNOSTIC_CLOCK_DOMAIN_MISMATCH");
    return;
  }
  last_diagnostic_clock_residual_sec_ = clock_residual_sec;

  for (auto & candidate : candidates) {
    candidate.stamp_order = diagnostic_sequence_guards_.at(
      candidate.required->name).Classify(stamp_ns.value());
    if (candidate.stamp_order == StampOrder::kDuplicate) {
      diagnostic_evidence_rejected_ += candidates.size();
      ++diagnostic_duplicate_;
      health_gate_.MarkTransient("UPSTREAM_DIAGNOSTIC_DUPLICATE_TIMESTAMP");
      return;
    }
    if (candidate.stamp_order == StampOrder::kNonmonotonic) {
      diagnostic_evidence_rejected_ += candidates.size();
      ++diagnostic_nonmonotonic_;
      health_gate_.MarkLatched("UPSTREAM_DIAGNOSTIC_NONMONOTONIC_TIMESTAMP");
      return;
    }
  }

  for (const auto & candidate : candidates) {
    diagnostic_sequence_guards_.at(candidate.required->name).Commit(stamp_ns.value());
  }

  for (const auto & candidate : candidates) {
    std::vector<std::pair<std::string, std::string>> actual_values;
    actual_values.reserve(candidate.status->values.size());
    for (const auto & value : candidate.status->values) {
      actual_values.emplace_back(value.key, value.value);
    }
    const auto evidence_error = CheckDiagnosticEvidence(
      *candidate.required,
      candidate.status->level,
      candidate.status->hardware_id,
      actual_values);
    if (evidence_error.has_value()) {
      diagnostic_evidence_rejected_ += candidates.size();
      if (evidence_error.value() == "UPSTREAM_DIAGNOSTIC_DUPLICATE_VALUE_KEY") {
        health_gate_.MarkLatched(evidence_error.value());
      } else {
        health_gate_.MarkTransient(evidence_error.value());
      }
      return;
    }
  }

  for (const auto & candidate : candidates) {
    upstream_diagnostics_[candidate.status->name] = UpstreamDiagnosticObservation{
      candidate.status->level, stamp_ns.value(), now};
  }
  MaybeAdvanceHealthLocked(now);
}

bool CuvslamLocalizationAdapter::ValidateMessagePublisherLocked(
  const rclcpp::MessageInfo & message_info,
  const std::string & topic,
  const std::string & expected_publisher,
  const std::string & expected_type,
  std::optional<PublisherGid> * bound_gid,
  std::size_t * publisher_count,
  bool * publisher_identity_valid,
  bool * publisher_type_valid,
  std::uint64_t * authority_violation,
  const std::string & reason_prefix)
{
  if (health_gate_.State() == HealthState::kLatchedFault) {
    return false;
  }

  PublisherGid message_gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, message_gid.begin());
  const auto endpoints = get_publishers_info_by_topic(topic);
  *publisher_count = endpoints.size();
  *publisher_identity_valid =
    endpoints.size() == 1U &&
    FullyQualifiedNodeName(endpoints.front()) == expected_publisher;
  *publisher_type_valid =
    endpoints.size() == 1U && endpoints.front().topic_type() == expected_type;

  if (endpoints.size() > 1U) {
    ++(*authority_violation);
    health_gate_.MarkLatched(reason_prefix + "_PUBLISHER_NOT_UNIQUE");
    return false;
  }
  if (endpoints.empty()) {
    if (health_gate_.State() == HealthState::kStarting) {
      health_gate_.MarkStarting(reason_prefix + "_PUBLISHER_NOT_DISCOVERED");
    } else {
      health_gate_.MarkTransient(reason_prefix + "_PUBLISHER_NOT_DISCOVERED");
    }
    return false;
  }
  if (!*publisher_identity_valid) {
    ++(*authority_violation);
    health_gate_.MarkLatched(reason_prefix + "_PUBLISHER_IDENTITY_MISMATCH");
    return false;
  }
  if (!*publisher_type_valid) {
    ++(*authority_violation);
    health_gate_.MarkLatched(reason_prefix + "_PUBLISHER_TYPE_MISMATCH");
    return false;
  }
  if (!GidsEqual(endpoints.front().endpoint_gid(), message_gid)) {
    ++(*authority_violation);
    health_gate_.MarkLatched(reason_prefix + "_MESSAGE_GID_NOT_DISCOVERED");
    return false;
  }
  if (bound_gid->has_value() && !GidsEqual(bound_gid->value(), message_gid)) {
    ++(*authority_violation);
    health_gate_.MarkLatched(reason_prefix + "_PUBLISHER_GID_CHANGED");
    return false;
  }
  if (!bound_gid->has_value()) {
    *bound_gid = message_gid;
  }
  return true;
}

void CuvslamLocalizationAdapter::UpdatePublisherEvidenceLocked()
{
  const auto odometry_publishers = get_publishers_info_by_topic(
    contract_.input.odometry_topic);
  odometry_publisher_count_ = odometry_publishers.size();
  odometry_publisher_identity_valid_ =
    odometry_publishers.size() == 1U &&
    FullyQualifiedNodeName(odometry_publishers.front()) ==
    contract_.input.expected_odometry_publisher;
  odometry_publisher_type_valid_ =
    odometry_publishers.size() == 1U &&
    odometry_publishers.front().topic_type() == kOdometryMessageType;
  if (health_gate_.State() != HealthState::kLatchedFault &&
    bound_odometry_publisher_gid_.has_value() && odometry_publishers.size() == 1U &&
    !GidsEqual(
      bound_odometry_publisher_gid_.value(),
      odometry_publishers.front().endpoint_gid()))
  {
    ++odometry_publisher_authority_violation_;
    health_gate_.MarkLatched("ODOMETRY_PUBLISHER_GID_CHANGED");
  }

  const auto status_publishers = get_publishers_info_by_topic(
    contract_.input.visual_slam_status_topic);
  status_publisher_count_ = status_publishers.size();
  status_publisher_identity_valid_ =
    status_publishers.size() == 1U &&
    FullyQualifiedNodeName(status_publishers.front()) ==
    contract_.input.expected_status_publisher;
  status_publisher_type_valid_ =
    status_publishers.size() == 1U &&
    status_publishers.front().topic_type() == kStatusMessageType;
  if (health_gate_.State() != HealthState::kLatchedFault &&
    bound_status_publisher_gid_.has_value() && status_publishers.size() == 1U &&
    !GidsEqual(
      bound_status_publisher_gid_.value(),
      status_publishers.front().endpoint_gid()))
  {
    ++status_publisher_authority_violation_;
    health_gate_.MarkLatched("TRACKING_STATUS_PUBLISHER_GID_CHANGED");
  }
}

void CuvslamLocalizationAdapter::UpdateCandidatePublisherEvidenceLocked()
{
  const auto endpoints = get_publishers_info_by_topic(kCandidateTopic);
  candidate_publisher_count_ = endpoints.size();
  candidate_publisher_identity_valid_ =
    endpoints.size() == 1U &&
    FullyQualifiedNodeName(endpoints.front()) == get_fully_qualified_name();
  candidate_publisher_type_valid_ =
    endpoints.size() == 1U && endpoints.front().topic_type() == kCandidateMessageType;
  candidate_publisher_gid_valid_ =
    endpoints.size() == 1U &&
    GidMatchesRmw(endpoints.front().endpoint_gid(), source_candidate_publisher_->get_gid());
}

bool CuvslamLocalizationAdapter::CandidatePublisherIsAuthoritativeLocked()
{
  UpdateCandidatePublisherEvidenceLocked();
  if (candidate_publisher_count_ == 1U && candidate_publisher_identity_valid_ &&
    candidate_publisher_type_valid_ && candidate_publisher_gid_valid_)
  {
    return true;
  }

  ++candidate_publisher_authority_violation_;
  health_gate_.MarkLatched(
    candidate_publisher_count_ > 1U ?
    "SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE" :
    "SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID");
  return false;
}

void CuvslamLocalizationAdapter::UpdateOdometryRateLocked(const std::int64_t stamp_ns)
{
  recent_odometry_stamps_ns_.push_back(stamp_ns);
  while (recent_odometry_stamps_ns_.size() >
    contract_.health.odometry_rate_window_samples)
  {
    recent_odometry_stamps_ns_.pop_front();
  }
  if (recent_odometry_stamps_ns_.size() <
    contract_.health.odometry_rate_window_samples)
  {
    observed_odometry_rate_hz_.reset();
    maximum_recent_stamp_gap_sec_.reset();
    return;
  }

  const std::vector<std::int64_t> stamps(
    recent_odometry_stamps_ns_.begin(), recent_odometry_stamps_ns_.end());
  const auto metrics = localization_contracts::ComputeStampWindowMetrics(stamps);
  observed_odometry_rate_hz_ = metrics->rate_hz;
  maximum_recent_stamp_gap_sec_ = metrics->maximum_gap_sec;
}

bool CuvslamLocalizationAdapter::ClockDomainMatches(
  const std::int64_t stamp_ns,
  double * residual_sec)
{
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  if (now_ns <= 0 || stamp_ns <= 0) {
    *residual_sec = std::numeric_limits<double>::infinity();
    return false;
  }
  *residual_sec = localization_contracts::AbsoluteStampDifferenceSeconds(
    now_ns, stamp_ns);
  return std::isfinite(*residual_sec) &&
         *residual_sec <= contract_.health.maximum_clock_residual_sec;
}

bool CuvslamLocalizationAdapter::InputsAreHealthyLocked(
  const SteadyTime & now,
  std::string * reason) const
{
  if (odometry_publisher_count_ == 0U) {
    *reason = "WAITING_FOR_ODOMETRY_PUBLISHER";
    return false;
  }
  if (odometry_publisher_count_ != 1U || !odometry_publisher_identity_valid_ ||
    !odometry_publisher_type_valid_)
  {
    *reason = "ODOMETRY_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (!bound_odometry_publisher_gid_.has_value()) {
    *reason = "WAITING_FOR_ODOMETRY_PUBLISHER_GID";
    return false;
  }
  if (status_publisher_count_ == 0U) {
    *reason = "WAITING_FOR_TRACKING_STATUS_PUBLISHER";
    return false;
  }
  if (status_publisher_count_ != 1U || !status_publisher_identity_valid_ ||
    !status_publisher_type_valid_)
  {
    *reason = "TRACKING_STATUS_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (!bound_status_publisher_gid_.has_value()) {
    *reason = "WAITING_FOR_TRACKING_STATUS_PUBLISHER_GID";
    return false;
  }
  if (candidate_publisher_count_ == 0U) {
    *reason = "WAITING_FOR_SOURCE_CANDIDATE_PUBLISHER";
    return false;
  }
  if (candidate_publisher_count_ != 1U || !candidate_publisher_identity_valid_ ||
    !candidate_publisher_type_valid_ || !candidate_publisher_gid_valid_)
  {
    *reason = "SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (!last_odometry_received_at_.has_value()) {
    if (!observed_odometry_rate_hz_.has_value() &&
      recent_odometry_stamps_ns_.size() > 0U)
    {
      *reason = "WAITING_FOR_ODOMETRY_RATE_WINDOW";
      return false;
    }
    if (received_ > 0U && !last_odometry_valid_) {
      *reason = "ODOMETRY_INVALID";
      return false;
    }
    *reason = "WAITING_FOR_ODOMETRY";
    return false;
  }
  if (!last_status_received_at_.has_value()) {
    *reason = "WAITING_FOR_TRACKING_STATUS";
    return false;
  }
  if (!last_odometry_valid_) {
    *reason = "ODOMETRY_INVALID";
    return false;
  }
  if (!last_status_valid_) {
    *reason = "TRACKING_STATE_NOT_ACCEPTED";
    return false;
  }
  if (!observed_odometry_rate_hz_.has_value()) {
    *reason = "WAITING_FOR_ODOMETRY_RATE_WINDOW";
    return false;
  }
  if (observed_odometry_rate_hz_.value() <
    contract_.health.minimum_odometry_rate_hz)
  {
    *reason = "ODOMETRY_RATE_TOO_LOW";
    return false;
  }
  if (!maximum_recent_stamp_gap_sec_.has_value() ||
    maximum_recent_stamp_gap_sec_.value() >
    contract_.health.maximum_odometry_stamp_gap_sec)
  {
    *reason = "ODOMETRY_STAMP_GAP_EXCEEDED";
    return false;
  }
  const auto odometry_stamp = stamp_guard_.LastAcceptedNanoseconds();
  const auto status_stamp = status_stamp_guard_.LastAcceptedNanoseconds();
  if (!odometry_stamp.has_value() || !status_stamp.has_value()) {
    *reason = "WAITING_FOR_CROSS_STREAM_TIMESTAMPS";
    return false;
  }
  const double stream_skew_sec = localization_contracts::AbsoluteStampDifferenceSeconds(
    odometry_stamp.value(), status_stamp.value());
  if (stream_skew_sec > contract_.health.maximum_status_odometry_skew_sec) {
    *reason = "ODOMETRY_STATUS_SKEW_EXCEEDED";
    return false;
  }
  if (AgeSeconds(last_odometry_received_at_, now).value() >
    contract_.health.odometry_stale_after_sec)
  {
    *reason = "ODOMETRY_STALE";
    return false;
  }
  if (AgeSeconds(last_status_received_at_, now).value() >
    contract_.health.tracking_stale_after_sec)
  {
    *reason = "TRACKING_STATUS_STALE";
    return false;
  }
  for (const auto & required : contract_.input.required_diagnostics) {
    const auto observation = upstream_diagnostics_.find(required.name);
    if (observation == upstream_diagnostics_.end()) {
      *reason = "WAITING_FOR_UPSTREAM_DIAGNOSTIC";
      return false;
    }
    if (observation->second.level > required.maximum_level) {
      *reason = "UPSTREAM_DIAGNOSTIC_NOT_OK";
      return false;
    }
    const double age = std::chrono::duration<double>(
      now - observation->second.received_at).count();
    if (age > contract_.health.diagnostic_stale_after_sec) {
      *reason = "UPSTREAM_DIAGNOSTIC_STALE";
      return false;
    }
  }
  *reason = "INPUT_HEALTHY";
  return true;
}

void CuvslamLocalizationAdapter::MaybeAdvanceHealthLocked(const SteadyTime & now)
{
  if (health_gate_.State() == HealthState::kLatchedFault) {
    return;
  }
  std::string reason;
  if (!InputsAreHealthyLocked(now, &reason)) {
    if (health_gate_.State() != HealthState::kStarting) {
      health_gate_.MarkTransient(reason);
    }
    return;
  }
  const auto stamp = stamp_guard_.LastAcceptedNanoseconds();
  if (!stamp.has_value() || last_health_observed_stamp_ns_ == stamp) {
    return;
  }
  health_gate_.ObserveHealthySample();
  last_health_observed_stamp_ns_ = stamp;
}

std::optional<double> CuvslamLocalizationAdapter::AgeSeconds(
  const std::optional<SteadyTime> & received_at,
  const SteadyTime & now) const
{
  if (!received_at.has_value()) {
    return std::nullopt;
  }
  return std::chrono::duration<double>(now - received_at.value()).count();
}

void CuvslamLocalizationAdapter::OnDiagnosticTimer()
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  UpdatePublisherEvidenceLocked();
  UpdateCandidatePublisherEvidenceLocked();
  const bool candidate_publisher_authority_invalid =
    candidate_publisher_count_ == 1U &&
    (!candidate_publisher_identity_valid_ || !candidate_publisher_type_valid_ ||
    !candidate_publisher_gid_valid_);
  if (health_gate_.State() != HealthState::kLatchedFault) {
    if (odometry_publisher_count_ > 1U) {
      ++odometry_publisher_authority_violation_;
      health_gate_.MarkLatched("ODOMETRY_PUBLISHER_NOT_UNIQUE");
    } else if (odometry_publisher_count_ == 1U && !odometry_publisher_identity_valid_) {
      ++odometry_publisher_authority_violation_;
      health_gate_.MarkLatched("ODOMETRY_PUBLISHER_IDENTITY_MISMATCH");
    } else if (odometry_publisher_count_ == 1U && !odometry_publisher_type_valid_) {
      ++odometry_publisher_authority_violation_;
      health_gate_.MarkLatched("ODOMETRY_PUBLISHER_TYPE_MISMATCH");
    } else if (status_publisher_count_ > 1U) {
      ++status_publisher_authority_violation_;
      health_gate_.MarkLatched("TRACKING_STATUS_PUBLISHER_NOT_UNIQUE");
    } else if (status_publisher_count_ == 1U && !status_publisher_identity_valid_) {
      ++status_publisher_authority_violation_;
      health_gate_.MarkLatched("TRACKING_STATUS_PUBLISHER_IDENTITY_MISMATCH");
    } else if (status_publisher_count_ == 1U && !status_publisher_type_valid_) {
      ++status_publisher_authority_violation_;
      health_gate_.MarkLatched("TRACKING_STATUS_PUBLISHER_TYPE_MISMATCH");
    } else if (candidate_publisher_count_ > 1U) {
      ++candidate_publisher_authority_violation_;
      health_gate_.MarkLatched("SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE");
    } else if (candidate_publisher_authority_invalid) {
      ++candidate_publisher_authority_violation_;
      health_gate_.MarkLatched("SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID");
    }
  }
  if (health_gate_.State() != HealthState::kLatchedFault) {
    std::string reason;
    if (!InputsAreHealthyLocked(now, &reason)) {
      const double startup_age = std::chrono::duration<double>(now - started_at_).count();
      const bool waiting = reason.rfind("WAITING_FOR_", 0U) == 0U;
      if (waiting && startup_age <= contract_.health.startup_timeout_sec &&
        health_gate_.State() == HealthState::kStarting)
      {
        health_gate_.MarkStarting(reason);
      } else if (waiting && health_gate_.State() == HealthState::kStarting) {
        health_gate_.MarkTransient("STARTUP_TIMEOUT");
      } else {
        health_gate_.MarkTransient(reason);
      }
    } else {
      MaybeAdvanceHealthLocked(now);
    }
  }
  PublishDiagnosticsLocked(now);
}

void CuvslamLocalizationAdapter::PublishSourceCandidateLocked(
  const builtin_interfaces::msg::Time & stamp)
{
  if (!CandidatePublisherIsAuthoritativeLocked()) {
    return;
  }
  if (!last_shadow_pose_.has_value()) {
    health_gate_.MarkLatched("SOURCE_CANDIDATE_POSE_MISSING");
    return;
  }

  localization_adapter_interfaces::msg::LocalizationSourceCandidate output;
  output.header.stamp = stamp;
  output.header.frame_id = contract_.output_parent_frame;
  output.semantic_child_frame = contract_.output_child_frame;
  const Eigen::Vector3d & translation = last_shadow_pose_->Translation();
  output.pose.position.x = translation.x();
  output.pose.position.y = translation.y();
  output.pose.position.z = translation.z();
  const QuaternionXyzw rotation = last_shadow_pose_->RotationXyzw();
  output.pose.orientation.x = rotation.x;
  output.pose.orientation.y = rotation.y;
  output.pose.orientation.z = rotation.z;
  output.pose.orientation.w = rotation.w;
  output.source_id = kSourceId;
  output.source_contract_id = contract_.contract_id;
  output.authorization = kCandidateAuthorization;
  source_candidate_publisher_->publish(output);
  ++source_candidates_published_;
}

void CuvslamLocalizationAdapter::PublishDiagnosticsLocked(const SteadyTime & now)
{
  const auto decision = localization_contracts::EvaluateLocalizationPublishGate(
    mode_, health_gate_.State(), contract_.authorization, contract_.Approvals());
  DiagnosticStatus status;
  status.name = diagnostic_status_name_;
  status.hardware_id = contract_.contract_id;
  switch (health_gate_.State()) {
    case HealthState::kStarting:
      status.level = DiagnosticStatus::STALE;
      status.message = "waiting for complete upstream health evidence";
      break;
    case HealthState::kTransientFault:
      status.level = DiagnosticStatus::ERROR;
      status.message = "transient localization contract fault";
      break;
    case HealthState::kRecovering:
      status.level = DiagnosticStatus::WARN;
      status.message = "localization inputs are recovering";
      break;
    case HealthState::kLatchedFault:
      status.level = DiagnosticStatus::ERROR;
      status.message = "latched localization contract fault; restart required";
      break;
    case HealthState::kHealthy:
      status.level = decision.publish ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
      status.message = decision.publish ?
        "localization publication is authorized" :
        "inputs are healthy; standard odometry remains blocked by contract";
      break;
  }

  status.values = {
    Value("contract_id", contract_.contract_id),
    Value("contract_source_sha256", contract_.source_sha256),
    Value("mode", mode_),
    Value("health_state", localization_contracts::ToString(health_gate_.State())),
    Value("authorization_state", localization_contracts::ToString(contract_.authorization)),
    Value("reason_code", health_gate_.Reason()),
    Value("publish_gate_reasons", Join(decision.reasons)),
    Value("latched", health_gate_.State() == HealthState::kLatchedFault),
    Value("input_odometry_topic", contract_.input.odometry_topic),
    Value("expected_odometry_publisher", contract_.input.expected_odometry_publisher),
    Value("expected_status_publisher", contract_.input.expected_status_publisher),
    Value("expected_parent_frame", contract_.input.expected_parent_frame),
    Value("expected_child_frame", contract_.input.expected_child_frame),
    Value("output_topic_contract", contract_.output_topic),
    Value("output_parent_frame_contract", contract_.output_parent_frame),
    Value("output_child_frame_contract", contract_.output_child_frame),
    Value("last_actual_parent_frame", last_actual_parent_frame_),
    Value("last_actual_child_frame", last_actual_child_frame_),
    Value("odometry_publisher_count", odometry_publisher_count_),
    Value("status_publisher_count", status_publisher_count_),
    Value("odometry_publisher_identity_valid", odometry_publisher_identity_valid_),
    Value("status_publisher_identity_valid", status_publisher_identity_valid_),
    Value("odometry_publisher_type_valid", odometry_publisher_type_valid_),
    Value("status_publisher_type_valid", status_publisher_type_valid_),
    Value(
      "bound_odometry_publisher_gid",
      bound_odometry_publisher_gid_.has_value() ?
      GidToString(bound_odometry_publisher_gid_.value()) : "unbound"),
    Value(
      "bound_status_publisher_gid",
      bound_status_publisher_gid_.has_value() ?
      GidToString(bound_status_publisher_gid_.value()) : "unbound"),
    Value("source_candidate_topic", kCandidateTopic),
    Value("source_candidate_type", kCandidateMessageType),
    Value("source_candidate_authorization", kCandidateAuthorization),
    Value("source_candidate_publisher_count", candidate_publisher_count_),
    Value(
      "source_candidate_publisher_identity_valid",
      candidate_publisher_identity_valid_),
    Value("source_candidate_publisher_type_valid", candidate_publisher_type_valid_),
    Value("source_candidate_publisher_gid_valid", candidate_publisher_gid_valid_),
    Value("last_vo_state", static_cast<unsigned int>(last_vo_state_)),
    Value("received", received_),
    Value("accepted", accepted_),
    Value("rejected", rejected_),
    Value("duplicate", duplicate_),
    Value("nonmonotonic", nonmonotonic_),
    Value("status_received", status_received_),
    Value("status_rejected", status_rejected_),
    Value("status_duplicate", status_duplicate_),
    Value("status_nonmonotonic", status_nonmonotonic_),
    Value("diagnostic_evidence_received", diagnostic_evidence_received_),
    Value("diagnostic_evidence_rejected", diagnostic_evidence_rejected_),
    Value("diagnostic_duplicate", diagnostic_duplicate_),
    Value("diagnostic_nonmonotonic", diagnostic_nonmonotonic_),
    Value("clock_domain_mismatch", clock_domain_mismatch_),
    Value("odometry_stamp_gap_violation", odometry_stamp_gap_violation_),
    Value("odometry_receive_gap_violation", odometry_receive_gap_violation_),
    Value("odometry_rate_violation", odometry_rate_violation_),
    Value("pose_jump_violation", pose_jump_violation_),
    Value(
      "reset_detection_policy",
      "large_pose_step_heuristic; explicit epoch signal unavailable"),
    Value("shadow_processed", shadow_processed_),
    Value("shadow_pose_computed", shadow_pose_computed_),
    Value("published_source_candidates", source_candidates_published_),
    Value(
      "odometry_publisher_authority_violation",
      odometry_publisher_authority_violation_),
    Value(
      "status_publisher_authority_violation",
      status_publisher_authority_violation_),
    Value(
      "source_candidate_publisher_authority_violation",
      candidate_publisher_authority_violation_),
    Value("published", kPublished),
    Value("recovery_progress", health_gate_.RecoveryProgress()),
    Value("recovery_required", health_gate_.RecoveryRequired()),
    Value("extrinsic_status", contract_.extrinsic.status),
    Value("twist_status", contract_.twist.status),
    Value("covariance_status", contract_.covariance.status),
  };
  const auto stamp = stamp_guard_.LastAcceptedNanoseconds();
  status.values.push_back(
    Value(
      "last_input_stamp_ns", stamp.has_value() ? std::to_string(stamp.value()) : "not_received"));
  const auto sequence_stamp = odometry_sequence_guard_.LastAcceptedNanoseconds();
  status.values.push_back(
    Value(
      "last_sequence_stamp_ns",
      sequence_stamp.has_value() ?
      std::to_string(sequence_stamp.value()) : "not_received"));
  const auto status_stamp = status_stamp_guard_.LastAcceptedNanoseconds();
  status.values.push_back(
    Value(
      "last_status_stamp_ns",
      status_stamp.has_value() ? std::to_string(status_stamp.value()) : "not_received"));
  const auto odometry_age = AgeSeconds(last_odometry_received_at_, now);
  status.values.push_back(
    Value(
      "last_receive_age_sec",
      odometry_age.has_value() ? std::to_string(odometry_age.value()) : "not_received"));
  status.values.push_back(
    Value(
      "observed_odometry_rate_hz",
      observed_odometry_rate_hz_.has_value() ?
      std::to_string(observed_odometry_rate_hz_.value()) : "warming_up"));
  status.values.push_back(
    Value(
      "maximum_recent_stamp_gap_sec",
      maximum_recent_stamp_gap_sec_.has_value() ?
      std::to_string(maximum_recent_stamp_gap_sec_.value()) : "warming_up"));
  status.values.push_back(
    Value(
      "last_odometry_receive_gap_sec",
      last_odometry_receive_gap_sec_.has_value() ?
      std::to_string(last_odometry_receive_gap_sec_.value()) : "not_received"));
  status.values.push_back(
    Value(
      "last_odometry_clock_residual_sec",
      last_odometry_clock_residual_sec_.has_value() ?
      std::to_string(last_odometry_clock_residual_sec_.value()) : "not_received"));
  status.values.push_back(
    Value(
      "last_status_clock_residual_sec",
      last_status_clock_residual_sec_.has_value() ?
      std::to_string(last_status_clock_residual_sec_.value()) : "not_received"));
  status.values.push_back(
    Value(
      "last_diagnostic_clock_residual_sec",
      last_diagnostic_clock_residual_sec_.has_value() ?
      std::to_string(last_diagnostic_clock_residual_sec_.value()) : "not_received"));
  if (last_shadow_pose_.has_value()) {
    const Eigen::Vector3d & position = last_shadow_pose_->Translation();
    status.values.push_back(Value("shadow_base_x", position.x()));
    status.values.push_back(Value("shadow_base_y", position.y()));
    status.values.push_back(Value("shadow_base_z", position.z()));
  }

  DiagnosticArray output;
  output.header.stamp = get_clock()->now();
  output.status.push_back(std::move(status));
  diagnostics_publisher_->publish(output);
}

}  // namespace cuvslam_localization_adapter
