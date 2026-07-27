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

#include "localization_source_selector/selector_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace localization_source_selector
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::chrono::seconds kStartupGraphGrace{5};
constexpr char kCandidateMessageType[] =
  "localization_adapter_interfaces/msg/LocalizationSourceCandidate";
constexpr char kSelectedMessageType[] =
  "localization_adapter_interfaces/msg/SelectedPoseCandidate";
constexpr char kUnknownNodeName[] = "_NODE_NAME_UNKNOWN_";
constexpr char kUnknownNodeNamespace[] = "_NODE_NAMESPACE_UNKNOWN_";

rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

std::string DefaultContractFile()
{
  return ament_index_cpp::get_package_share_directory("localization_source_selector") +
         "/config/cuvslam_primary.contract.yaml";
}

std::optional<std::int64_t> StampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
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
  const std::int64_t result =
    seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(stamp.nanosec);
  return result > 0 ? std::optional<std::int64_t>(result) : std::nullopt;
}

std::string FullyQualifiedNodeName(const rclcpp::TopicEndpointInfo & endpoint)
{
  const std::string node_namespace = endpoint.node_namespace();
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + endpoint.node_name();
  }
  return node_namespace + (node_namespace.back() == '/' ? "" : "/") +
         endpoint.node_name();
}

bool EndpointIdentityIsUnknown(const rclcpp::TopicEndpointInfo & endpoint)
{
  return endpoint.node_name() == kUnknownNodeName ||
         endpoint.node_namespace() == kUnknownNodeNamespace;
}

template<typename Gid>
bool GidsEqual(const Gid & left, const Gid & right)
{
  return std::equal(left.begin(), left.end(), right.begin(), right.end());
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

std::string ReliabilityToString(const rmw_qos_reliability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "best_effort";
    case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
      return "unknown";
  }
  return "invalid";
}

std::string DurabilityToString(const rmw_qos_durability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "transient_local";
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "volatile";
    case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
      return "unknown";
  }
  return "invalid";
}

std::string HistoryToString(const rmw_qos_history_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      return "system_default";
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      return "keep_last";
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return "keep_all";
    case RMW_QOS_POLICY_HISTORY_UNKNOWN:
      return "unknown";
  }
  return "invalid";
}

std::string GenerateEpochId()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch().count();
  std::random_device random;
  std::ostringstream stream;
  stream << "selector-" << std::hex << now << '-' << random();
  return stream.str();
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

bool PoseComponentsAreFinite(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

bool CandidatePublisherQosIsCompatible(
  const rmw_qos_profile_t & qos,
  const QosContractConfig & expected) noexcept
{
  const bool history_matches =
    qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ||
    (expected.allow_rmw_unknown_history &&
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN);
  const bool depth_matches =
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN || qos.depth == expected.depth;
  return expected.reliability == "reliable" &&
         expected.durability == "volatile" && expected.history == "keep_last" &&
         qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
         qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
         history_matches && depth_matches;
}

LocalizationSourceSelector::LocalizationSourceSelector(const rclcpp::NodeOptions & options)
: Node("localization_source_selector", options)
{
  mode_ = declare_parameter<std::string>(
    "mode", "cuvslam_primary",
    ReadOnlyDescriptor(
      "Launch-time source mode; only cuvslam_primary or mocap_primary is accepted."));
  contract_file_ = declare_parameter<std::string>(
    "contract_file", DefaultContractFile(),
    ReadOnlyDescriptor("Version-controlled selector contract YAML."));
  contract_ = LoadContractConfig(contract_file_);
  ValidateModeContract(contract_, mode_);
  startup_graph_deadline_ = std::chrono::steady_clock::now() + kStartupGraphGrace;
  localization_epoch_id_ = GenerateEpochId();
  diagnostic_status_name_ =
    std::string(get_fully_qualified_name()) + ": localization source selector";

  if (std::string(get_fully_qualified_name()) != contract_.output.expected_publisher) {
    throw std::runtime_error(
            "selector node name or namespace remapping is forbidden by the contract");
  }

  const auto require_unremapped_topic = [this](const std::string & topic) {
      if (get_node_topics_interface()->resolve_topic_name(topic, false) != topic) {
        throw std::runtime_error("ROS remapping of selector contract topics is forbidden");
      }
    };
  require_unremapped_topic(contract_.input.topic);
  require_unremapped_topic(contract_.output.topic);
  require_unremapped_topic("/diagnostics");

  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(contract_.input.qos.depth))
    .reliable().durability_volatile();
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(contract_.output.qos.depth))
    .reliable().durability_volatile();
  const auto diagnostics_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  selected_publisher_ = create_publisher<SelectedPoseCandidate>(
    contract_.output.topic, output_qos);
  const auto & local_output_gid = selected_publisher_->get_gid();
  std::copy_n(
    local_output_gid.data, RMW_GID_STORAGE_SIZE, selected_publisher_gid_.begin());
  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    "/diagnostics", diagnostics_qos);
  source_subscription_ = create_subscription<LocalizationSourceCandidate>(
    contract_.input.topic,
    input_qos,
    std::bind(
      &LocalizationSourceSelector::OnCandidate,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  if (std::string(selected_publisher_->get_topic_name()) != contract_.output.topic ||
    std::string(diagnostics_publisher_->get_topic_name()) != "/diagnostics" ||
    std::string(source_subscription_->get_topic_name()) != contract_.input.topic)
  {
    throw std::runtime_error("ROS remapping of selector contract topics is forbidden");
  }

  const auto diagnostic_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(contract_.health.diagnostic_period_sec));
  diagnostic_timer_ = create_wall_timer(
    diagnostic_period,
    std::bind(&LocalizationSourceSelector::OnDiagnosticTimer, this));

  RCLCPP_WARN(
    get_logger(),
    "Loaded %s selector contract %s; automatic source switching and privileged outputs are "
    "disabled",
    mode_.c_str(), contract_.selector_contract_id.c_str());
}

LocalizationSourceSelector::TopicEndpointInfoList
LocalizationSourceSelector::GetPublishersInfoByTopic(const std::string & topic)
{
  return get_publishers_info_by_topic(topic);
}

void LocalizationSourceSelector::OnCandidate(
  const LocalizationSourceCandidate::ConstSharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  last_actual_parent_frame_ = message->header.frame_id;
  last_actual_child_frame_ = message->semantic_child_frame;
  last_actual_source_id_ = message->source_id;
  last_actual_source_contract_id_ = message->source_contract_id;
  last_actual_authorization_ = message->authorization;

  if (state_ == State::kLatchedFault || !ValidateMessagePublisherLocked(message_info)) {
    ++rejected_;
    return;
  }
  if (message->source_id != contract_.input.source_id ||
    message->source_contract_id != contract_.input.source_contract_id ||
    message->authorization != contract_.input.authorization)
  {
    ++rejected_;
    ++contract_violation_;
    LatchLocked("SOURCE_CONTRACT_MISMATCH");
    return;
  }
  if (message->header.frame_id != contract_.input.parent_frame ||
    message->semantic_child_frame != contract_.input.semantic_child_frame)
  {
    ++rejected_;
    ++frame_violation_;
    LatchLocked("SOURCE_FRAME_MISMATCH");
    return;
  }
  if (!PoseComponentsAreFinite(message->pose)) {
    ++rejected_;
    ++nonfinite_pose_;
    LatchLocked("NONFINITE_SOURCE_POSE");
    return;
  }
  if (!PoseIsFiniteAndNormalized(message->pose)) {
    ++rejected_;
    ++quaternion_violation_;
    LatchLocked("INVALID_SOURCE_QUATERNION");
    return;
  }

  const auto stamp_ns = StampToNanoseconds(message->header.stamp);
  if (!stamp_ns.has_value()) {
    ++rejected_;
    ++clock_violation_;
    MarkRecoveringLocked("INVALID_SOURCE_TIMESTAMP");
    return;
  }
  if (last_accepted_stamp_ns_.has_value()) {
    if (stamp_ns.value() == last_accepted_stamp_ns_.value()) {
      ++rejected_;
      ++duplicate_;
      MarkRecoveringLocked("DUPLICATE_SOURCE_TIMESTAMP");
      return;
    }
    if (stamp_ns.value() < last_accepted_stamp_ns_.value()) {
      ++rejected_;
      ++nonmonotonic_;
      LatchLocked("SOURCE_TIME_REGRESSION");
      return;
    }
  }

  const std::int64_t now_ns = system_clock_.now().nanoseconds();
  const double clock_residual_sec =
    std::abs(static_cast<double>(now_ns) - static_cast<double>(stamp_ns.value())) /
    static_cast<double>(kNanosecondsPerSecond);
  if (now_ns <= 0 || !std::isfinite(clock_residual_sec) ||
    clock_residual_sec > contract_.health.maximum_clock_residual_sec)
  {
    ++rejected_;
    ++clock_violation_;
    MarkRecoveringLocked("SOURCE_TIMESTAMP_NOT_FRESH");
    return;
  }

  if (last_valid_receive_time_.has_value() && state_ == State::kHealthy) {
    const double receive_gap_sec =
      std::chrono::duration<double>(now - last_valid_receive_time_.value()).count();
    if (receive_gap_sec > contract_.health.stale_after_sec) {
      state_ = State::kRecovering;
      reason_code_ = "RECOVERING_AFTER_STALE";
      recovery_progress_ = 0U;
      ++stale_events_;
    }
  }

  if (last_source_pose_.has_value()) {
    const double translation_step = PoseStepTranslation(last_source_pose_.value(), message->pose);
    const double rotation_step = PoseStepRotation(last_source_pose_.value(), message->pose);
    if (translation_step > contract_.health.maximum_position_step_m ||
      rotation_step > contract_.health.maximum_rotation_step_rad)
    {
      ++rejected_;
      ++pose_reset_;
      LatchLocked("SOURCE_POSE_RESET_DETECTED");
      return;
    }
  }

  if (!alignment_.has_value()) {
    try {
      if (mode_ == "mocap_primary") {
        alignment_ = YawAlignment::Identity();
      } else {
        alignment_.emplace(message->pose);
      }
    } catch (const std::exception &) {
      ++rejected_;
      ++quaternion_violation_;
      LatchLocked("ALIGNMENT_INITIALIZATION_FAILED");
      return;
    }
  }

  geometry_msgs::msg::Pose aligned_pose;
  try {
    aligned_pose = alignment_->Transform(message->pose);
  } catch (const std::exception &) {
    ++rejected_;
    LatchLocked("ALIGNMENT_TRANSFORM_FAILED");
    return;
  }
  if (!PoseComponentsAreFinite(aligned_pose)) {
    ++rejected_;
    ++nonfinite_pose_;
    LatchLocked("NONFINITE_ALIGNED_POSE");
    return;
  }
  if (!PoseIsFiniteAndNormalized(aligned_pose)) {
    ++rejected_;
    ++quaternion_violation_;
    LatchLocked("INVALID_ALIGNED_QUATERNION");
    return;
  }

  last_valid_receive_time_ = now;
  last_accepted_stamp_ns_ = stamp_ns;
  last_source_pose_ = message->pose;
  ++accepted_;

  bool output_authority_valid = false;
  if (state_ == State::kStarting) {
    output_authority_valid = OutputAuthorityValidLocked();
    if (!output_authority_valid) {
      return;
    }
    state_ = State::kHealthy;
    reason_code_ = "SOURCE_HEALTHY";
  } else if (state_ == State::kStale) {
    state_ = State::kRecovering;
    reason_code_ = "RECOVERING_SAME_SOURCE";
    recovery_progress_ = 1U;
  } else if (state_ == State::kRecovering) {
    ++recovery_progress_;
  }
  if (state_ == State::kRecovering &&
    recovery_progress_ >= contract_.health.recovery_consecutive_samples)
  {
    state_ = State::kHealthy;
    reason_code_ = "SOURCE_HEALTHY";
    recovery_progress_ = contract_.health.recovery_consecutive_samples;
  }

  if (state_ == State::kHealthy &&
    (output_authority_valid || OutputAuthorityValidLocked()))
  {
    PublishSelectedLocked(*message, aligned_pose);
  }
}

bool LocalizationSourceSelector::ValidateMessagePublisherLocked(
  const rclcpp::MessageInfo & message_info)
{
  UpdateInputAuthorityLocked();
  if (state_ == State::kLatchedFault || input_publisher_count_ != 1U) {
    return false;
  }

  PublisherGid message_gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, message_gid.begin());
  const auto endpoints = GetPublishersInfoByTopic(contract_.input.topic);
  if (endpoints.empty() &&
    StartupGraphGraceActiveLocked(bound_publisher_gid_.has_value()))
  {
    MarkStartupGraphWaitLocked("WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
    return false;
  }
  if (endpoints.size() > 1U) {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_NOT_UNIQUE");
    return false;
  }
  if (endpoints.empty()) {
    ++input_authority_violation_;
    LatchLocked("SOURCE_MESSAGE_GID_NOT_UNIQUE");
    return false;
  }
  const auto & endpoint = endpoints.front();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  if (endpoint.topic_type() != kCandidateMessageType ||
    !CandidatePublisherQosIsCompatible(qos, contract_.input.qos))
  {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (StartupGraphGraceActiveLocked(bound_publisher_gid_.has_value())) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
    } else {
      ++input_authority_violation_;
      LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (FullyQualifiedNodeName(endpoint) != contract_.input.expected_publisher) {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  if (!GidsEqual(endpoint.endpoint_gid(), message_gid)) {
    if (StartupGraphGraceActiveLocked(bound_publisher_gid_.has_value())) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
    } else {
      ++input_authority_violation_;
      LatchLocked("SOURCE_MESSAGE_GID_NOT_UNIQUE");
    }
    return false;
  }
  if (bound_publisher_gid_.has_value() &&
    !GidsEqual(bound_publisher_gid_.value(), message_gid))
  {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_GID_CHANGED");
    return false;
  }
  if (!bound_publisher_gid_.has_value()) {
    bound_publisher_gid_ = message_gid;
  }
  return true;
}

void LocalizationSourceSelector::UpdateInputAuthorityLocked()
{
  const auto endpoints = GetPublishersInfoByTopic(contract_.input.topic);
  input_publisher_count_ = endpoints.size();
  actual_input_publisher_ = "not_unique_or_missing";
  actual_input_type_ = "not_unique_or_missing";
  actual_qos_reliability_ = "not_unique_or_missing";
  actual_qos_durability_ = "not_unique_or_missing";
  actual_qos_history_ = "not_unique_or_missing";
  actual_qos_depth_ = 0U;

  if (endpoints.size() > 1U) {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_NOT_UNIQUE");
    return;
  }
  if (endpoints.empty()) {
    if (StartupGraphGraceActiveLocked(bound_publisher_gid_.has_value())) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
    } else {
      ++input_authority_violation_;
      LatchLocked(
        bound_publisher_gid_.has_value() ?
        "SOURCE_PUBLISHER_EPOCH_CHANGED" : "SOURCE_PUBLISHER_GRAPH_TIMEOUT");
    }
    return;
  }

  const auto & endpoint = endpoints.front();
  actual_input_publisher_ = FullyQualifiedNodeName(endpoint);
  actual_input_type_ = endpoint.topic_type();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  actual_qos_reliability_ = ReliabilityToString(qos.reliability);
  actual_qos_durability_ = DurabilityToString(qos.durability);
  actual_qos_history_ = HistoryToString(qos.history);
  actual_qos_depth_ = qos.depth;

  if (actual_input_type_ != kCandidateMessageType ||
    !CandidatePublisherQosIsCompatible(qos, contract_.input.qos))
  {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    return;
  }
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (StartupGraphGraceActiveLocked(bound_publisher_gid_.has_value())) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
    } else {
      ++input_authority_violation_;
      LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return;
  }
  if (actual_input_publisher_ != contract_.input.expected_publisher) {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
    return;
  }
  if (bound_publisher_gid_.has_value() &&
    !GidsEqual(bound_publisher_gid_.value(), endpoint.endpoint_gid()))
  {
    ++input_authority_violation_;
    LatchLocked("SOURCE_PUBLISHER_EPOCH_CHANGED");
  }
}

bool LocalizationSourceSelector::OutputAuthorityValidLocked()
{
  const auto endpoints = GetPublishersInfoByTopic(contract_.output.topic);
  output_publisher_count_ = endpoints.size();
  output_publisher_gid_valid_ = false;
  if (endpoints.empty()) {
    if (StartupGraphGraceActiveLocked(output_authority_was_validated_)) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SELECTED_OUTPUT_GRAPH");
    } else {
      ++output_authority_violation_;
      LatchLocked("SELECTED_OUTPUT_PUBLISHER_MISSING");
    }
    return false;
  }
  if (endpoints.size() > 1U) {
    ++output_authority_violation_;
    LatchLocked("SELECTED_OUTPUT_PUBLISHER_NOT_UNIQUE");
    return false;
  }
  const auto & endpoint = endpoints.front();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  if (endpoint.topic_type() != kSelectedMessageType ||
    !CandidatePublisherQosIsCompatible(qos, contract_.output.qos))
  {
    ++output_authority_violation_;
    LatchLocked("SELECTED_OUTPUT_AUTHORITY_MISMATCH");
    return false;
  }
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (StartupGraphGraceActiveLocked(output_authority_was_validated_)) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SELECTED_OUTPUT_GRAPH");
    } else {
      ++output_authority_violation_;
      LatchLocked("SELECTED_OUTPUT_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (FullyQualifiedNodeName(endpoint) != contract_.output.expected_publisher) {
    ++output_authority_violation_;
    LatchLocked("SELECTED_OUTPUT_AUTHORITY_MISMATCH");
    return false;
  }
  if (!GidsEqual(endpoint.endpoint_gid(), selected_publisher_gid_)) {
    if (StartupGraphGraceActiveLocked(output_authority_was_validated_)) {
      MarkStartupGraphWaitLocked("WAITING_FOR_SELECTED_OUTPUT_GRAPH");
    } else {
      ++output_authority_violation_;
      LatchLocked("SELECTED_OUTPUT_PUBLISHER_GID_MISMATCH");
    }
    return false;
  }
  output_publisher_gid_valid_ = true;
  if (reason_code_ == "WAITING_FOR_SELECTED_OUTPUT_GRAPH") {
    reason_code_ = state_ == State::kHealthy ? "SOURCE_HEALTHY" : "WAITING_FOR_SOURCE";
  }
  return true;
}

bool LocalizationSourceSelector::StartupGraphGraceActiveLocked(
  const bool authority_was_validated) const
{
  return !authority_was_validated &&
         std::chrono::steady_clock::now() <= startup_graph_deadline_;
}

void LocalizationSourceSelector::MarkStartupGraphWaitLocked(const std::string & reason)
{
  if (reason == "WAITING_FOR_SELECTED_OUTPUT_GRAPH" &&
    !bound_publisher_gid_.has_value())
  {
    return;
  }
  reason_code_ = reason;
}

void LocalizationSourceSelector::MarkRecoveringLocked(const std::string & reason)
{
  if (state_ == State::kLatchedFault) {
    return;
  }
  reason_code_ = reason;
  recovery_progress_ = 0U;
  state_ = alignment_.has_value() ? State::kRecovering : State::kStarting;
}

void LocalizationSourceSelector::LatchLocked(const std::string & reason)
{
  state_ = State::kLatchedFault;
  reason_code_ = reason;
  recovery_progress_ = 0U;
}

void LocalizationSourceSelector::PublishSelectedLocked(
  const LocalizationSourceCandidate & input,
  const geometry_msgs::msg::Pose & aligned_pose)
{
  SelectedPoseCandidate output;
  output.header.stamp = input.header.stamp;
  output.header.frame_id = contract_.output.parent_frame;
  output.semantic_child_frame = contract_.output.semantic_child_frame;
  output.pose = aligned_pose;
  output.selector_contract_id = contract_.selector_contract_id;
  output.localization_epoch_id = localization_epoch_id_;
  output.mode = mode_;
  output.source_contract_id = contract_.input.source_contract_id;
  output.authorization = contract_.output.authorization;
  selected_publisher_->publish(output);
  ++published_;
  output_authority_was_validated_ = true;
}

void LocalizationSourceSelector::OnDiagnosticTimer()
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  UpdateInputAuthorityLocked();
  if (state_ != State::kLatchedFault) {
    (void)OutputAuthorityValidLocked();
  }
  if (state_ != State::kLatchedFault && state_ != State::kStarting &&
    last_valid_receive_time_.has_value())
  {
    const double age_sec =
      std::chrono::duration<double>(now - last_valid_receive_time_.value()).count();
    if (age_sec > contract_.health.stale_after_sec &&
      state_ != State::kStale)
    {
      state_ = State::kStale;
      reason_code_ = "SOURCE_STALE";
      recovery_progress_ = 0U;
      ++stale_events_;
    }
  }
  PublishDiagnosticsLocked(now);
}

void LocalizationSourceSelector::PublishDiagnosticsLocked(const SteadyTime & now)
{
  DiagnosticStatus status;
  status.name = diagnostic_status_name_;
  status.hardware_id = contract_.selector_contract_id;
  switch (state_) {
    case State::kStarting:
      status.level = DiagnosticStatus::STALE;
      status.message = "waiting for the launch-selected localization source";
      break;
    case State::kHealthy:
      status.level = DiagnosticStatus::OK;
      status.message = "selected pose candidate is healthy; no PX4 or control authority";
      break;
    case State::kStale:
      status.level = DiagnosticStatus::ERROR;
      status.message = "selected source is stale; selected pose output is stopped";
      break;
    case State::kRecovering:
      status.level = DiagnosticStatus::WARN;
      status.message = "same selected source epoch is recovering";
      break;
    case State::kLatchedFault:
      status.level = DiagnosticStatus::ERROR;
      status.message = "selector contract fault is latched; restart required";
      break;
  }

  const std::optional<double> receive_age = last_valid_receive_time_.has_value() ?
    std::optional<double>(
    std::chrono::duration<double>(now - last_valid_receive_time_.value()).count()) :
    std::nullopt;
  status.values = {
    Value("mode", mode_),
    Value("selected_source", contract_.input.source_id),
    Value("selector_contract_id", contract_.selector_contract_id),
    Value("source_contract_id", contract_.input.source_contract_id),
    Value("localization_epoch_id", localization_epoch_id_),
    Value("authorization", contract_.output.authorization),
    Value("state", StateToString(state_)),
    Value("reason_code", reason_code_),
    Value("latched", state_ == State::kLatchedFault),
    Value("expected_input_topic", contract_.input.topic),
    Value("expected_input_type", contract_.input.type),
    Value("expected_input_publisher", contract_.input.expected_publisher),
    Value("actual_input_publisher", actual_input_publisher_),
    Value("actual_input_type", actual_input_type_),
    Value("input_publisher_count", input_publisher_count_),
    Value(
      "bound_publisher_gid",
      bound_publisher_gid_.has_value() ?
      GidToString(bound_publisher_gid_.value()) : "unbound"),
    Value("actual_qos_reliability", actual_qos_reliability_),
    Value("actual_qos_durability", actual_qos_durability_),
    Value("actual_qos_history", actual_qos_history_),
    Value("actual_qos_depth", actual_qos_depth_),
    Value("expected_parent_frame", contract_.input.parent_frame),
    Value("actual_parent_frame", last_actual_parent_frame_),
    Value("expected_semantic_child_frame", contract_.input.semantic_child_frame),
    Value("actual_semantic_child_frame", last_actual_child_frame_),
    Value("actual_source_id", last_actual_source_id_),
    Value("actual_source_contract_id", last_actual_source_contract_id_),
    Value("actual_source_authorization", last_actual_authorization_),
    Value(
      "last_source_stamp_ns",
      last_accepted_stamp_ns_.has_value() ?
      std::to_string(last_accepted_stamp_ns_.value()) : "not_received"),
    Value(
      "last_valid_receive_age_sec",
      receive_age.has_value() ? std::to_string(receive_age.value()) : "not_received"),
    Value("alignment_locked", alignment_.has_value()),
    Value(
      "pose_reference_semantics",
      mode_ == "mocap_primary" ? "global_mocap_world" : "epoch_local_initial_pose"),
    Value(
      "alignment_yaw_map_from_source_rad",
      alignment_.has_value() ?
      std::to_string(alignment_->YawMapFromSource()) : "not_initialized"),
    Value(
      "alignment_translation_x_m",
      alignment_.has_value() ?
      std::to_string(alignment_->TranslationMapFromSource().x()) : "not_initialized"),
    Value(
      "alignment_translation_y_m",
      alignment_.has_value() ?
      std::to_string(alignment_->TranslationMapFromSource().y()) : "not_initialized"),
    Value(
      "alignment_translation_z_m",
      alignment_.has_value() ?
      std::to_string(alignment_->TranslationMapFromSource().z()) : "not_initialized"),
    Value("recovery_progress", recovery_progress_),
    Value("recovery_required", contract_.health.recovery_consecutive_samples),
    Value("received", received_),
    Value("accepted", accepted_),
    Value("published", published_),
    Value("rejected", rejected_),
    Value("stale_events", stale_events_),
    Value("duplicate", duplicate_),
    Value("nonmonotonic", nonmonotonic_),
    Value("frame_violation", frame_violation_),
    Value("contract_violation", contract_violation_),
    Value("nonfinite_pose", nonfinite_pose_),
    Value("quaternion_violation", quaternion_violation_),
    Value("clock_violation", clock_violation_),
    Value("pose_reset", pose_reset_),
    Value("input_authority_violation", input_authority_violation_),
    Value("output_publisher_count", output_publisher_count_),
    Value("output_publisher_gid", GidToString(selected_publisher_gid_)),
    Value("output_publisher_gid_valid", output_publisher_gid_valid_),
    Value("output_authority_violation", output_authority_violation_),
    Value("canonical_odometry_authorized", false),
    Value("tf_authorized", false),
    Value("mavros_authorized", false),
    Value("control_authorized", false),
  };

  DiagnosticArray output;
  output.header.stamp = system_clock_.now();
  output.status.push_back(std::move(status));
  diagnostics_publisher_->publish(output);
}

std::string LocalizationSourceSelector::StateToString(const State state)
{
  switch (state) {
    case State::kStarting:
      return "starting";
    case State::kHealthy:
      return "healthy";
    case State::kStale:
      return "stale";
    case State::kRecovering:
      return "recovering";
    case State::kLatchedFault:
      return "latched_fault";
  }
  return "invalid";
}

}  // namespace localization_source_selector
