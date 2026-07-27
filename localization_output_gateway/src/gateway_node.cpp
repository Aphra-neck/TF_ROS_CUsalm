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

#include "localization_output_gateway/gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace localization_output_gateway
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr double kMaximumQuaternionNormError = 1.0e-3;
constexpr char kSelectedPoseType[] =
  "localization_adapter_interfaces/msg/SelectedPoseCandidate";
constexpr char kExternalVisionType[] =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
constexpr char kUnknownNodeName[] = "_NODE_NAME_UNKNOWN_";
constexpr char kUnknownNodeNamespace[] = "_NODE_NAMESPACE_UNKNOWN_";
constexpr char kMavrosStateType[] = "mavros_msgs/msg/State";
constexpr char kTimesyncStatusType[] = "mavros_msgs/msg/TimesyncStatus";

rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

std::string DefaultContractFile()
{
  return ament_index_cpp::get_package_share_directory("localization_output_gateway") +
         "/config/disabled.contract.yaml";
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

bool EndpointIdentityIsUnknown(const rclcpp::TopicEndpointInfo & endpoint) noexcept
{
  return endpoint.node_name() == kUnknownNodeName ||
         endpoint.node_namespace() == kUnknownNodeNamespace;
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

bool QosIsCompatible(
  const rmw_qos_profile_t & qos,
  const QosContractConfig & expected) noexcept
{
  const bool history_matches =
    qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ||
    (expected.allow_rmw_unknown_history &&
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN);
  const bool depth_matches =
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN || qos.depth == expected.depth;
  return qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
         qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
         history_matches && depth_matches;
}

bool MavrosStateQosIsCompatible(const rmw_qos_profile_t & qos) noexcept
{
  return qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
         qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
}

bool TimesyncQosIsCompatible(const rmw_qos_profile_t & qos) noexcept
{
  return qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
         qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE;
}

std::optional<std::int64_t> StampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds >
    (std::numeric_limits<std::int64_t>::max() - stamp.nanosec) /
    kNanosecondsPerSecond)
  {
    return std::nullopt;
  }
  const std::int64_t result =
    seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(stamp.nanosec);
  return result > 0 ? std::optional<std::int64_t>(result) : std::nullopt;
}

bool PoseIsFiniteAndNormalized(const geometry_msgs::msg::Pose & pose)
{
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
    !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
    !std::isfinite(pose.orientation.w))
  {
    return false;
  }
  const double norm_squared =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::abs(norm_squared - 1.0) <= kMaximumQuaternionNormError;
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

}  // namespace

LocalizationOutputGateway::LocalizationOutputGateway(const rclcpp::NodeOptions & options)
: Node("localization_output_gateway", options)
{
  contract_file_ = declare_parameter<std::string>(
    "contract_file", DefaultContractFile(),
    ReadOnlyDescriptor("Version-controlled localization output gateway contract YAML."));
  contract_ = LoadContractConfig(contract_file_);
  state_ = contract_.IsActive() ? State::kActiveStarting : State::kDisabled;
  reason_code_ = contract_.IsActive() ?
    "WAITING_FOR_AUTHORITY_GRAPH" : "OUTPUT_AUTHORIZATION_DENIED";

  if (std::string(get_fully_qualified_name()) != contract_.expected_node_fqn) {
    throw std::runtime_error(
            "gateway node name or namespace remapping is forbidden by the contract");
  }

  const auto require_unremapped_topic = [this](const std::string & topic) {
      if (get_node_topics_interface()->resolve_topic_name(topic, false) != topic) {
        throw std::runtime_error("ROS remapping of gateway contract topics is forbidden");
      }
    };
  require_unremapped_topic(contract_.input.topic);
  require_unremapped_topic(contract_.diagnostics.topic);
  require_unremapped_topic(contract_.external_vision_output.topic);
  require_unremapped_topic(contract_.canonical_odometry.topic);
  if (contract_.IsActive()) {
    require_unremapped_topic(contract_.mavros.state_topic);
    require_unremapped_topic(contract_.mavros.timesync_topic);
  }

  const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(contract_.input.qos.depth))
    .reliable().durability_volatile();
  const auto diagnostics_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().durability_volatile();

  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    contract_.diagnostics.topic, diagnostics_qos);
  selected_subscription_ = create_subscription<SelectedPoseCandidate>(
    contract_.input.topic,
    input_qos,
    std::bind(
      &LocalizationOutputGateway::OnSelectedPose,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  if (contract_.IsActive()) {
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable().transient_local();
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      contract_.mavros.state_topic,
      state_qos,
      std::bind(
        &LocalizationOutputGateway::OnMavrosState,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    timesync_subscription_ = create_subscription<mavros_msgs::msg::TimesyncStatus>(
      contract_.mavros.timesync_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &LocalizationOutputGateway::OnTimesyncStatus,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

  if (std::string(diagnostics_publisher_->get_topic_name()) != contract_.diagnostics.topic ||
    std::string(selected_subscription_->get_topic_name()) != contract_.input.topic)
  {
    throw std::runtime_error("ROS remapping of gateway contract topics is forbidden");
  }
  if (contract_.IsActive() &&
    (std::string(state_subscription_->get_topic_name()) != contract_.mavros.state_topic ||
    std::string(timesync_subscription_->get_topic_name()) !=
    contract_.mavros.timesync_topic))
  {
    throw std::runtime_error("ROS remapping of MAVROS contract topics is forbidden");
  }

  diagnostic_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&LocalizationOutputGateway::PublishDiagnostics, this));

  RCLCPP_WARN(
    get_logger(),
    "Loaded %s gateway contract %s; gateway never arms or changes PX4 mode",
    contract_.profile.c_str(), contract_.gateway_contract_id.c_str());
}

LocalizationOutputGateway::TopicEndpointInfoList
LocalizationOutputGateway::GetPublishersInfoByTopic(const std::string & topic)
{
  return get_publishers_info_by_topic(topic);
}

LocalizationOutputGateway::TopicEndpointInfoList
LocalizationOutputGateway::GetSubscriptionsInfoByTopic(const std::string & topic)
{
  return get_subscriptions_info_by_topic(topic);
}

void LocalizationOutputGateway::OnSelectedPose(
  const SelectedPoseCandidate::ConstSharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  if (!contract_.IsActive()) {
    return;
  }
  if (state_ == State::kLatchedFault) {
    ++rejected_;
    return;
  }
  const bool input_authority_valid = InputAuthorityIsValidLocked(message_info);
  if (!input_authority_valid) {
    ++rejected_;
    return;
  }
  if (!SelectedPoseIsValidLocked(*message)) {
    ++rejected_;
    return;
  }
  if (!MavrosOutputSubscriberIsValidLocked() ||
    !OutputAuthorityIsValidLocked())
  {
    ++rejected_;
    return;
  }
  if (state_ != State::kActiveHealthy && mavros_mode_ == "OFFBOARD") {
    ++authority_violations_;
    LatchLocked("PX4_ENTERED_OFFBOARD_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
    ++rejected_;
    return;
  }
  if (state_ != State::kActiveHealthy && mavros_armed_) {
    ++authority_violations_;
    LatchLocked("PX4_ARMED_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
    ++rejected_;
    return;
  }
  PublishExternalVisionLocked(*message);
}

void LocalizationOutputGateway::OnMavrosState(
  const mavros_msgs::msg::State::ConstSharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!contract_.IsActive() || state_ == State::kLatchedFault) {
    return;
  }
  if (!MessagePublisherIsValidLocked(
      contract_.mavros.state_topic,
      kMavrosStateType,
      contract_.mavros.expected_state_publisher,
      message_info,
      &bound_state_gid_,
      &state_publisher_count_,
      "MAVROS_STATE_PUBLISHER"))
  {
    return;
  }
  mavros_state_received_ = true;
  mavros_connected_ = message->connected;
  mavros_armed_ = message->armed;
  mavros_mode_ = message->mode;
  if (external_vision_publisher_ && state_ != State::kActiveHealthy) {
    if (mavros_mode_ == "OFFBOARD") {
      LatchLocked("PX4_ENTERED_OFFBOARD_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
      return;
    }
    if (mavros_armed_) {
      LatchLocked("PX4_ARMED_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
      return;
    }
  }
  if (!mavros_connected_) {
    if (external_vision_publisher_) {
      LatchLocked("MAVROS_DISCONNECTED_AFTER_OUTPUT_ENABLED");
    } else {
      reason_code_ = "WAITING_FOR_MAVROS_CONNECTION";
    }
  }
}

void LocalizationOutputGateway::OnTimesyncStatus(
  const mavros_msgs::msg::TimesyncStatus::ConstSharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!contract_.IsActive() || state_ == State::kLatchedFault) {
    return;
  }
  if (!MessagePublisherIsValidLocked(
      contract_.mavros.timesync_topic,
      kTimesyncStatusType,
      contract_.mavros.expected_timesync_publisher,
      message_info,
      &bound_timesync_gid_,
      &timesync_publisher_count_,
      "MAVROS_TIMESYNC_PUBLISHER"))
  {
    return;
  }
  if (!std::isfinite(message->round_trip_time_ms) ||
    message->round_trip_time_ms < 0.0F)
  {
    if (!timesync_received_) {
      reason_code_ = "WAITING_FOR_VALID_TIMESYNC_SAMPLE";
    }
    return;
  }
  timesync_received_ = true;
  last_round_trip_time_ms_ = message->round_trip_time_ms;
}

bool LocalizationOutputGateway::MessagePublisherIsValidLocked(
  const std::string & topic,
  const std::string & type,
  const std::string & expected_publisher,
  const rclcpp::MessageInfo & message_info,
  std::optional<PublisherGid> * bound_gid,
  std::size_t * publisher_count,
  const std::string & reason_prefix)
{
  PublisherGid message_gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, message_gid.begin());
  const auto endpoints = GetPublishersInfoByTopic(topic);
  *publisher_count = endpoints.size();
  if (endpoints.empty()) {
    reason_code_ = "WAITING_FOR_" + reason_prefix + "_GRAPH";
    return false;
  }
  if (endpoints.size() != 1U) {
    ++authority_violations_;
    LatchLocked(reason_prefix + "_NOT_UNIQUE");
    return false;
  }
  const auto & endpoint = endpoints.front();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  const bool qos_valid = type == kMavrosStateType ?
    MavrosStateQosIsCompatible(qos) : TimesyncQosIsCompatible(qos);
  if (endpoint.topic_type() != type || !qos_valid) {
    ++authority_violations_;
    LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
    return false;
  }
  const bool startup_unbound =
    state_ == State::kActiveStarting && !bound_gid->has_value();
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (startup_unbound) {
      reason_code_ = "WAITING_FOR_" + reason_prefix + "_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (FullyQualifiedNodeName(endpoint) != expected_publisher) {
    ++authority_violations_;
    LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
    return false;
  }
  if (!GidsEqual(endpoint.endpoint_gid(), message_gid)) {
    if (startup_unbound) {
      reason_code_ = "WAITING_FOR_" + reason_prefix + "_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (bound_gid->has_value() &&
    !GidsEqual(bound_gid->value(), message_gid))
  {
    ++authority_violations_;
    LatchLocked(reason_prefix + "_GID_CHANGED");
    return false;
  }
  *bound_gid = message_gid;
  return true;
}

bool LocalizationOutputGateway::InputAuthorityIsValidLocked(
  const rclcpp::MessageInfo & message_info)
{
  PublisherGid message_gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, message_gid.begin());
  const auto endpoints = GetPublishersInfoByTopic(contract_.input.topic);
  input_publisher_count_ = endpoints.size();
  if (endpoints.empty()) {
    reason_code_ = "WAITING_FOR_SELECTED_PUBLISHER_GRAPH";
    return false;
  }
  if (endpoints.size() != 1U) {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_NOT_UNIQUE");
    return false;
  }
  const auto & endpoint = endpoints.front();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  if (endpoint.topic_type() != kSelectedPoseType ||
    !QosIsCompatible(qos, contract_.input.qos))
  {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  const bool startup_unbound =
    state_ == State::kActiveStarting && !bound_input_gid_.has_value();
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (startup_unbound) {
      reason_code_ = "WAITING_FOR_SELECTED_PUBLISHER_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (FullyQualifiedNodeName(endpoint) != contract_.input.expected_publisher) {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  if (!GidsEqual(endpoint.endpoint_gid(), message_gid)) {
    if (startup_unbound) {
      reason_code_ = "WAITING_FOR_SELECTED_PUBLISHER_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (bound_input_gid_.has_value() &&
    !GidsEqual(bound_input_gid_.value(), message_gid))
  {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_GID_CHANGED");
    return false;
  }
  bound_input_gid_ = message_gid;
  return true;
}

bool LocalizationOutputGateway::MavrosOutputSubscriberIsValidLocked()
{
  const auto endpoints = GetSubscriptionsInfoByTopic(
    contract_.external_vision_output.topic);
  output_subscription_count_ = endpoints.size();
  std::size_t matching_subscribers = 0U;
  bool expected_fqn_seen = false;
  bool unknown_identity_seen = false;
  const bool output_subscriber_was_bound =
    bound_output_subscriber_gid_.has_value() || external_vision_publisher_;
  PublisherGid matching_gid{};
  for (const auto & endpoint : endpoints) {
    if (EndpointIdentityIsUnknown(endpoint)) {
      unknown_identity_seen = true;
      continue;
    }
    if (FullyQualifiedNodeName(endpoint) !=
      contract_.mavros.expected_external_vision_subscriber)
    {
      continue;
    }
    expected_fqn_seen = true;
    const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
    if (endpoint.topic_type() != kExternalVisionType ||
      !QosIsCompatible(qos, contract_.external_vision_output.qos))
    {
      continue;
    }
    ++matching_subscribers;
    std::copy_n(
      endpoint.endpoint_gid().begin(), RMW_GID_STORAGE_SIZE, matching_gid.begin());
  }
  if (matching_subscribers == 0U) {
    if (expected_fqn_seen) {
      ++authority_violations_;
      LatchLocked("MAVROS_EXTERNAL_VISION_SUBSCRIBER_AUTHORITY_MISMATCH");
    } else if (output_subscriber_was_bound) {
      ++authority_violations_;
      LatchLocked("MAVROS_EXTERNAL_VISION_SUBSCRIBER_DISAPPEARED");
    } else if (unknown_identity_seen && state_ == State::kActiveStarting) {
      reason_code_ =
        "WAITING_FOR_MAVROS_EXTERNAL_VISION_SUBSCRIBER_GRAPH_STABILITY";
    } else {
      reason_code_ = "WAITING_FOR_MAVROS_EXTERNAL_VISION_SUBSCRIBER";
    }
    return false;
  }
  if (matching_subscribers != 1U) {
    ++authority_violations_;
    LatchLocked("MAVROS_EXTERNAL_VISION_SUBSCRIBER_NOT_UNIQUE");
    return false;
  }
  if (bound_output_subscriber_gid_.has_value() &&
    !GidsEqual(bound_output_subscriber_gid_.value(), matching_gid))
  {
    ++authority_violations_;
    LatchLocked("MAVROS_EXTERNAL_VISION_SUBSCRIBER_GID_CHANGED");
    return false;
  }
  bound_output_subscriber_gid_ = matching_gid;
  return true;
}

bool LocalizationOutputGateway::OutputAuthorityIsValidLocked()
{
  if (!external_vision_publisher_) {
    return false;
  }
  const auto endpoints = GetPublishersInfoByTopic(
    contract_.external_vision_output.topic);
  output_publisher_count_ = endpoints.size();
  const bool output_graph_was_bound = output_publisher_gid_valid_;
  output_publisher_gid_valid_ = false;
  if (endpoints.empty()) {
    if (state_ == State::kActiveHealthy || output_graph_was_bound) {
      ++authority_violations_;
      LatchLocked("EXTERNAL_VISION_PUBLISHER_DISAPPEARED");
    } else {
      reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH";
    }
    return false;
  }
  if (endpoints.size() != 1U) {
    ++authority_violations_;
    LatchLocked("EXTERNAL_VISION_PUBLISHER_NOT_UNIQUE");
    return false;
  }
  const auto & endpoint = endpoints.front();
  const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
  if (endpoint.topic_type() != kExternalVisionType ||
    !QosIsCompatible(qos, contract_.external_vision_output.qos))
  {
    ++authority_violations_;
    LatchLocked("EXTERNAL_VISION_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  if (EndpointIdentityIsUnknown(endpoint)) {
    if (state_ == State::kActiveStarting && !output_graph_was_bound) {
      reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked("EXTERNAL_VISION_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return false;
  }
  if (FullyQualifiedNodeName(endpoint) != contract_.expected_node_fqn) {
    ++authority_violations_;
    LatchLocked("EXTERNAL_VISION_PUBLISHER_AUTHORITY_MISMATCH");
    return false;
  }
  if (!GidMatchesRmw(endpoint.endpoint_gid(), external_vision_publisher_->get_gid())) {
    if (state_ == State::kActiveStarting && !output_graph_was_bound) {
      reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked("EXTERNAL_VISION_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return false;
  }
  output_publisher_gid_valid_ = true;
  return true;
}

void LocalizationOutputGateway::CreateExternalVisionPublisherLocked()
{
  const auto existing = GetPublishersInfoByTopic(
    contract_.external_vision_output.topic);
  output_publisher_count_ = existing.size();
  if (!existing.empty()) {
    if (existing.size() == 1U && EndpointIdentityIsUnknown(existing.front()) &&
      state_ == State::kActiveStarting)
    {
      reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH_STABILITY";
      return;
    }
    ++authority_violations_;
    LatchLocked("EXTERNAL_VISION_PUBLISHER_ALREADY_EXISTS");
    return;
  }
  const auto output_qos = rclcpp::QoS(
    rclcpp::KeepLast(contract_.external_vision_output.qos.depth))
    .reliable().durability_volatile();
  external_vision_publisher_ = create_publisher<PoseWithCovarianceStamped>(
    contract_.external_vision_output.topic, output_qos);
  if (std::string(external_vision_publisher_->get_topic_name()) !=
    contract_.external_vision_output.topic)
  {
    LatchLocked("EXTERNAL_VISION_TOPIC_REMAPPED");
    return;
  }
  reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH";
}

bool LocalizationOutputGateway::SelectedPoseIsValidLocked(
  const SelectedPoseCandidate & message)
{
  if (message.header.frame_id != contract_.input.parent_frame ||
    message.semantic_child_frame != contract_.input.semantic_child_frame ||
    message.authorization != contract_.input.authorization ||
    message.mode != contract_.input.expected_mode ||
    message.selector_contract_id != contract_.input.expected_selector_contract_id ||
    message.source_contract_id != contract_.input.expected_source_contract_id ||
    message.localization_epoch_id.empty())
  {
    ++contract_violations_;
    LatchLocked("SELECTED_POSE_CONTRACT_MISMATCH");
    return false;
  }
  const auto stamp_ns = StampToNanoseconds(message.header.stamp);
  if (!stamp_ns.has_value() ||
    (last_accepted_stamp_ns_.has_value() &&
    stamp_ns.value() <= last_accepted_stamp_ns_.value()))
  {
    ++timestamp_violations_;
    LatchLocked("SELECTED_POSE_TIMESTAMP_INVALID");
    return false;
  }
  if (!PoseIsFiniteAndNormalized(message.pose)) {
    ++pose_violations_;
    LatchLocked("SELECTED_POSE_NUMERIC_INVALID");
    return false;
  }
  if (bound_epoch_id_.has_value() &&
    bound_epoch_id_.value() != message.localization_epoch_id)
  {
    ++contract_violations_;
    LatchLocked("LOCALIZATION_EPOCH_CHANGED");
    return false;
  }
  bound_epoch_id_ = message.localization_epoch_id;
  last_accepted_stamp_ns_ = stamp_ns;
  return true;
}

void LocalizationOutputGateway::PublishExternalVisionLocked(
  const SelectedPoseCandidate & message)
{
  PoseWithCovarianceStamped output;
  output.header = message.header;
  output.pose.pose = message.pose;
  output.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());
  external_vision_publisher_->publish(output);
  ++published_;
  state_ = State::kActiveHealthy;
  reason_code_ = "EXTERNAL_VISION_PUBLISHED";
}

void LocalizationOutputGateway::LatchLocked(const std::string & reason)
{
  state_ = State::kLatchedFault;
  reason_code_ = reason;
  output_publisher_gid_valid_ = false;
  external_vision_publisher_.reset();
}

void LocalizationOutputGateway::UpdateGraphLocked()
{
  const auto input_endpoints = GetPublishersInfoByTopic(contract_.input.topic);
  input_publisher_count_ = input_endpoints.size();
  if (!contract_.IsActive() || state_ == State::kLatchedFault) {
    output_publisher_count_ = GetPublishersInfoByTopic(
      contract_.external_vision_output.topic).size();
    return;
  }
  if (!external_vision_publisher_) {
    const auto existing_output_publishers = GetPublishersInfoByTopic(
      contract_.external_vision_output.topic);
    output_publisher_count_ = existing_output_publishers.size();
    if (!existing_output_publishers.empty()) {
      if (existing_output_publishers.size() == 1U &&
        EndpointIdentityIsUnknown(existing_output_publishers.front()) &&
        state_ == State::kActiveStarting)
      {
        reason_code_ = "WAITING_FOR_EXTERNAL_VISION_PUBLISHER_GRAPH_STABILITY";
        return;
      }
      ++authority_violations_;
      LatchLocked("EXTERNAL_VISION_PUBLISHER_ALREADY_EXISTS");
      return;
    }
  }

  if (input_endpoints.empty()) {
    if (bound_input_gid_.has_value()) {
      ++authority_violations_;
      LatchLocked("SELECTED_PUBLISHER_DISAPPEARED");
    } else {
      reason_code_ = "WAITING_FOR_SELECTED_PUBLISHER_GRAPH";
    }
    return;
  }
  if (input_endpoints.size() != 1U) {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_NOT_UNIQUE");
    return;
  }
  const auto & input_endpoint = input_endpoints.front();
  const auto & input_qos = input_endpoint.qos_profile().get_rmw_qos_profile();
  if (input_endpoint.topic_type() != kSelectedPoseType ||
    !QosIsCompatible(input_qos, contract_.input.qos))
  {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    return;
  }
  if (EndpointIdentityIsUnknown(input_endpoint)) {
    if (state_ == State::kActiveStarting && !bound_input_gid_.has_value()) {
      reason_code_ = "WAITING_FOR_SELECTED_PUBLISHER_GRAPH_STABILITY";
    } else {
      ++authority_violations_;
      LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    }
    return;
  }
  if (FullyQualifiedNodeName(input_endpoint) != contract_.input.expected_publisher) {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_AUTHORITY_MISMATCH");
    return;
  }
  PublisherGid input_gid{};
  std::copy_n(
    input_endpoint.endpoint_gid().begin(), RMW_GID_STORAGE_SIZE, input_gid.begin());
  if (bound_input_gid_.has_value() &&
    !GidsEqual(bound_input_gid_.value(), input_gid))
  {
    ++authority_violations_;
    LatchLocked("SELECTED_PUBLISHER_GID_CHANGED");
    return;
  }

  const auto validate_mavros_publisher = [this](
    const std::string & topic,
    const std::string & type,
    const std::string & expected_publisher,
    std::optional<PublisherGid> * bound_gid,
    std::size_t * publisher_count,
    const std::string & reason_prefix,
    bool (* qos_validator)(const rmw_qos_profile_t &)) {
      const auto endpoints = GetPublishersInfoByTopic(topic);
      *publisher_count = endpoints.size();
      if (endpoints.empty()) {
        if (bound_gid->has_value()) {
          ++authority_violations_;
          LatchLocked(reason_prefix + "_DISAPPEARED");
        } else {
          reason_code_ = "WAITING_FOR_" + reason_prefix + "_GRAPH";
        }
        return false;
      }
      if (endpoints.size() != 1U) {
        ++authority_violations_;
        LatchLocked(reason_prefix + "_NOT_UNIQUE");
        return false;
      }
      const auto & endpoint = endpoints.front();
      const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
      PublisherGid endpoint_gid{};
      std::copy_n(
        endpoint.endpoint_gid().begin(), RMW_GID_STORAGE_SIZE, endpoint_gid.begin());
      if (endpoint.topic_type() != type || !qos_validator(qos)) {
        ++authority_violations_;
        LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
        return false;
      }
      if (EndpointIdentityIsUnknown(endpoint)) {
        if (state_ == State::kActiveStarting && !bound_gid->has_value()) {
          reason_code_ = "WAITING_FOR_" + reason_prefix + "_GRAPH_STABILITY";
        } else {
          ++authority_violations_;
          LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
        }
        return false;
      }
      if (FullyQualifiedNodeName(endpoint) != expected_publisher) {
        ++authority_violations_;
        LatchLocked(reason_prefix + "_AUTHORITY_MISMATCH");
        return false;
      }
      if (bound_gid->has_value() &&
        !GidsEqual(bound_gid->value(), endpoint_gid))
      {
        ++authority_violations_;
        LatchLocked(reason_prefix + "_GID_CHANGED");
        return false;
      }
      return true;
    };

  if (!validate_mavros_publisher(
      contract_.mavros.state_topic,
      kMavrosStateType,
      contract_.mavros.expected_state_publisher,
      &bound_state_gid_,
      &state_publisher_count_,
      "MAVROS_STATE_PUBLISHER",
      MavrosStateQosIsCompatible))
  {
    return;
  }
  if (!validate_mavros_publisher(
      contract_.mavros.timesync_topic,
      kTimesyncStatusType,
      contract_.mavros.expected_timesync_publisher,
      &bound_timesync_gid_,
      &timesync_publisher_count_,
      "MAVROS_TIMESYNC_PUBLISHER",
      TimesyncQosIsCompatible))
  {
    return;
  }
  if (!MavrosOutputSubscriberIsValidLocked()) {
    return;
  }
  if (!mavros_state_received_) {
    reason_code_ = "WAITING_FOR_MAVROS_STATE";
    return;
  }
  if (!mavros_connected_) {
    if (external_vision_publisher_) {
      LatchLocked("MAVROS_DISCONNECTED_AFTER_OUTPUT_ENABLED");
    } else {
      reason_code_ = "WAITING_FOR_MAVROS_CONNECTION";
    }
    return;
  }
  if (state_ != State::kActiveHealthy && mavros_mode_ == "OFFBOARD") {
    if (external_vision_publisher_) {
      LatchLocked("PX4_ENTERED_OFFBOARD_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
    } else {
      reason_code_ = "WAITING_FOR_NON_OFFBOARD_STARTUP_MODE";
    }
    return;
  }
  if (state_ != State::kActiveHealthy && mavros_armed_) {
    if (external_vision_publisher_) {
      LatchLocked("PX4_ARMED_BEFORE_FIRST_EXTERNAL_VISION_OUTPUT");
    } else {
      reason_code_ = "WAITING_FOR_PX4_DISARMED_STARTUP";
    }
    return;
  }
  if (!timesync_received_) {
    reason_code_ = "WAITING_FOR_VALID_TIMESYNC_SAMPLE";
    return;
  }
  if (!external_vision_publisher_) {
    CreateExternalVisionPublisherLocked();
    return;
  }
  (void)OutputAuthorityIsValidLocked();
}

void LocalizationOutputGateway::PublishDiagnostics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  UpdateGraphLocked();

  DiagnosticStatus status;
  status.name = std::string(get_fully_qualified_name()) + ": localization output gateway";
  status.hardware_id = contract_.gateway_contract_id;
  status.level = state_ == State::kActiveHealthy ?
    DiagnosticStatus::OK : DiagnosticStatus::WARN;
  if (state_ == State::kLatchedFault) {
    status.level = DiagnosticStatus::ERROR;
  }
  status.message = reason_code_;
  const std::string authorization = !contract_.IsActive() ?
    "denied" : (state_ == State::kLatchedFault ? "revoked" : "explicit_pose_only");
  status.values = {
    Value("gateway_contract_id", contract_.gateway_contract_id),
    Value("profile", contract_.profile),
    Value("state", StateToString(state_)),
    Value("authorization", authorization),
    Value("external_vision_output_authorization", authorization),
    Value("reason_code", reason_code_),
    Value("expected_mode", contract_.input.expected_mode),
    Value("expected_selector_contract_id", contract_.input.expected_selector_contract_id),
    Value("expected_source_contract_id", contract_.input.expected_source_contract_id),
    Value(
      "localization_epoch_id",
      bound_epoch_id_.has_value() ? bound_epoch_id_.value() : "not_bound"),
    Value("input_publisher_count", input_publisher_count_),
    Value("mavros_state_publisher_count", state_publisher_count_),
    Value("mavros_timesync_publisher_count", timesync_publisher_count_),
    Value("output_publisher_count", output_publisher_count_),
    Value("mavros_output_subscription_count", output_subscription_count_),
    Value("output_publisher_gid_valid", output_publisher_gid_valid_),
    Value("mavros_state_received", mavros_state_received_),
    Value("mavros_connected", mavros_connected_),
    Value("mavros_armed", mavros_armed_),
    Value("mavros_mode", mavros_mode_),
    Value("timesync_received", timesync_received_),
    Value("last_round_trip_time_ms", last_round_trip_time_ms_),
    Value("received", received_),
    Value("published", published_),
    Value("rejected", rejected_),
    Value("authority_violations", authority_violations_),
    Value("contract_violations", contract_violations_),
    Value("timestamp_violations", timestamp_violations_),
    Value("pose_violations", pose_violations_),
    Value("canonical_odometry_authorization", "denied"),
    Value("velocity_authorization", "denied"),
    Value("offboard_authorization", "denied"),
    Value("arming_authorization", "denied"),
    Value("flight_authorization", "denied"),
  };

  DiagnosticArray output;
  output.header.stamp = now();
  output.status.push_back(std::move(status));
  diagnostics_publisher_->publish(output);
}

std::string LocalizationOutputGateway::StateToString(const State state)
{
  switch (state) {
    case State::kDisabled:
      return "disabled";
    case State::kActiveStarting:
      return "active_starting";
    case State::kActiveHealthy:
      return "active_healthy";
    case State::kLatchedFault:
      return "latched_fault";
  }
  return "invalid";
}

}  // namespace localization_output_gateway
