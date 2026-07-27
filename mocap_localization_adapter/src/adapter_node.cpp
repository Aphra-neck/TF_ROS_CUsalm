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

#include "mocap_localization_adapter/adapter_node.hpp"

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
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/shadow_pose_candidate.hpp>
#include <rclcpp/message_info.hpp>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/validation.hpp"
#include "mocap_localization_adapter/pose_transform.hpp"

namespace mocap_localization_adapter
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using localization_contracts::ContractViolation;
using localization_contracts::HealthState;
using localization_contracts::QuaternionXyzw;
using localization_contracts::RigidTransform;
using localization_contracts::StampOrder;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr char kPoseMessageType[] = "geometry_msgs/msg/PoseStamped";
constexpr char kShadowMessageType[] =
  "localization_adapter_interfaces/msg/ShadowPoseCandidate";
constexpr char kSourceCandidateTopic[] = "/localization/candidates/mocap/base_pose";
constexpr char kSourceCandidateMessageType[] =
  "localization_adapter_interfaces/msg/LocalizationSourceCandidate";
constexpr char kSourceCandidateAuthorization[] = "source_pose_candidate_only";
constexpr char kSourceId[] = "mocap";

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
  const std::int64_t stamp_ns =
    seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(stamp.nanosec);
  return stamp_ns > 0 ? std::optional<std::int64_t>(stamp_ns) : std::nullopt;
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
  return std::equal(left.begin(), left.end(), right.begin(), right.end());
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

std::string OptionalDouble(const std::optional<double> & value, const std::string & fallback)
{
  return value.has_value() ? std::to_string(value.value()) : fallback;
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

}  // namespace

bool ShadowInputPublisherQosIsCompatible(const rmw_qos_profile_t & qos) noexcept
{
  // Fast DDS on Humble may report UNKNOWN for an explicit KeepLast publisher.
  const bool history_is_compatible =
    qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ||
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN;
  return qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
         qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
         history_is_compatible;
}

MocapLocalizationAdapter::MocapLocalizationAdapter(const rclcpp::NodeOptions & options)
: Node("mocap_localization_adapter", options),
  config_(DeclareAndValidateConfig(this)),
  diagnostic_status_name_(
    std::string(get_fully_qualified_name()) + ": mocap shadow contract"),
  health_gate_(config_.health.recovery_consecutive_samples),
  started_at_(std::chrono::steady_clock::now())
{
  bool use_sim_time = false;
  (void)get_parameter("use_sim_time", use_sim_time);
  if (use_sim_time) {
    throw std::runtime_error(
            "mocap shadow timestamp contract requires use_sim_time=false and system ROS time");
  }

  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  diagnostics_publisher_ = create_publisher<DiagnosticArray>(
    config_.diagnostics_topic, reliable_qos);
  shadow_pose_publisher_ = create_publisher<
    localization_adapter_interfaces::msg::ShadowPoseCandidate>(
    config_.output_topic, reliable_qos);
  source_candidate_publisher_ = create_publisher<
    localization_adapter_interfaces::msg::LocalizationSourceCandidate>(
    kSourceCandidateTopic, reliable_qos);
  pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.input_topic,
    reliable_qos,
    std::bind(
      &MocapLocalizationAdapter::OnPose,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  if (std::string(diagnostics_publisher_->get_topic_name()) != config_.diagnostics_topic ||
    std::string(shadow_pose_publisher_->get_topic_name()) != config_.output_topic ||
    std::string(source_candidate_publisher_->get_topic_name()) != kSourceCandidateTopic ||
    std::string(pose_subscription_->get_topic_name()) != config_.input_topic)
  {
    throw std::runtime_error("ROS remapping of contract-bound mocap topics is forbidden");
  }

  const auto diagnostic_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(config_.health.diagnostic_period_sec));
  diagnostic_timer_ = create_wall_timer(
    diagnostic_period,
    std::bind(&MocapLocalizationAdapter::OnDiagnosticTimer, this));

  RCLCPP_WARN(
    get_logger(),
    "Loaded shadow-only mocap contract %s; typed candidate is isolated from canonical "
    "odometry, TF, MAVROS and control message types",
    config_.contract_id.c_str());
}

void MocapLocalizationAdapter::OnPose(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr message,
  const rclcpp::MessageInfo & message_info)
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  last_actual_frame_ = message->header.frame_id;

  if (!ValidateMessagePublisherLocked(message_info, now)) {
    ++rejected_;
    last_pose_valid_ = false;
    return;
  }

  bool receive_gap_valid = true;
  if (last_callback_received_at_.has_value()) {
    last_receive_gap_sec_ = std::chrono::duration<double>(
      now - last_callback_received_at_.value()).count();
    if (last_receive_gap_sec_.value() > config_.health.maximum_receive_gap_sec) {
      ++receive_gap_violation_;
      receive_gap_valid = false;
      health_gate_.MarkTransient("POSE_RECEIVE_GAP_EXCEEDED");
    }
  }
  last_callback_received_at_ = now;

  const auto stamp_ns = StampToNanoseconds(message->header.stamp);
  if (!stamp_ns.has_value()) {
    ++rejected_;
    ++zero_or_invalid_stamp_;
    last_pose_valid_ = false;
    health_gate_.MarkTransient("INVALID_TIMESTAMP");
    return;
  }

  const auto previous_observed_stamp = observed_stamp_guard_.LastAcceptedNanoseconds();
  const StampOrder order = observed_stamp_guard_.Classify(stamp_ns.value());
  if (order == StampOrder::kDuplicate) {
    ++rejected_;
    ++duplicate_;
    last_pose_valid_ = false;
    health_gate_.MarkTransient("DUPLICATE_TIMESTAMP");
    return;
  }
  if (order == StampOrder::kNonmonotonic) {
    ++rejected_;
    ++nonmonotonic_;
    last_pose_valid_ = false;
    health_gate_.MarkLatched("NONMONOTONIC_TIMESTAMP");
    return;
  }
  observed_stamp_guard_.Commit(stamp_ns.value());

  if (!receive_gap_valid) {
    ++rejected_;
    last_pose_valid_ = false;
    return;
  }

  if (previous_observed_stamp.has_value()) {
    const double stamp_gap_sec = static_cast<double>(
      stamp_ns.value() - previous_observed_stamp.value()) /
      static_cast<double>(kNanosecondsPerSecond);
    if (stamp_gap_sec > config_.health.maximum_stamp_gap_sec) {
      ++rejected_;
      ++stamp_gap_violation_;
      last_pose_valid_ = false;
      health_gate_.MarkTransient("POSE_STAMP_GAP_EXCEEDED");
      return;
    }
  }

  if (message->header.frame_id != config_.expected_input_frame) {
    ++rejected_;
    ++frame_mismatch_;
    last_pose_valid_ = false;
    health_gate_.MarkLatched("INPUT_FRAME_MISMATCH");
    return;
  }

  const Eigen::Vector3d position(
    message->pose.position.x,
    message->pose.position.y,
    message->pose.position.z);
  if (!position.allFinite()) {
    ++rejected_;
    ++nonfinite_position_;
    last_pose_valid_ = false;
    health_gate_.MarkTransient("NONFINITE_POSITION");
    return;
  }
  if (!std::isfinite(position.norm()) ||
    position.norm() > config_.health.maximum_position_norm_m)
  {
    ++rejected_;
    ++out_of_bounds_position_;
    last_pose_valid_ = false;
    health_gate_.MarkLatched("POSITION_OUTSIDE_SHADOW_SANITY_BOUND");
    return;
  }

  std::optional<RigidTransform> input_pose;
  try {
    input_pose = RigidTransform::Create(
      config_.expected_input_frame,
      config_.semantic_input_child_frame,
      position,
      QuaternionXyzw{
        message->pose.orientation.x,
        message->pose.orientation.y,
        message->pose.orientation.z,
        message->pose.orientation.w});
  } catch (const ContractViolation &) {
    ++rejected_;
    ++invalid_quaternion_;
    last_pose_valid_ = false;
    health_gate_.MarkTransient("INVALID_QUATERNION");
    return;
  }

  double clock_residual_sec = 0.0;
  if (!ClockDomainMatches(stamp_ns.value(), &clock_residual_sec)) {
    ++rejected_;
    ++clock_domain_mismatch_;
    last_pose_valid_ = false;
    last_clock_residual_sec_ = clock_residual_sec;
    health_gate_.MarkTransient("CLOCK_DOMAIN_OR_RESIDUAL_MISMATCH");
    return;
  }
  last_clock_residual_sec_ = clock_residual_sec;

  if (last_input_pose_.has_value()) {
    const auto step = localization_contracts::MeasurePoseStep(
      last_input_pose_.value(), input_pose.value());
    last_position_step_m_ = step.translation_m;
    last_rotation_step_rad_ = step.rotation_rad;
    if (step.translation_m > config_.health.maximum_position_step_m ||
      step.rotation_rad > config_.health.maximum_rotation_step_rad)
    {
      ++rejected_;
      ++pose_reset_candidate_;
      last_pose_valid_ = false;
      health_gate_.MarkLatched("POSE_RESET_OR_JUMP_CANDIDATE");
      return;
    }
  }

  std::optional<RigidTransform> shadow_pose;
  try {
    shadow_pose = MocapPoseToBasePose(
      config_.output_world_from_input_world,
      input_pose.value(),
      config_.base_from_rigid_body);
  } catch (const ContractViolation &) {
    ++rejected_;
    last_pose_valid_ = false;
    health_gate_.MarkLatched("TRANSFORM_CONTRACT_FAILURE");
    return;
  }

  ++accepted_;
  last_pose_valid_ = true;
  last_valid_pose_received_at_ = now;
  last_accepted_stamp_ns_ = stamp_ns.value();
  last_input_pose_ = input_pose;
  last_shadow_pose_ = shadow_pose;
  UpdatePoseRateLocked(stamp_ns.value());
  MaybeAdvanceHealthLocked(now);

  if (health_gate_.State() != HealthState::kHealthy) {
    return;
  }
  if (!SourceCandidatePublisherIsAuthoritativeLocked()) {
    return;
  }

  localization_adapter_interfaces::msg::ShadowPoseCandidate output;
  output.header.stamp = message->header.stamp;
  output.header.frame_id = config_.output_parent_frame;
  output.semantic_child_frame = config_.output_child_frame;
  output.pose.position.x = shadow_pose->Translation().x();
  output.pose.position.y = shadow_pose->Translation().y();
  output.pose.position.z = shadow_pose->Translation().z();
  const auto rotation = shadow_pose->RotationXyzw();
  output.pose.orientation.x = rotation.x;
  output.pose.orientation.y = rotation.y;
  output.pose.orientation.z = rotation.z;
  output.pose.orientation.w = rotation.w;
  output.contract_id = config_.contract_id;
  output.authorization = config_.authorization;
  output.source_topic = config_.input_topic;
  output.source_parent_frame = config_.expected_input_frame;
  output.source_child_frame = config_.semantic_input_child_frame;
  output.source_world_axes = config_.world_axes;
  output.source_body_axes = config_.rigid_body_axes;
  output.geographic_alignment_validated = false;
  output.world_alignment_status = "local_lab_identity_not_geographic";
  output.world_alignment_approved = false;
  output.extrinsic_status = config_.extrinsic_status;
  output.extrinsic_assumption_id = config_.extrinsic_assumption_id;
  output.extrinsic_approved = false;
  output.expected_source_revision = config_.source_revision;
  output.expected_timestamp_semantics = config_.timestamp_semantics;
  output.source_configuration_validated = false;
  output.capture_time_validated = false;
  shadow_pose_publisher_->publish(output);
  ++published_;

  localization_adapter_interfaces::msg::LocalizationSourceCandidate source_candidate;
  source_candidate.header = output.header;
  source_candidate.semantic_child_frame = output.semantic_child_frame;
  source_candidate.pose = output.pose;
  source_candidate.source_id = kSourceId;
  source_candidate.source_contract_id = config_.contract_id;
  source_candidate.authorization = kSourceCandidateAuthorization;
  source_candidate_publisher_->publish(source_candidate);
  ++source_candidates_published_;
}

bool MocapLocalizationAdapter::ValidateMessagePublisherLocked(
  const rclcpp::MessageInfo & message_info,
  const SteadyTime & now)
{
  PublisherGid message_gid{};
  const auto & raw_gid = message_info.get_rmw_message_info().publisher_gid;
  std::copy_n(raw_gid.data, RMW_GID_STORAGE_SIZE, message_gid.begin());
  const auto endpoints = get_publishers_info_by_topic(config_.input_topic);
  publisher_count_ = endpoints.size();
  publisher_identity_valid_ = false;
  publisher_type_valid_ = false;
  publisher_qos_valid_ = false;
  actual_publisher_ = "not_unique_or_missing";
  actual_qos_reliability_ = "not_unique_or_missing";
  actual_qos_durability_ = "not_unique_or_missing";
  actual_qos_history_ = "not_unique_or_missing";
  actual_qos_depth_ = 0U;
  std::size_t matching_gid_count = 0U;
  for (const auto & endpoint : endpoints) {
    if (!GidsEqual(endpoint.endpoint_gid(), message_gid)) {
      continue;
    }
    ++matching_gid_count;
    actual_publisher_ = FullyQualifiedNodeName(endpoint);
    publisher_identity_valid_ = actual_publisher_ == config_.expected_publisher;
    publisher_type_valid_ = endpoint.topic_type() == kPoseMessageType;
    const auto & qos = endpoint.qos_profile().get_rmw_qos_profile();
    actual_qos_reliability_ = ReliabilityToString(qos.reliability);
    actual_qos_durability_ = DurabilityToString(qos.durability);
    actual_qos_history_ = HistoryToString(qos.history);
    actual_qos_depth_ = qos.depth;
    publisher_qos_valid_ = ShadowInputPublisherQosIsCompatible(qos);
  }
  const double startup_age = std::chrono::duration<double>(now - started_at_).count();
  const bool startup_discovery_open =
    !bound_publisher_gid_.has_value() &&
    health_gate_.State() == HealthState::kStarting &&
    startup_age <= config_.health.startup_timeout_sec;

  if (bound_publisher_gid_.has_value()) {
    if (publisher_count_ != 1U) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched(
        publisher_count_ > 1U ?
        "POSE_PUBLISHER_NOT_UNIQUE" : "POSE_PUBLISHER_GID_NOT_DISCOVERED");
      return false;
    }
    if (matching_gid_count != 1U) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_GID_NOT_DISCOVERED");
      return false;
    }
    if (!GidsEqual(message_gid, bound_publisher_gid_.value())) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_GID_CHANGED");
      return false;
    }
    if (!publisher_identity_valid_ || !publisher_type_valid_ || !publisher_qos_valid_) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_AUTHORITY_INVALID");
      return false;
    }
    return true;
  }

  if (matching_gid_count == 0U) {
    if (startup_discovery_open) {
      health_gate_.MarkStarting("WAITING_FOR_POSE_PUBLISHER_GID");
    } else {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_GID_NOT_DISCOVERED");
    }
    return false;
  }
  if (publisher_count_ != 1U || matching_gid_count != 1U ||
    !publisher_identity_valid_ || !publisher_type_valid_ || !publisher_qos_valid_)
  {
    if (startup_discovery_open) {
      health_gate_.MarkStarting("WAITING_FOR_POSE_PUBLISHER_AUTHORITY");
    } else {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_AUTHORITY_INVALID");
    }
    return false;
  }
  if (!bound_publisher_gid_.has_value()) {
    bound_publisher_gid_ = message_gid;
  }
  return true;
}

void MocapLocalizationAdapter::UpdatePublisherEvidenceLocked()
{
  const auto endpoints = get_publishers_info_by_topic(config_.input_topic);
  publisher_count_ = endpoints.size();
  publisher_identity_valid_ =
    endpoints.size() == 1U &&
    FullyQualifiedNodeName(endpoints.front()) == config_.expected_publisher;
  publisher_type_valid_ =
    endpoints.size() == 1U && endpoints.front().topic_type() == kPoseMessageType;
  actual_publisher_ = endpoints.size() == 1U ?
    FullyQualifiedNodeName(endpoints.front()) : "not_unique_or_missing";
  publisher_qos_valid_ = false;
  actual_qos_reliability_ = "not_unique_or_missing";
  actual_qos_durability_ = "not_unique_or_missing";
  actual_qos_history_ = "not_unique_or_missing";
  actual_qos_depth_ = 0U;
  if (endpoints.size() == 1U) {
    const auto & qos = endpoints.front().qos_profile().get_rmw_qos_profile();
    actual_qos_reliability_ = ReliabilityToString(qos.reliability);
    actual_qos_durability_ = DurabilityToString(qos.durability);
    actual_qos_history_ = HistoryToString(qos.history);
    actual_qos_depth_ = qos.depth;
    publisher_qos_valid_ = ShadowInputPublisherQosIsCompatible(qos);
  }

  if (bound_publisher_gid_.has_value() && endpoints.size() == 1U &&
    !GidsEqual(bound_publisher_gid_.value(), endpoints.front().endpoint_gid()))
  {
    ++publisher_authority_violation_;
    health_gate_.MarkLatched("POSE_PUBLISHER_EPOCH_CHANGED");
  }
}

void MocapLocalizationAdapter::UpdateOutputPublisherEvidenceLocked()
{
  const auto endpoints = get_publishers_info_by_topic(config_.output_topic);
  output_publisher_count_ = endpoints.size();
  output_publisher_identity_valid_ =
    endpoints.size() == 1U &&
    FullyQualifiedNodeName(endpoints.front()) == get_fully_qualified_name();
  output_publisher_type_valid_ =
    endpoints.size() == 1U && endpoints.front().topic_type() == kShadowMessageType;
}

void MocapLocalizationAdapter::UpdateSourceCandidatePublisherEvidenceLocked()
{
  const auto endpoints = get_publishers_info_by_topic(kSourceCandidateTopic);
  source_candidate_publisher_count_ = endpoints.size();
  source_candidate_publisher_identity_valid_ =
    endpoints.size() == 1U &&
    FullyQualifiedNodeName(endpoints.front()) == get_fully_qualified_name();
  source_candidate_publisher_type_valid_ =
    endpoints.size() == 1U &&
    endpoints.front().topic_type() == kSourceCandidateMessageType;
  source_candidate_publisher_gid_valid_ =
    endpoints.size() == 1U &&
    GidMatchesRmw(endpoints.front().endpoint_gid(), source_candidate_publisher_->get_gid());
}

bool MocapLocalizationAdapter::SourceCandidatePublisherIsAuthoritativeLocked()
{
  UpdateSourceCandidatePublisherEvidenceLocked();
  if (source_candidate_publisher_count_ == 1U &&
    source_candidate_publisher_identity_valid_ &&
    source_candidate_publisher_type_valid_ &&
    source_candidate_publisher_gid_valid_)
  {
    return true;
  }

  ++source_candidate_publisher_authority_violation_;
  health_gate_.MarkLatched(
    source_candidate_publisher_count_ > 1U ?
    "SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE" :
    "SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID");
  return false;
}

void MocapLocalizationAdapter::UpdatePoseRateLocked(const std::int64_t stamp_ns)
{
  recent_pose_stamps_ns_.push_back(stamp_ns);
  while (recent_pose_stamps_ns_.size() > config_.health.pose_rate_window_samples) {
    recent_pose_stamps_ns_.pop_front();
  }
  if (recent_pose_stamps_ns_.size() < config_.health.pose_rate_window_samples) {
    observed_pose_rate_hz_.reset();
    minimum_recent_stamp_gap_sec_.reset();
    maximum_recent_stamp_gap_sec_.reset();
    submillisecond_stamp_gap_ratio_.reset();
    return;
  }
  const std::vector<std::int64_t> stamps(
    recent_pose_stamps_ns_.begin(), recent_pose_stamps_ns_.end());
  const auto metrics = localization_contracts::ComputeStampWindowMetrics(stamps);
  if (!metrics.has_value()) {
    observed_pose_rate_hz_.reset();
    minimum_recent_stamp_gap_sec_.reset();
    maximum_recent_stamp_gap_sec_.reset();
    submillisecond_stamp_gap_ratio_.reset();
    health_gate_.MarkLatched("POSE_RATE_WINDOW_INVALID");
    return;
  }
  observed_pose_rate_hz_ = metrics->rate_hz;
  maximum_recent_stamp_gap_sec_ = metrics->maximum_gap_sec;
  double minimum_gap_sec = std::numeric_limits<double>::infinity();
  std::size_t submillisecond_gap_count = 0U;
  for (std::size_t index = 1U; index < stamps.size(); ++index) {
    const double gap_sec = static_cast<double>(stamps[index] - stamps[index - 1U]) /
      static_cast<double>(kNanosecondsPerSecond);
    minimum_gap_sec = std::min(minimum_gap_sec, gap_sec);
    if (gap_sec < 0.001) {
      ++submillisecond_gap_count;
    }
  }
  minimum_recent_stamp_gap_sec_ = minimum_gap_sec;
  submillisecond_stamp_gap_ratio_ = static_cast<double>(submillisecond_gap_count) /
    static_cast<double>(stamps.size() - 1U);
}

bool MocapLocalizationAdapter::ClockDomainMatches(
  const std::int64_t stamp_ns,
  double * residual_sec)
{
  const std::int64_t now_ns = system_clock_.now().nanoseconds();
  if (now_ns <= 0 || stamp_ns <= 0) {
    *residual_sec = std::numeric_limits<double>::infinity();
    return false;
  }
  *residual_sec = localization_contracts::AbsoluteStampDifferenceSeconds(now_ns, stamp_ns);
  return std::isfinite(*residual_sec) &&
         *residual_sec <= config_.health.maximum_clock_residual_sec;
}

bool MocapLocalizationAdapter::InputIsHealthyLocked(
  const SteadyTime & now,
  std::string * reason) const
{
  bool use_sim_time = false;
  (void)get_parameter("use_sim_time", use_sim_time);
  if (use_sim_time) {
    *reason = "SIM_TIME_ENABLED";
    return false;
  }
  if (!bound_publisher_gid_.has_value()) {
    const bool publisher_authority_incomplete =
      publisher_count_ != 1U || !publisher_identity_valid_ ||
      !publisher_type_valid_ || !publisher_qos_valid_;
    if (publisher_count_ == 0U) {
      *reason = "WAITING_FOR_POSE_PUBLISHER";
    } else if (publisher_authority_incomplete) {
      *reason = "WAITING_FOR_POSE_PUBLISHER_AUTHORITY";
    } else {
      *reason = "WAITING_FOR_POSE_PUBLISHER_GID";
    }
    return false;
  }
  if (publisher_count_ == 0U) {
    *reason = "WAITING_FOR_POSE_PUBLISHER";
    return false;
  }
  if (publisher_count_ != 1U || !publisher_identity_valid_ || !publisher_type_valid_ ||
    !publisher_qos_valid_)
  {
    *reason = "POSE_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (output_publisher_count_ == 0U) {
    *reason = "WAITING_FOR_SHADOW_OUTPUT_PUBLISHER";
    return false;
  }
  if (output_publisher_count_ != 1U || !output_publisher_identity_valid_ ||
    !output_publisher_type_valid_)
  {
    *reason = "SHADOW_OUTPUT_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (source_candidate_publisher_count_ == 0U) {
    *reason = "WAITING_FOR_SOURCE_CANDIDATE_PUBLISHER";
    return false;
  }
  if (source_candidate_publisher_count_ != 1U ||
    !source_candidate_publisher_identity_valid_ ||
    !source_candidate_publisher_type_valid_ ||
    !source_candidate_publisher_gid_valid_)
  {
    *reason = "SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID";
    return false;
  }
  if (!last_valid_pose_received_at_.has_value()) {
    *reason = "WAITING_FOR_VALID_POSE";
    return false;
  }
  if (!last_pose_valid_) {
    *reason = "LATEST_POSE_INVALID";
    return false;
  }
  if (!observed_pose_rate_hz_.has_value() || !maximum_recent_stamp_gap_sec_.has_value()) {
    *reason = "WAITING_FOR_POSE_RATE_WINDOW";
    return false;
  }
  if (observed_pose_rate_hz_.value() < config_.health.minimum_pose_rate_hz) {
    *reason = "POSE_RATE_TOO_LOW";
    return false;
  }
  if (maximum_recent_stamp_gap_sec_.value() > config_.health.maximum_stamp_gap_sec) {
    *reason = "POSE_STAMP_GAP_EXCEEDED";
    return false;
  }
  if (AgeSeconds(last_valid_pose_received_at_, now).value() > config_.health.stale_after_sec) {
    *reason = "POSE_STALE";
    return false;
  }
  *reason = "INPUT_HEALTHY";
  return true;
}

void MocapLocalizationAdapter::MaybeAdvanceHealthLocked(const SteadyTime & now)
{
  if (health_gate_.State() == HealthState::kLatchedFault) {
    return;
  }
  std::string reason;
  if (!InputIsHealthyLocked(now, &reason)) {
    if (health_gate_.State() == HealthState::kStarting) {
      health_gate_.MarkStarting(reason);
    } else {
      health_gate_.MarkTransient(reason);
    }
    return;
  }
  if (!last_accepted_stamp_ns_.has_value() ||
    last_health_observed_stamp_ns_ == last_accepted_stamp_ns_)
  {
    return;
  }
  health_gate_.ObserveHealthySample();
  last_health_observed_stamp_ns_ = last_accepted_stamp_ns_;
}

std::optional<double> MocapLocalizationAdapter::AgeSeconds(
  const std::optional<SteadyTime> & received_at,
  const SteadyTime & now) const
{
  if (!received_at.has_value()) {
    return std::nullopt;
  }
  return std::chrono::duration<double>(now - received_at.value()).count();
}

void MocapLocalizationAdapter::OnDiagnosticTimer()
{
  const SteadyTime now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  UpdatePublisherEvidenceLocked();
  UpdateOutputPublisherEvidenceLocked();
  UpdateSourceCandidatePublisherEvidenceLocked();
  const bool shadow_output_publisher_authority_invalid =
    output_publisher_count_ == 1U &&
    (!output_publisher_identity_valid_ || !output_publisher_type_valid_);
  const bool source_candidate_publisher_authority_invalid =
    source_candidate_publisher_count_ == 1U &&
    (!source_candidate_publisher_identity_valid_ ||
    !source_candidate_publisher_type_valid_ ||
    !source_candidate_publisher_gid_valid_);
  const bool input_publisher_bound = bound_publisher_gid_.has_value();
  const double startup_age = std::chrono::duration<double>(now - started_at_).count();
  const bool startup_discovery_open =
    !input_publisher_bound && health_gate_.State() == HealthState::kStarting &&
    startup_age <= config_.health.startup_timeout_sec;
  const bool input_authority_enforced = input_publisher_bound || !startup_discovery_open;
  const bool input_publisher_not_unique =
    input_authority_enforced && publisher_count_ > 1U;
  const bool input_publisher_identity_mismatch =
    input_authority_enforced && publisher_count_ == 1U && !publisher_identity_valid_;
  const bool input_publisher_type_mismatch =
    input_authority_enforced && publisher_count_ == 1U && !publisher_type_valid_;
  const bool input_publisher_qos_mismatch =
    input_authority_enforced && publisher_count_ == 1U && !publisher_qos_valid_;

  if (health_gate_.State() != HealthState::kLatchedFault) {
    if (input_publisher_not_unique) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_NOT_UNIQUE");
    } else if (input_publisher_identity_mismatch) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_IDENTITY_MISMATCH");
    } else if (input_publisher_type_mismatch) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_TYPE_MISMATCH");
    } else if (input_publisher_qos_mismatch) {
      ++publisher_authority_violation_;
      health_gate_.MarkLatched("POSE_PUBLISHER_QOS_MISMATCH");
    } else if (output_publisher_count_ > 1U) {
      ++output_publisher_authority_violation_;
      health_gate_.MarkLatched("SHADOW_OUTPUT_PUBLISHER_NOT_UNIQUE");
    } else if (shadow_output_publisher_authority_invalid) {
      ++output_publisher_authority_violation_;
      health_gate_.MarkLatched("SHADOW_OUTPUT_PUBLISHER_AUTHORITY_INVALID");
    } else if (source_candidate_publisher_count_ > 1U) {
      ++source_candidate_publisher_authority_violation_;
      health_gate_.MarkLatched("SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE");
    } else if (source_candidate_publisher_authority_invalid) {
      ++source_candidate_publisher_authority_violation_;
      health_gate_.MarkLatched("SOURCE_CANDIDATE_PUBLISHER_AUTHORITY_INVALID");
    }
  }

  if (health_gate_.State() != HealthState::kLatchedFault) {
    std::string reason;
    if (!InputIsHealthyLocked(now, &reason)) {
      const bool waiting = reason.rfind("WAITING_FOR_", 0U) == 0U;
      if (waiting && startup_age <= config_.health.startup_timeout_sec &&
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

void MocapLocalizationAdapter::PublishDiagnosticsLocked(const SteadyTime & now)
{
  DiagnosticStatus status;
  status.name = diagnostic_status_name_;
  status.hardware_id = "droneyee207";
  switch (health_gate_.State()) {
    case HealthState::kStarting:
      status.level = DiagnosticStatus::STALE;
      status.message = "waiting for complete VRPN pose evidence";
      break;
    case HealthState::kTransientFault:
      status.level = DiagnosticStatus::ERROR;
      status.message = "transient mocap shadow contract fault";
      break;
    case HealthState::kRecovering:
      status.level = DiagnosticStatus::WARN;
      status.message = "mocap pose input is recovering";
      break;
    case HealthState::kLatchedFault:
      status.level = DiagnosticStatus::ERROR;
      status.message = "latched mocap contract fault; restart required";
      break;
    case HealthState::kHealthy:
      status.level = DiagnosticStatus::WARN;
      status.message = "mocap input is healthy; output remains a shadow-only pose candidate";
      break;
  }

  status.values = {
    Value("contract_id", config_.contract_id),
    Value("expected_source_revision", config_.source_revision),
    Value("source_configuration_validated", false),
    Value("mode", config_.mode),
    Value("authorization", config_.authorization),
    Value("health_state", localization_contracts::ToString(health_gate_.State())),
    Value("reason_code", health_gate_.Reason()),
    Value("latched", health_gate_.State() == HealthState::kLatchedFault),
    Value("input_topic", config_.input_topic),
    Value("input_type", kPoseMessageType),
    Value("expected_publisher", config_.expected_publisher),
    Value("actual_publisher", actual_publisher_),
    Value("publisher_count", publisher_count_),
    Value("publisher_identity_valid", publisher_identity_valid_),
    Value("publisher_type_valid", publisher_type_valid_),
    Value("publisher_qos_valid", publisher_qos_valid_),
    Value("output_publisher_count", output_publisher_count_),
    Value("output_publisher_identity_valid", output_publisher_identity_valid_),
    Value("output_publisher_type_valid", output_publisher_type_valid_),
    Value("source_candidate_topic", kSourceCandidateTopic),
    Value("source_candidate_type", kSourceCandidateMessageType),
    Value("source_candidate_authorization", kSourceCandidateAuthorization),
    Value("source_candidate_publisher_count", source_candidate_publisher_count_),
    Value(
      "source_candidate_publisher_identity_valid",
      source_candidate_publisher_identity_valid_),
    Value(
      "source_candidate_publisher_type_valid",
      source_candidate_publisher_type_valid_),
    Value(
      "source_candidate_publisher_gid_valid",
      source_candidate_publisher_gid_valid_),
    Value(
      "bound_publisher_gid",
      bound_publisher_gid_.has_value() ?
      GidToString(bound_publisher_gid_.value()) : "unbound"),
    Value("expected_input_frame", config_.expected_input_frame),
    Value("last_actual_frame", last_actual_frame_),
    Value("semantic_input_child_frame", config_.semantic_input_child_frame),
    Value("world_axes", config_.world_axes),
    Value("rigid_body_axes", config_.rigid_body_axes),
    Value("expected_timestamp_semantics", config_.timestamp_semantics),
    Value("capture_time_validated", false),
    Value(
      "expected_input_qos",
      "reliable,volatile,history_keep_last_or_rmw_unknown,depth_any"),
    Value("actual_qos_reliability", actual_qos_reliability_),
    Value("actual_qos_durability", actual_qos_durability_),
    Value("actual_qos_history", actual_qos_history_),
    Value("actual_qos_depth", actual_qos_depth_),
    Value("qos_history_keep_last_confirmed", actual_qos_history_ == "keep_last"),
    Value(
      "qos_history_unknown_shadow_fallback",
      publisher_qos_valid_ && actual_qos_history_ == "unknown"),
    Value("output_topic", config_.output_topic),
    Value("output_type", kShadowMessageType),
    Value("output_parent_frame", config_.output_parent_frame),
    Value("output_child_frame_semantic", config_.output_child_frame),
    Value("extrinsic_status", config_.extrinsic_status),
    Value("extrinsic_assumption_id", config_.extrinsic_assumption_id),
    Value("world_alignment_status", "local_lab_identity_not_geographic"),
    Value("world_alignment_approved", false),
    Value("standard_odometry_authorized", false),
    Value("tf_authorized", false),
    Value("mavros_authorized", false),
    Value("vrpn_twist_consumed", false),
    Value("vrpn_accel_consumed", false),
    Value("received", received_),
    Value("accepted", accepted_),
    Value("published_shadow_candidates", published_),
    Value("published_source_candidates", source_candidates_published_),
    Value("rejected", rejected_),
    Value("zero_or_invalid_stamp", zero_or_invalid_stamp_),
    Value("duplicate", duplicate_),
    Value("nonmonotonic", nonmonotonic_),
    Value("frame_mismatch", frame_mismatch_),
    Value("nonfinite_position", nonfinite_position_),
    Value("out_of_bounds_position", out_of_bounds_position_),
    Value("invalid_quaternion", invalid_quaternion_),
    Value("clock_domain_mismatch", clock_domain_mismatch_),
    Value("stamp_gap_violation", stamp_gap_violation_),
    Value("receive_gap_violation", receive_gap_violation_),
    Value("pose_reset_candidate", pose_reset_candidate_),
    Value("publisher_authority_violation", publisher_authority_violation_),
    Value(
      "output_publisher_authority_violation",
      output_publisher_authority_violation_),
    Value(
      "source_candidate_publisher_authority_violation",
      source_candidate_publisher_authority_violation_),
    Value("recovery_progress", health_gate_.RecoveryProgress()),
    Value("recovery_required", health_gate_.RecoveryRequired()),
    Value(
      "last_accepted_stamp_ns",
      last_accepted_stamp_ns_.has_value() ?
      std::to_string(last_accepted_stamp_ns_.value()) : "not_received"),
    Value(
      "last_valid_receive_age_sec",
      OptionalDouble(AgeSeconds(last_valid_pose_received_at_, now), "not_received")),
    Value(
      "observed_pose_rate_hz",
      OptionalDouble(observed_pose_rate_hz_, "warming_up")),
    Value(
      "minimum_recent_stamp_gap_sec",
      OptionalDouble(minimum_recent_stamp_gap_sec_, "warming_up")),
    Value(
      "maximum_recent_stamp_gap_sec",
      OptionalDouble(maximum_recent_stamp_gap_sec_, "warming_up")),
    Value(
      "submillisecond_stamp_gap_ratio",
      OptionalDouble(submillisecond_stamp_gap_ratio_, "warming_up")),
    Value(
      "last_receive_gap_sec",
      OptionalDouble(last_receive_gap_sec_, "not_received")),
    Value(
      "last_clock_residual_sec",
      OptionalDouble(last_clock_residual_sec_, "not_received")),
    Value(
      "last_position_step_m",
      OptionalDouble(last_position_step_m_, "not_received")),
    Value(
      "last_rotation_step_rad",
      OptionalDouble(last_rotation_step_rad_, "not_received")),
  };

  if (last_shadow_pose_.has_value()) {
    status.values.push_back(Value("shadow_base_x", last_shadow_pose_->Translation().x()));
    status.values.push_back(Value("shadow_base_y", last_shadow_pose_->Translation().y()));
    status.values.push_back(Value("shadow_base_z", last_shadow_pose_->Translation().z()));
  }

  DiagnosticArray output;
  output.header.stamp = get_clock()->now();
  output.status.push_back(std::move(status));
  diagnostics_publisher_->publish(output);
}

}  // namespace mocap_localization_adapter
