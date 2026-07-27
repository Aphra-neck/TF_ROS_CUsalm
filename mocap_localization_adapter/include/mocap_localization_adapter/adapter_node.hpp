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

#ifndef MOCAP_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
#define MOCAP_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_

#include <rmw/types.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/shadow_pose_candidate.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_contracts/gate.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/validation.hpp"
#include "mocap_localization_adapter/adapter_config.hpp"

namespace mocap_localization_adapter
{

// UNKNOWN history is an explicitly reported Humble RMW limitation in shadow mode only.
bool ShadowInputPublisherQosIsCompatible(const rmw_qos_profile_t & qos) noexcept;

class MocapLocalizationAdapter final : public rclcpp::Node
{
public:
  explicit MocapLocalizationAdapter(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using PublisherGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

  void OnPose(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnDiagnosticTimer();

  bool ValidateMessagePublisherLocked(
    const rclcpp::MessageInfo & message_info,
    const SteadyTime & now);
  void UpdatePublisherEvidenceLocked();
  void UpdateOutputPublisherEvidenceLocked();
  void UpdateSourceCandidatePublisherEvidenceLocked();
  bool SourceCandidatePublisherIsAuthoritativeLocked();
  void UpdatePoseRateLocked(std::int64_t stamp_ns);
  bool ClockDomainMatches(std::int64_t stamp_ns, double * residual_sec);
  bool InputIsHealthyLocked(const SteadyTime & now, std::string * reason) const;
  void MaybeAdvanceHealthLocked(const SteadyTime & now);
  void PublishDiagnosticsLocked(const SteadyTime & now);
  std::optional<double> AgeSeconds(
    const std::optional<SteadyTime> & received_at,
    const SteadyTime & now) const;

  AdapterConfig config_;
  std::string diagnostic_status_name_;
  localization_contracts::RuntimeHealthGate health_gate_;
  localization_contracts::StrictStampGuard observed_stamp_guard_;
  SteadyTime started_at_;
  std::optional<SteadyTime> last_callback_received_at_;
  std::optional<SteadyTime> last_valid_pose_received_at_;
  std::optional<std::int64_t> last_health_observed_stamp_ns_;
  std::optional<std::int64_t> last_accepted_stamp_ns_;
  std::optional<PublisherGid> bound_publisher_gid_;
  std::string actual_publisher_;
  std::string last_actual_frame_;
  bool publisher_identity_valid_{false};
  bool publisher_type_valid_{false};
  bool publisher_qos_valid_{false};
  bool output_publisher_identity_valid_{false};
  bool output_publisher_type_valid_{false};
  bool source_candidate_publisher_identity_valid_{false};
  bool source_candidate_publisher_type_valid_{false};
  bool source_candidate_publisher_gid_valid_{false};
  bool last_pose_valid_{false};
  std::size_t publisher_count_{0U};
  std::size_t output_publisher_count_{0U};
  std::size_t source_candidate_publisher_count_{0U};
  std::deque<std::int64_t> recent_pose_stamps_ns_;
  std::optional<double> observed_pose_rate_hz_;
  std::optional<double> minimum_recent_stamp_gap_sec_;
  std::optional<double> maximum_recent_stamp_gap_sec_;
  std::optional<double> submillisecond_stamp_gap_ratio_;
  std::optional<double> last_receive_gap_sec_;
  std::optional<double> last_clock_residual_sec_;
  std::optional<double> last_position_step_m_;
  std::optional<double> last_rotation_step_rad_;
  std::string actual_qos_reliability_;
  std::string actual_qos_durability_;
  std::string actual_qos_history_;
  std::size_t actual_qos_depth_{0U};
  std::optional<localization_contracts::RigidTransform> last_input_pose_;
  std::optional<localization_contracts::RigidTransform> last_shadow_pose_;

  std::uint64_t received_{0U};
  std::uint64_t accepted_{0U};
  std::uint64_t published_{0U};
  std::uint64_t source_candidates_published_{0U};
  std::uint64_t rejected_{0U};
  std::uint64_t zero_or_invalid_stamp_{0U};
  std::uint64_t duplicate_{0U};
  std::uint64_t nonmonotonic_{0U};
  std::uint64_t frame_mismatch_{0U};
  std::uint64_t nonfinite_position_{0U};
  std::uint64_t out_of_bounds_position_{0U};
  std::uint64_t invalid_quaternion_{0U};
  std::uint64_t clock_domain_mismatch_{0U};
  std::uint64_t stamp_gap_violation_{0U};
  std::uint64_t receive_gap_violation_{0U};
  std::uint64_t pose_reset_candidate_{0U};
  std::uint64_t publisher_authority_violation_{0U};
  std::uint64_t output_publisher_authority_violation_{0U};
  std::uint64_t source_candidate_publisher_authority_violation_{0U};

  mutable std::mutex mutex_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  rclcpp::Publisher<
    localization_adapter_interfaces::msg::ShadowPoseCandidate>::SharedPtr
    shadow_pose_publisher_;
  rclcpp::Publisher<
    localization_adapter_interfaces::msg::LocalizationSourceCandidate>::SharedPtr
    source_candidate_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace mocap_localization_adapter

#endif  // MOCAP_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
