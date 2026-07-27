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

#ifndef CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
#define CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_

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
#include <unordered_map>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cuvslam_localization_adapter/contract_config.hpp"
#include "localization_contracts/gate.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "localization_contracts/validation.hpp"

namespace cuvslam_localization_adapter
{

class CuvslamLocalizationAdapter final : public rclcpp::Node
{
public:
  explicit CuvslamLocalizationAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using PublisherGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

  struct UpstreamDiagnosticObservation
  {
    std::uint8_t level;
    std::int64_t stamp_ns;
    SteadyTime received_at;
  };

  void OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnVisualSlamStatus(
    const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::SharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnUpstreamDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnDiagnosticTimer();

  bool InputsAreHealthyLocked(const SteadyTime & now, std::string * reason) const;
  bool ValidateMessagePublisherLocked(
    const rclcpp::MessageInfo & message_info,
    const std::string & topic,
    const std::string & expected_publisher,
    const std::string & expected_type,
    std::optional<PublisherGid> * bound_gid,
    std::size_t * publisher_count,
    bool * publisher_identity_valid,
    bool * publisher_type_valid,
    std::uint64_t * authority_violation,
    const std::string & reason_prefix);
  void UpdatePublisherEvidenceLocked();
  void UpdateCandidatePublisherEvidenceLocked();
  bool CandidatePublisherIsAuthoritativeLocked();
  void UpdateOdometryRateLocked(std::int64_t stamp_ns);
  bool ClockDomainMatches(std::int64_t stamp_ns, double * residual_sec);
  void MaybeAdvanceHealthLocked(const SteadyTime & now);
  void PublishSourceCandidateLocked(const builtin_interfaces::msg::Time & stamp);
  void PublishDiagnosticsLocked(const SteadyTime & now);
  std::optional<double> AgeSeconds(
    const std::optional<SteadyTime> & received_at,
    const SteadyTime & now) const;

  std::string mode_;
  std::string contract_file_;
  ContractConfig contract_;
  std::string diagnostic_status_name_;
  localization_contracts::RuntimeHealthGate health_gate_;
  localization_contracts::StrictStampGuard odometry_sequence_guard_;
  localization_contracts::StrictStampGuard stamp_guard_;
  localization_contracts::StrictStampGuard status_sequence_guard_;
  localization_contracts::StrictStampGuard status_stamp_guard_;
  std::unordered_map<std::string, localization_contracts::StrictStampGuard>
  diagnostic_sequence_guards_;

  std::chrono::steady_clock::time_point started_at_;
  std::optional<SteadyTime> last_odometry_received_at_;
  std::optional<SteadyTime> last_odometry_observed_at_;
  std::optional<SteadyTime> last_status_received_at_;
  std::optional<std::int64_t> last_health_observed_stamp_ns_;
  bool last_odometry_valid_{false};
  bool last_status_valid_{false};
  std::uint8_t last_vo_state_{0U};
  std::string last_actual_parent_frame_;
  std::string last_actual_child_frame_;
  std::unordered_map<std::string, UpstreamDiagnosticObservation> upstream_diagnostics_;
  std::deque<std::int64_t> recent_odometry_stamps_ns_;
  std::optional<double> observed_odometry_rate_hz_;
  std::optional<double> maximum_recent_stamp_gap_sec_;
  std::optional<double> last_odometry_receive_gap_sec_;
  std::optional<double> last_odometry_clock_residual_sec_;
  std::optional<double> last_status_clock_residual_sec_;
  std::optional<double> last_diagnostic_clock_residual_sec_;
  std::optional<localization_contracts::RigidTransform> last_input_pose_;
  std::optional<localization_contracts::RigidTransform> last_shadow_pose_;

  std::size_t odometry_publisher_count_{0U};
  std::size_t status_publisher_count_{0U};
  bool odometry_publisher_identity_valid_{false};
  bool status_publisher_identity_valid_{false};
  bool odometry_publisher_type_valid_{false};
  bool status_publisher_type_valid_{false};
  std::optional<PublisherGid> bound_odometry_publisher_gid_;
  std::optional<PublisherGid> bound_status_publisher_gid_;
  std::size_t candidate_publisher_count_{0U};
  bool candidate_publisher_identity_valid_{false};
  bool candidate_publisher_type_valid_{false};
  bool candidate_publisher_gid_valid_{false};

  std::uint64_t received_{0U};
  std::uint64_t accepted_{0U};
  std::uint64_t rejected_{0U};
  std::uint64_t duplicate_{0U};
  std::uint64_t nonmonotonic_{0U};
  std::uint64_t status_received_{0U};
  std::uint64_t status_rejected_{0U};
  std::uint64_t status_duplicate_{0U};
  std::uint64_t status_nonmonotonic_{0U};
  std::uint64_t diagnostic_evidence_received_{0U};
  std::uint64_t diagnostic_evidence_rejected_{0U};
  std::uint64_t diagnostic_duplicate_{0U};
  std::uint64_t diagnostic_nonmonotonic_{0U};
  std::uint64_t clock_domain_mismatch_{0U};
  std::uint64_t odometry_stamp_gap_violation_{0U};
  std::uint64_t odometry_receive_gap_violation_{0U};
  std::uint64_t odometry_rate_violation_{0U};
  std::uint64_t pose_jump_violation_{0U};
  std::uint64_t shadow_processed_{0U};
  std::uint64_t shadow_pose_computed_{0U};
  std::uint64_t source_candidates_published_{0U};
  std::uint64_t odometry_publisher_authority_violation_{0U};
  std::uint64_t status_publisher_authority_violation_{0U};
  std::uint64_t candidate_publisher_authority_violation_{0U};
  static constexpr std::uint64_t kPublished = 0U;

  mutable std::mutex mutex_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<
    localization_adapter_interfaces::msg::LocalizationSourceCandidate>::SharedPtr
    source_candidate_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<
    isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    upstream_diagnostics_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace cuvslam_localization_adapter

#endif  // CUVSLAM_LOCALIZATION_ADAPTER__ADAPTER_NODE_HPP_
