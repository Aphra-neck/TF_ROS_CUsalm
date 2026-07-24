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

#ifndef LOCALIZATION_SOURCE_SELECTOR__SELECTOR_NODE_HPP_
#define LOCALIZATION_SOURCE_SELECTOR__SELECTOR_NODE_HPP_

#include <rmw/types.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_source_selector/contract_config.hpp"
#include "localization_source_selector/yaw_alignment.hpp"

namespace localization_source_selector
{

bool CandidatePublisherQosIsCompatible(
  const rmw_qos_profile_t & qos,
  const QosContractConfig & expected) noexcept;

class LocalizationSourceSelector : public rclcpp::Node
{
public:
  explicit LocalizationSourceSelector(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationSourceSelector() override = default;

protected:
  using TopicEndpointInfoList = std::vector<rclcpp::TopicEndpointInfo>;

  virtual TopicEndpointInfoList GetPublishersInfoByTopic(const std::string & topic);

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  using PublisherGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

  enum class State
  {
    kStarting,
    kHealthy,
    kStale,
    kRecovering,
    kLatchedFault,
  };

  void OnCandidate(
    const localization_adapter_interfaces::msg::LocalizationSourceCandidate::ConstSharedPtr
    message,
    const rclcpp::MessageInfo & message_info);
  void OnDiagnosticTimer();
  bool ValidateMessagePublisherLocked(const rclcpp::MessageInfo & message_info);
  void UpdateInputAuthorityLocked();
  bool OutputAuthorityValidLocked();
  bool StartupGraphGraceActiveLocked(bool authority_was_validated) const;
  void MarkStartupGraphWaitLocked(const std::string & reason);
  void MarkRecoveringLocked(const std::string & reason);
  void LatchLocked(const std::string & reason);
  void PublishSelectedLocked(
    const localization_adapter_interfaces::msg::LocalizationSourceCandidate & input,
    const geometry_msgs::msg::Pose & aligned_pose);
  void PublishDiagnosticsLocked(const SteadyTime & now);
  static std::string StateToString(State state);

  std::string mode_;
  std::string contract_file_;
  ContractConfig contract_;
  std::string localization_epoch_id_;
  std::string diagnostic_status_name_;
  State state_{State::kStarting};
  std::string reason_code_{"WAITING_FOR_SOURCE"};
  SteadyTime startup_graph_deadline_{};
  std::optional<SteadyTime> last_valid_receive_time_;
  std::optional<std::int64_t> last_accepted_stamp_ns_;
  std::optional<PublisherGid> bound_publisher_gid_;
  std::optional<YawAlignment> alignment_;
  std::optional<geometry_msgs::msg::Pose> last_source_pose_;
  std::size_t recovery_progress_{0U};

  std::size_t input_publisher_count_{0U};
  std::string actual_input_publisher_{"not_discovered"};
  std::string actual_input_type_{"not_discovered"};
  std::string actual_qos_reliability_{"not_discovered"};
  std::string actual_qos_durability_{"not_discovered"};
  std::string actual_qos_history_{"not_discovered"};
  std::size_t actual_qos_depth_{0U};
  std::size_t output_publisher_count_{0U};
  PublisherGid selected_publisher_gid_{};
  bool output_publisher_gid_valid_{false};
  bool output_authority_was_validated_{false};
  std::string last_actual_parent_frame_;
  std::string last_actual_child_frame_;
  std::string last_actual_source_id_;
  std::string last_actual_source_contract_id_;
  std::string last_actual_authorization_;

  std::uint64_t received_{0U};
  std::uint64_t accepted_{0U};
  std::uint64_t published_{0U};
  std::uint64_t rejected_{0U};
  std::uint64_t stale_events_{0U};
  std::uint64_t duplicate_{0U};
  std::uint64_t nonmonotonic_{0U};
  std::uint64_t frame_violation_{0U};
  std::uint64_t contract_violation_{0U};
  std::uint64_t nonfinite_pose_{0U};
  std::uint64_t quaternion_violation_{0U};
  std::uint64_t clock_violation_{0U};
  std::uint64_t pose_reset_{0U};
  std::uint64_t input_authority_violation_{0U};
  std::uint64_t output_authority_violation_{0U};

  mutable std::mutex mutex_;
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  rclcpp::Publisher<
    localization_adapter_interfaces::msg::SelectedPoseCandidate>::SharedPtr
    selected_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Subscription<
    localization_adapter_interfaces::msg::LocalizationSourceCandidate>::SharedPtr
    source_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace localization_source_selector

#endif  // LOCALIZATION_SOURCE_SELECTOR__SELECTOR_NODE_HPP_
