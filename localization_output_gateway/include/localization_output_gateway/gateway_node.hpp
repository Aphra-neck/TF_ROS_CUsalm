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

#ifndef LOCALIZATION_OUTPUT_GATEWAY__GATEWAY_NODE_HPP_
#define LOCALIZATION_OUTPUT_GATEWAY__GATEWAY_NODE_HPP_

#include <rmw/types.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_output_gateway/contract_config.hpp"

namespace localization_output_gateway
{

class LocalizationOutputGateway : public rclcpp::Node
{
public:
  explicit LocalizationOutputGateway(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationOutputGateway() override = default;

protected:
  using TopicEndpointInfoList = std::vector<rclcpp::TopicEndpointInfo>;

  virtual TopicEndpointInfoList GetPublishersInfoByTopic(const std::string & topic);
  virtual TopicEndpointInfoList GetSubscriptionsInfoByTopic(const std::string & topic);

private:
  using PublisherGid = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

  enum class State
  {
    kDisabled,
    kActiveStarting,
    kActiveHealthy,
    kLatchedFault,
  };

  void OnSelectedPose(
    const localization_adapter_interfaces::msg::SelectedPoseCandidate::ConstSharedPtr
    message,
    const rclcpp::MessageInfo & message_info);
  void OnMavrosState(
    const mavros_msgs::msg::State::ConstSharedPtr message,
    const rclcpp::MessageInfo & message_info);
  void OnTimesyncStatus(
    const mavros_msgs::msg::TimesyncStatus::ConstSharedPtr message,
    const rclcpp::MessageInfo & message_info);
  bool MessagePublisherIsValidLocked(
    const std::string & topic,
    const std::string & type,
    const std::string & expected_publisher,
    const rclcpp::MessageInfo & message_info,
    std::optional<PublisherGid> * bound_gid,
    std::size_t * publisher_count,
    const std::string & reason_prefix);
  bool InputAuthorityIsValidLocked(const rclcpp::MessageInfo & message_info);
  bool MavrosOutputSubscriberIsValidLocked();
  void CreateExternalVisionPublisherLocked();
  bool OutputAuthorityIsValidLocked();
  bool SelectedPoseIsValidLocked(
    const localization_adapter_interfaces::msg::SelectedPoseCandidate & message);
  void PublishExternalVisionLocked(
    const localization_adapter_interfaces::msg::SelectedPoseCandidate & message);
  void LatchLocked(const std::string & reason);
  void UpdateGraphLocked();
  void PublishDiagnostics();
  static std::string StateToString(State state);

  std::string contract_file_;
  ContractConfig contract_;
  State state_{State::kDisabled};
  std::string reason_code_{"OUTPUT_AUTHORIZATION_DENIED"};
  std::optional<PublisherGid> bound_input_gid_;
  std::optional<PublisherGid> bound_state_gid_;
  std::optional<PublisherGid> bound_timesync_gid_;
  std::optional<PublisherGid> bound_output_subscriber_gid_;
  std::optional<std::string> bound_epoch_id_;
  std::optional<std::int64_t> last_accepted_stamp_ns_;
  std::size_t input_publisher_count_{0U};
  std::size_t state_publisher_count_{0U};
  std::size_t timesync_publisher_count_{0U};
  std::size_t output_publisher_count_{0U};
  std::size_t output_subscription_count_{0U};
  bool output_publisher_gid_valid_{false};
  bool mavros_state_received_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  std::string mavros_mode_{"not_observed"};
  bool timesync_received_{false};
  float last_round_trip_time_ms_{0.0F};
  std::uint64_t received_{0U};
  std::uint64_t published_{0U};
  std::uint64_t rejected_{0U};
  std::uint64_t authority_violations_{0U};
  std::uint64_t contract_violations_{0U};
  std::uint64_t timestamp_violations_{0U};
  std::uint64_t pose_violations_{0U};
  mutable std::mutex mutex_;
  rclcpp::Subscription<
    localization_adapter_interfaces::msg::SelectedPoseCandidate>::SharedPtr
    selected_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr
    timesync_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    external_vision_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace localization_output_gateway

#endif  // LOCALIZATION_OUTPUT_GATEWAY__GATEWAY_NODE_HPP_
