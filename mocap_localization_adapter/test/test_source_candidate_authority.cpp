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

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mocap_localization_adapter/adapter_node.hpp"

namespace mocap_localization_adapter
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::PoseStamped;
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using namespace std::chrono_literals;

std::string DiagnosticValue(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "missing";
}

class SourceCandidateAuthority : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(SourceCandidateAuthority, CandidateOnlySpoofLatchesBeforeNextCandidate)
{
  auto adapter = std::make_shared<MocapLocalizationAdapter>();
  auto source = std::make_shared<rclcpp::Node>("vrpn_client_node");
  auto observer = std::make_shared<rclcpp::Node>("candidate_authority_observer");
  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto pose_publisher = source->create_publisher<PoseStamped>(
    "/droneyee207/pose", reliable_qos);

  std::size_t candidate_count = 0U;
  auto candidate_subscription = observer->create_subscription<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose",
    reliable_qos,
    [&candidate_count](const LocalizationSourceCandidate::ConstSharedPtr) {
      ++candidate_count;
    });
  std::optional<DiagnosticStatus> latest_adapter_status;
  const std::string status_name =
    std::string(adapter->get_fully_qualified_name()) + ": mocap shadow contract";
  auto diagnostics_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics",
    reliable_qos,
    [&latest_adapter_status, status_name](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name == status_name) {
          latest_adapter_status = status;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(source);
  executor.add_node(observer);

  const auto graph_is_ready = [&adapter]() {
      return adapter->get_publishers_info_by_topic("/droneyee207/pose").size() == 1U &&
             adapter->get_publishers_info_by_topic(
        "/localization/shadow/mocap/assumed_base_pose").size() == 1U &&
             adapter->get_publishers_info_by_topic(
        "/localization/candidates/mocap/base_pose").size() == 1U;
    };
  const auto graph_deadline = std::chrono::steady_clock::now() + 5s;
  while (!graph_is_ready() && std::chrono::steady_clock::now() < graph_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(graph_is_ready());

  const auto publish_pose = [&]() {
      PoseStamped pose;
      pose.header.stamp = source->get_clock()->now();
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose_publisher->publish(pose);
      const auto spin_deadline = std::chrono::steady_clock::now() + 8ms;
      while (std::chrono::steady_clock::now() < spin_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    };

  for (std::size_t index = 0U; index < 400U && candidate_count == 0U; ++index) {
    publish_pose();
  }
  ASSERT_GT(candidate_count, 0U);

  auto spoof = std::make_shared<rclcpp::Node>("candidate_only_spoof_writer");
  auto spoof_publisher = spoof->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose", reliable_qos);
  executor.add_node(spoof);
  const auto spoof_deadline = std::chrono::steady_clock::now() + 5s;
  while (adapter->get_publishers_info_by_topic(
      "/localization/candidates/mocap/base_pose").size() != 2U &&
    std::chrono::steady_clock::now() < spoof_deadline)
  {
    publish_pose();
  }
  ASSERT_EQ(
    adapter->get_publishers_info_by_topic(
      "/localization/candidates/mocap/base_pose").size(),
    2U);

  const std::size_t count_after_conflict_discovery = candidate_count;
  publish_pose();
  EXPECT_EQ(candidate_count, count_after_conflict_discovery);

  const auto diagnostic_deadline = std::chrono::steady_clock::now() + 500ms;
  while ((!latest_adapter_status.has_value() ||
    DiagnosticValue(latest_adapter_status.value(), "reason_code") !=
    "SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE") &&
    std::chrono::steady_clock::now() < diagnostic_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(latest_adapter_status.has_value());
  EXPECT_EQ(
    DiagnosticValue(latest_adapter_status.value(), "reason_code"),
    "SOURCE_CANDIDATE_PUBLISHER_NOT_UNIQUE");
  EXPECT_EQ(
    DiagnosticValue(
      latest_adapter_status.value(),
      "source_candidate_publisher_authority_violation"),
    "1");

  (void)candidate_subscription;
  (void)diagnostics_subscription;
  (void)spoof_publisher;
}

}  // namespace
}  // namespace mocap_localization_adapter
