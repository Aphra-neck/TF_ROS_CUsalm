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
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_adapter_interfaces/msg/shadow_pose_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mocap_localization_adapter/adapter_node.hpp"

namespace mocap_localization_adapter
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using localization_adapter_interfaces::msg::ShadowPoseCandidate;
using namespace std::chrono_literals;

std::string DiagnosticValue(
  const DiagnosticStatus & status,
  const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "missing";
}

bool SpinUntil(
  rclcpp::executors::SingleThreadedExecutor * executor,
  const std::function<bool()> & predicate,
  const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

class InputAuthorityTimeout : public ::testing::Test
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

TEST_F(InputAuthorityTimeout, InvalidPublisherLatchesWhenDiscoveryWindowExpires)
{
  auto adapter = std::make_shared<MocapLocalizationAdapter>();
  auto invalid_source = std::make_shared<rclcpp::Node>("invalid_vrpn_identity");
  auto observer = std::make_shared<rclcpp::Node>("input_authority_timeout_observer");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto invalid_publisher = invalid_source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", qos);
  const std::string diagnostic_status_name =
    std::string(adapter->get_fully_qualified_name()) + ": mocap shadow contract";
  std::optional<DiagnosticStatus> latest_status;
  std::size_t candidate_count = 0U;
  auto diagnostics_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics",
    qos,
    [&latest_status, diagnostic_status_name](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name == diagnostic_status_name) {
          latest_status = status;
        }
      }
    });
  auto candidate_subscription = observer->create_subscription<ShadowPoseCandidate>(
    "/localization/shadow/mocap/assumed_base_pose",
    qos,
    [&candidate_count](const ShadowPoseCandidate::ConstSharedPtr) {++candidate_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(invalid_source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&latest_status]() {
        return latest_status.has_value() &&
        DiagnosticValue(latest_status.value(), "health_state") == "starting" &&
        DiagnosticValue(latest_status.value(), "reason_code") ==
        "WAITING_FOR_POSE_PUBLISHER_AUTHORITY";
      },
      3s));
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "bound_publisher_gid"), "unbound");
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "publisher_authority_violation"), "0");
  EXPECT_EQ(candidate_count, 0U);

  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&latest_status]() {
        return latest_status.has_value() &&
        DiagnosticValue(latest_status.value(), "health_state") == "latched_fault" &&
        DiagnosticValue(latest_status.value(), "reason_code") ==
        "POSE_PUBLISHER_IDENTITY_MISMATCH";
      },
      12s));
  EXPECT_GT(
    std::stoull(DiagnosticValue(latest_status.value(), "publisher_authority_violation")),
    0U);
  EXPECT_EQ(candidate_count, 0U);

  executor.remove_node(invalid_source);
  (void)invalid_publisher;
  (void)diagnostics_subscription;
  (void)candidate_subscription;
}

}  // namespace
}  // namespace mocap_localization_adapter
