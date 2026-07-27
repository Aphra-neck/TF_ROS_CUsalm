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
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
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
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
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

std::int64_t StampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
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

class InputAuthorityLifecycle : public ::testing::Test
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

TEST_F(InputAuthorityLifecycle, DuplicateInputLatchesOnTheNextTrustedFrame)
{
  auto adapter = std::make_shared<MocapLocalizationAdapter>();
  auto observer = std::make_shared<rclcpp::Node>("input_authority_observer");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const std::string diagnostic_status_name =
    std::string(adapter->get_fully_qualified_name()) + ": mocap shadow contract";
  std::optional<DiagnosticStatus> latest_status;
  std::optional<std::chrono::steady_clock::time_point> latest_status_received_at;
  std::size_t diagnostic_count = 0U;
  std::optional<std::int64_t> trigger_stamp_ns;
  bool shadow_candidate_for_trigger = false;
  bool source_candidate_for_trigger = false;
  std::size_t candidate_count = 0U;
  auto diagnostics_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics",
    qos,
    [&latest_status, &latest_status_received_at, &diagnostic_count,
    diagnostic_status_name](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name == diagnostic_status_name) {
          latest_status = status;
          latest_status_received_at = std::chrono::steady_clock::now();
          ++diagnostic_count;
        }
      }
    });
  auto candidate_subscription = observer->create_subscription<ShadowPoseCandidate>(
    "/localization/shadow/mocap/assumed_base_pose",
    qos,
    [&candidate_count, &trigger_stamp_ns,
    &shadow_candidate_for_trigger](const ShadowPoseCandidate::ConstSharedPtr message) {
      ++candidate_count;
      if (trigger_stamp_ns.has_value() &&
      StampToNanoseconds(message->header.stamp) == trigger_stamp_ns.value())
      {
        shadow_candidate_for_trigger = true;
      }
    });
  std::size_t source_candidate_count = 0U;
  auto source_candidate_subscription =
    observer->create_subscription<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose",
    qos,
    [&source_candidate_count, &trigger_stamp_ns,
    &source_candidate_for_trigger](const LocalizationSourceCandidate::ConstSharedPtr message) {
      ++source_candidate_count;
      if (trigger_stamp_ns.has_value() &&
      StampToNanoseconds(message->header.stamp) == trigger_stamp_ns.value())
      {
        source_candidate_for_trigger = true;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&latest_status]() {
        return latest_status.has_value() &&
        DiagnosticValue(latest_status.value(), "health_state") == "starting" &&
        DiagnosticValue(latest_status.value(), "publisher_count") == "0";
      },
      2s));

  auto provisional_source = std::make_shared<rclcpp::Node>("graph_discovery_placeholder");
  auto provisional_publisher =
    provisional_source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", qos);
  executor.add_node(provisional_source);
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&adapter]() {
        return adapter->get_publishers_info_by_topic("/droneyee207/pose").size() == 1U;
      },
      2s));
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&provisional_publisher]() {
        return provisional_publisher->get_subscription_count() == 1U;
      },
      2s));
  for (std::size_t index = 0U; index < 10U; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = provisional_source->get_clock()->now();
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    provisional_publisher->publish(pose);
    executor.spin_some();
    std::this_thread::sleep_for(8ms);
  }
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&latest_status]() {
        return latest_status.has_value() &&
        DiagnosticValue(latest_status.value(), "reason_code") ==
        "WAITING_FOR_POSE_PUBLISHER_AUTHORITY" &&
        DiagnosticValue(latest_status.value(), "received") != "missing" &&
        DiagnosticValue(latest_status.value(), "rejected") != "missing" &&
        std::stoull(DiagnosticValue(latest_status.value(), "received")) > 0U &&
        std::stoull(DiagnosticValue(latest_status.value(), "rejected")) > 0U;
      },
      2s));
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "health_state"), "starting");
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "bound_publisher_gid"), "unbound");
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "publisher_authority_violation"), "0");
  EXPECT_EQ(candidate_count, 0U);

  executor.remove_node(provisional_source);
  provisional_publisher.reset();
  provisional_source.reset();
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&adapter]() {
        return adapter->get_publishers_info_by_topic("/droneyee207/pose").empty();
      },
      5s));

  auto source = std::make_shared<rclcpp::Node>("vrpn_client_node");
  auto pose_publisher = source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", qos);
  executor.add_node(source);
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&adapter]() {
        const auto endpoints = adapter->get_publishers_info_by_topic("/droneyee207/pose");
        return endpoints.size() == 1U && endpoints.front().node_name() == "vrpn_client_node";
      },
      5s));

  constexpr std::int64_t kPosePeriodNanoseconds = 8000000LL;
  const auto first_pose_steady_time = std::chrono::steady_clock::now() + 8ms;
  const std::int64_t first_pose_stamp_ns =
    source->get_clock()->now().nanoseconds() + kPosePeriodNanoseconds;
  std::size_t pose_index = 0U;
  const auto publish_pose = [&]() {
      const auto pose_offset = std::chrono::nanoseconds(
        static_cast<std::int64_t>(pose_index) * kPosePeriodNanoseconds);
      while (std::chrono::steady_clock::now() < first_pose_steady_time + pose_offset) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = rclcpp::Time(
        first_pose_stamp_ns + pose_offset.count(), source->get_clock()->get_clock_type());
      pose.header.frame_id = "world";
      pose.pose.orientation.w = 1.0;
      pose_publisher->publish(pose);
      ++pose_index;
      executor.spin_some();
    };
  const auto input_is_healthy = [&latest_status]() {
      return latest_status.has_value() &&
             DiagnosticValue(latest_status.value(), "health_state") == "healthy";
    };
  while (pose_index < 500U && (candidate_count == 0U || !input_is_healthy())) {
    publish_pose();
  }
  ASSERT_GT(candidate_count, 0U);
  ASSERT_GT(source_candidate_count, 0U);
  ASSERT_TRUE(input_is_healthy());
  EXPECT_NE(DiagnosticValue(latest_status.value(), "bound_publisher_gid"), "unbound");
  EXPECT_EQ(
    DiagnosticValue(latest_status.value(), "source_candidate_publisher_gid_valid"), "1");

  executor.spin_some();
  const std::size_t settled_diagnostic_count = diagnostic_count;
  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&diagnostic_count, settled_diagnostic_count]() {
        return diagnostic_count > settled_diagnostic_count;
      },
      200ms));
  const std::size_t diagnostic_count_before_duplicate = diagnostic_count;
  ASSERT_TRUE(latest_status_received_at.has_value());
  ASSERT_EQ(DiagnosticValue(latest_status.value(), "health_state"), "healthy");
  ASSERT_EQ(DiagnosticValue(latest_status.value(), "publisher_count"), "1");
  const std::uint64_t received_before_trigger = std::stoull(
    DiagnosticValue(latest_status.value(), "received"));
  const std::uint64_t rejected_before_trigger = std::stoull(
    DiagnosticValue(latest_status.value(), "rejected"));
  const std::uint64_t shadow_published_before_trigger = std::stoull(
    DiagnosticValue(latest_status.value(), "published_shadow_candidates"));
  const std::uint64_t source_published_before_trigger = std::stoull(
    DiagnosticValue(latest_status.value(), "published_source_candidates"));
  const std::uint64_t violations_before_duplicate = std::stoull(
    DiagnosticValue(latest_status.value(), "publisher_authority_violation"));

  auto duplicate_source = std::make_shared<rclcpp::Node>("unapproved_vrpn_writer");
  auto duplicate_publisher = duplicate_source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", qos);
  executor.add_node(duplicate_source);
  const auto duplicate_graph_deadline = std::chrono::steady_clock::now() + 60ms;
  while (adapter->get_publishers_info_by_topic("/droneyee207/pose").size() != 2U &&
    std::chrono::steady_clock::now() < duplicate_graph_deadline)
  {
    std::this_thread::yield();
  }
  const auto duplicate_endpoints = adapter->get_publishers_info_by_topic("/droneyee207/pose");
  ASSERT_EQ(duplicate_endpoints.size(), 2U);
  EXPECT_NE(duplicate_endpoints[0].endpoint_gid(), duplicate_endpoints[1].endpoint_gid());
  const auto diagnostic_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - latest_status_received_at.value()).count();
  ASSERT_LT(diagnostic_age_ms, 50);

  geometry_msgs::msg::PoseStamped trusted_pose;
  trusted_pose.header.stamp = source->get_clock()->now();
  trusted_pose.header.frame_id = "world";
  trusted_pose.pose.orientation.w = 1.0;
  trigger_stamp_ns = StampToNanoseconds(trusted_pose.header.stamp);
  pose_publisher->publish(trusted_pose);
  executor.spin_once(20ms);
  EXPECT_EQ(diagnostic_count, diagnostic_count_before_duplicate);

  ASSERT_TRUE(
    SpinUntil(
      &executor,
      [&latest_status]() {
        return latest_status.has_value() &&
        DiagnosticValue(latest_status.value(), "health_state") == "latched_fault" &&
        DiagnosticValue(latest_status.value(), "reason_code") ==
        "POSE_PUBLISHER_NOT_UNIQUE";
      },
      5s));
  EXPECT_EQ(
    std::stoull(DiagnosticValue(latest_status.value(), "publisher_authority_violation")),
    violations_before_duplicate + 1U);
  EXPECT_EQ(
    std::stoull(DiagnosticValue(latest_status.value(), "received")),
    received_before_trigger + 1U);
  EXPECT_EQ(
    std::stoull(DiagnosticValue(latest_status.value(), "rejected")),
    rejected_before_trigger + 1U);
  EXPECT_EQ(
    std::stoull(DiagnosticValue(latest_status.value(), "published_shadow_candidates")),
    shadow_published_before_trigger);
  EXPECT_EQ(
    std::stoull(DiagnosticValue(latest_status.value(), "published_source_candidates")),
    source_published_before_trigger);
  const auto candidate_drain_deadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < candidate_drain_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_FALSE(shadow_candidate_for_trigger);
  EXPECT_FALSE(source_candidate_for_trigger);
  EXPECT_EQ(DiagnosticValue(latest_status.value(), "health_state"), "latched_fault");

  executor.remove_node(duplicate_source);
  executor.remove_node(source);

  (void)diagnostics_subscription;
  (void)candidate_subscription;
  (void)source_candidate_subscription;
  (void)duplicate_publisher;
}

}  // namespace
}  // namespace mocap_localization_adapter
