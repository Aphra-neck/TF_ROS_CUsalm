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
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cuvslam_localization_adapter/adapter_node.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus;
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

class LiveSourceCandidate : public ::testing::Test
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

TEST_F(LiveSourceCandidate, PublishesTransformedPoseOnlyAfterHealthRecovery)
{
  const std::string contract =
    std::string(TEST_FIXTURE_DIR) + "/approved_synthetic_contract.yaml";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  auto adapter = std::make_shared<CuvslamLocalizationAdapter>(options);
  auto visual_slam = std::make_shared<rclcpp::Node>("synthetic_visual_slam");
  auto diagnostic_source =
    std::make_shared<rclcpp::Node>("synthetic_diagnostic_source");
  auto observer = std::make_shared<rclcpp::Node>("source_candidate_observer");

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
  const auto reliable_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto odometry_publisher = visual_slam->create_publisher<Odometry>(
    "/synthetic/odometry", sensor_qos);
  auto status_publisher = visual_slam->create_publisher<VisualSlamStatus>(
    "/synthetic/status", sensor_qos);
  auto diagnostic_publisher = diagnostic_source->create_publisher<DiagnosticArray>(
    "/diagnostics", reliable_qos);

  std::optional<LocalizationSourceCandidate> candidate;
  std::size_t candidate_count = 0U;
  auto candidate_subscription = observer->create_subscription<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose",
    reliable_qos,
    [&candidate, &candidate_count](const LocalizationSourceCandidate::ConstSharedPtr message) {
      candidate = *message;
      ++candidate_count;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(visual_slam);
  executor.add_node(diagnostic_source);
  executor.add_node(observer);

  const auto graph_is_ready = [&adapter]() {
      return adapter->get_publishers_info_by_topic("/synthetic/odometry").size() == 1U &&
             adapter->get_publishers_info_by_topic("/synthetic/status").size() == 1U &&
             adapter->get_publishers_info_by_topic(
        "/localization/candidates/cuvslam/base_pose").size() == 1U;
    };
  const auto graph_deadline = std::chrono::steady_clock::now() + 5s;
  while (!graph_is_ready() && std::chrono::steady_clock::now() < graph_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(graph_is_ready());

  constexpr double kSqrtHalf = 0.7071067811865476;
  const auto publish_evidence = [&]() {
      const auto stamp = visual_slam->get_clock()->now();

      DiagnosticArray diagnostics;
      diagnostics.header.stamp = stamp;
      DiagnosticStatus diagnostic;
      diagnostic.name = "synthetic upstream";
      diagnostic.hardware_id = "synthetic-hardware";
      diagnostic.level = DiagnosticStatus::OK;
      diagnostic.values.resize(1U);
      diagnostic.values.front().key = "state";
      diagnostic.values.front().value = "ready";
      diagnostics.status.push_back(diagnostic);
      diagnostic_publisher->publish(diagnostics);

      VisualSlamStatus status;
      status.header.stamp = stamp;
      status.header.frame_id = "map";
      status.vo_state = 1U;
      status_publisher->publish(status);

      Odometry odometry;
      odometry.header.stamp = stamp;
      odometry.header.frame_id = "odom";
      odometry.child_frame_id = "camera_link";
      odometry.pose.pose.position.x = 0.2;
      odometry.pose.pose.position.y = -0.1;
      odometry.pose.pose.position.z = 0.05;
      odometry.pose.pose.orientation.x = kSqrtHalf;
      odometry.pose.pose.orientation.w = kSqrtHalf;
      odometry_publisher->publish(odometry);

      const auto spin_deadline = std::chrono::steady_clock::now() + 20ms;
      while (std::chrono::steady_clock::now() < spin_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    };

  for (std::size_t index = 0U; index < 3U; ++index) {
    publish_evidence();
  }
  EXPECT_FALSE(candidate.has_value());

  for (std::size_t index = 0U; index < 50U && !candidate.has_value(); ++index) {
    publish_evidence();
  }
  ASSERT_TRUE(candidate.has_value());
  EXPECT_EQ(candidate->header.frame_id, "odom");
  EXPECT_EQ(candidate->semantic_child_frame, "base_link");
  EXPECT_NEAR(candidate->pose.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.position.z, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.orientation.x, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.orientation.y, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.orientation.z, 0.0, 1.0e-12);
  EXPECT_NEAR(candidate->pose.orientation.w, 1.0, 1.0e-12);
  EXPECT_EQ(candidate->source_id, "cuvslam");
  EXPECT_EQ(candidate->source_contract_id, "synthetic_unit_test_only");
  EXPECT_EQ(candidate->authorization, "source_pose_candidate_only");

  auto spoof = std::make_shared<rclcpp::Node>("spoof_candidate_writer");
  auto spoof_publisher = spoof->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", reliable_qos);
  executor.add_node(spoof);
  const auto spoof_deadline = std::chrono::steady_clock::now() + 5s;
  while (adapter->get_publishers_info_by_topic(
      "/localization/candidates/cuvslam/base_pose").size() != 2U &&
    std::chrono::steady_clock::now() < spoof_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(
    adapter->get_publishers_info_by_topic(
      "/localization/candidates/cuvslam/base_pose").size(),
    2U);
  const std::size_t count_before_spoof_evidence = candidate_count;
  publish_evidence();
  EXPECT_EQ(candidate_count, count_before_spoof_evidence);

  const auto publishers = adapter->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(adapter->get_name(), adapter->get_namespace());
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/state/odom"), 0U);
  EXPECT_EQ(publishers.count("/mavros/vision_pose/pose_cov"), 0U);
  EXPECT_EQ(publishers.count("/tf"), 0U);
  EXPECT_EQ(publishers.count("/tf_static"), 0U);

  (void)candidate_subscription;
  (void)spoof_publisher;
}

}  // namespace
}  // namespace cuvslam_localization_adapter
