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
#include <thread>

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

using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using localization_adapter_interfaces::msg::ShadowPoseCandidate;
using namespace std::chrono_literals;

class OutputAuthority : public ::testing::Test
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

TEST_F(OutputAuthority, DuplicateAdapterWritersRemainFailClosed)
{
  rclcpp::NodeOptions first_options;
  first_options.arguments({"--ros-args", "--remap", "__ns:=/shadow_a"});
  rclcpp::NodeOptions second_options;
  second_options.arguments({"--ros-args", "--remap", "__ns:=/shadow_b"});
  auto first_adapter = std::make_shared<MocapLocalizationAdapter>(first_options);
  auto second_adapter = std::make_shared<MocapLocalizationAdapter>(second_options);
  auto source = std::make_shared<rclcpp::Node>("vrpn_client_node");
  auto observer = std::make_shared<rclcpp::Node>("duplicate_writer_observer");
  auto pose_publisher = source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  std::size_t candidate_count = 0U;
  auto candidate_subscription = observer->create_subscription<ShadowPoseCandidate>(
    "/localization/shadow/mocap/assumed_base_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [&candidate_count](const ShadowPoseCandidate::ConstSharedPtr) {++candidate_count;});
  std::size_t source_candidate_count = 0U;
  auto source_candidate_subscription =
    observer->create_subscription<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [&source_candidate_count](const LocalizationSourceCandidate::ConstSharedPtr) {
      ++source_candidate_count;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(first_adapter);
  executor.add_node(second_adapter);
  executor.add_node(source);
  executor.add_node(observer);
  for (std::size_t index = 0U; index < 40U; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  for (std::size_t index = 0U; index < 180U; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = source->get_clock()->now();
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose_publisher->publish(pose);
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  for (std::size_t index = 0U; index < 40U; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_EQ(candidate_count, 0U);
  EXPECT_EQ(source_candidate_count, 0U);
  (void)candidate_subscription;
  (void)source_candidate_subscription;
}

}  // namespace
}  // namespace mocap_localization_adapter
