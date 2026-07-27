// Copyright 2026 u5-4
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
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
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_source_selector/selector_node.hpp"

namespace localization_source_selector
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;
using namespace std::chrono_literals;

class SelectorPoseSemantics : public ::testing::Test
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

struct ModeFixture
{
  std::string mode;
  std::string contract_file;
  std::string source_node;
  std::string input_topic;
  std::string input_frame;
  std::string source_id;
  std::string source_contract_id;
};

struct SelectionResult
{
  std::vector<SelectedPoseCandidate> outputs;
  std::optional<DiagnosticStatus> diagnostics;
};

ModeFixture MocapFixture()
{
  return ModeFixture{
    "mocap_primary",
    std::string(TEST_CONFIG_DIR) + "/mocap_primary.contract.yaml",
    "mocap_localization_adapter",
    "/localization/candidates/mocap/base_pose",
    "mocap_world",
    "mocap",
    "droneyee207_mocap_shadow_20260722_v2"};
}

ModeFixture CuvslamFixture()
{
  return ModeFixture{
    "cuvslam_primary",
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml",
    "cuvslam_localization_adapter",
    "/localization/candidates/cuvslam/base_pose",
    "odom",
    "cuvslam",
    "d435i_fcu_cuvslam_shadow_20260723_v2"};
}

rclcpp::NodeOptions Options(const ModeFixture & fixture)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", fixture.mode),
        rclcpp::Parameter("contract_file", fixture.contract_file),
      });
  return options;
}

LocalizationSourceCandidate Candidate(
  rclcpp::Node & source,
  const ModeFixture & fixture,
  const double x = 2.0,
  const double y = 4.0,
  const double z = 0.5,
  const double yaw = 0.6)
{
  LocalizationSourceCandidate message;
  message.header.stamp = source.get_clock()->now();
  message.header.frame_id = fixture.input_frame;
  message.semantic_child_frame = "base_link";
  message.pose.position.x = x;
  message.pose.position.y = y;
  message.pose.position.z = z;
  message.pose.orientation.z = std::sin(0.5 * yaw);
  message.pose.orientation.w = std::cos(0.5 * yaw);
  message.source_id = fixture.source_id;
  message.source_contract_id = fixture.source_contract_id;
  message.authorization = "source_pose_candidate_only";
  return message;
}

bool SpinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & condition,
  const std::function<void()> & tick,
  const std::chrono::milliseconds timeout = 5000ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    tick();
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return condition();
}

std::string DiagnosticValue(
  const std::optional<DiagnosticStatus> & status,
  const std::string & key)
{
  if (!status.has_value()) {
    return "missing_status";
  }
  for (const auto & value : status->values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "missing_key";
}

SelectionResult SelectPose(const ModeFixture & fixture)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(Options(fixture));
  auto source = std::make_shared<rclcpp::Node>(fixture.source_node);
  auto observer = std::make_shared<rclcpp::Node>(fixture.mode + "_pose_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    fixture.input_topic, qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics", qos,
    [&latest_status](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name.find("localization source selector") != std::string::npos) {
          latest_status = status;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  const bool graph_ready = SpinUntil(
    executor,
    [&publisher]() {return publisher->get_subscription_count() == 1U;},
    []() {});
  EXPECT_TRUE(graph_ready);
  const bool selected = SpinUntil(
    executor,
    [&outputs, &latest_status]() {
      return !outputs.empty() &&
      DiagnosticValue(latest_status, "alignment_locked") == "1";
    },
    [&publisher, &source, &fixture]() {
      publisher->publish(Candidate(*source, fixture));
    });
  EXPECT_TRUE(selected);
  std::this_thread::sleep_for(1ms);
  const auto followup = Candidate(*source, fixture, 2.25, 3.75, 0.8, 0.9);
  const auto followup_stamp = followup.header.stamp;
  publisher->publish(followup);
  const bool followup_selected = SpinUntil(
    executor,
    [&outputs, followup_stamp]() {
      return !outputs.empty() &&
      outputs.back().header.stamp.sec == followup_stamp.sec &&
      outputs.back().header.stamp.nanosec == followup_stamp.nanosec;
    },
    []() {});
  EXPECT_TRUE(followup_selected);
  (void)diagnostic_subscription;
  (void)subscription;
  return SelectionResult{outputs, latest_status};
}

TEST_F(SelectorPoseSemantics, MocapPreservesGlobalPositionAndYaw)
{
  constexpr double kYaw = 0.6;
  constexpr double kFollowupYaw = 0.9;
  const auto result = SelectPose(MocapFixture());
  ASSERT_FALSE(result.outputs.empty());
  const auto & initial = result.outputs.front();
  const auto & followup = result.outputs.back();

  EXPECT_EQ(initial.header.frame_id, "map");
  EXPECT_EQ(initial.mode, "mocap_primary");
  EXPECT_EQ(
    initial.selector_contract_id,
    "yopo_mocap_primary_selector_20260727_v2");
  EXPECT_NEAR(initial.pose.position.x, 2.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.position.y, 4.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.position.z, 0.5, 1.0e-12);
  EXPECT_NEAR(initial.pose.orientation.z, std::sin(0.5 * kYaw), 1.0e-12);
  EXPECT_NEAR(initial.pose.orientation.w, std::cos(0.5 * kYaw), 1.0e-12);
  EXPECT_NEAR(followup.pose.position.x, 2.25, 1.0e-12);
  EXPECT_NEAR(followup.pose.position.y, 3.75, 1.0e-12);
  EXPECT_NEAR(followup.pose.position.z, 0.8, 1.0e-12);
  EXPECT_NEAR(
    followup.pose.orientation.z, std::sin(0.5 * kFollowupYaw), 1.0e-12);
  EXPECT_NEAR(
    followup.pose.orientation.w, std::cos(0.5 * kFollowupYaw), 1.0e-12);
  EXPECT_EQ(
    DiagnosticValue(result.diagnostics, "pose_reference_semantics"),
    "global_mocap_world");
  EXPECT_EQ(DiagnosticValue(result.diagnostics, "alignment_locked"), "1");
  EXPECT_NEAR(
    std::stod(DiagnosticValue(result.diagnostics, "alignment_yaw_map_from_source_rad")),
    0.0, 1.0e-12);
  EXPECT_NEAR(
    std::stod(DiagnosticValue(result.diagnostics, "alignment_translation_x_m")),
    0.0, 1.0e-12);
  EXPECT_NEAR(
    std::stod(DiagnosticValue(result.diagnostics, "alignment_translation_y_m")),
    0.0, 1.0e-12);
  EXPECT_NEAR(
    std::stod(DiagnosticValue(result.diagnostics, "alignment_translation_z_m")),
    0.0, 1.0e-12);
}

TEST_F(SelectorPoseSemantics, CuvslamStillAnchorsTheInitialPoseAtTheLocalOrigin)
{
  constexpr double kInitialYaw = 0.6;
  constexpr double kFollowupYaw = 0.9;
  const double expected_x =
    std::cos(kInitialYaw) * 0.25 + std::sin(kInitialYaw) * -0.25;
  const double expected_y =
    -std::sin(kInitialYaw) * 0.25 + std::cos(kInitialYaw) * -0.25;
  const auto result = SelectPose(CuvslamFixture());
  ASSERT_FALSE(result.outputs.empty());
  const auto & initial = result.outputs.front();
  const auto & followup = result.outputs.back();

  EXPECT_EQ(initial.header.frame_id, "map");
  EXPECT_EQ(initial.mode, "cuvslam_primary");
  EXPECT_NEAR(initial.pose.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.position.z, 0.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.orientation.z, 0.0, 1.0e-12);
  EXPECT_NEAR(initial.pose.orientation.w, 1.0, 1.0e-12);
  EXPECT_NEAR(followup.pose.position.x, expected_x, 1.0e-12);
  EXPECT_NEAR(followup.pose.position.y, expected_y, 1.0e-12);
  EXPECT_NEAR(followup.pose.position.z, 0.3, 1.0e-12);
  EXPECT_NEAR(
    followup.pose.orientation.z,
    std::sin(0.5 * (kFollowupYaw - kInitialYaw)), 1.0e-12);
  EXPECT_NEAR(
    followup.pose.orientation.w,
    std::cos(0.5 * (kFollowupYaw - kInitialYaw)), 1.0e-12);
  EXPECT_EQ(
    DiagnosticValue(result.diagnostics, "pose_reference_semantics"),
    "epoch_local_initial_pose");
}

}  // namespace
}  // namespace localization_source_selector
