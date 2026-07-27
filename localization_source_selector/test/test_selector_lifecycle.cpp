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
#include <limits>
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

class SelectorLifecycle : public ::testing::Test
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

rclcpp::NodeOptions LifecycleOptions()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "cuvslam_primary"),
        rclcpp::Parameter(
          "contract_file",
          std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml"),
      });
  return options;
}

LocalizationSourceCandidate Message(rclcpp::Node & source, const double x)
{
  LocalizationSourceCandidate message;
  message.header.stamp = source.get_clock()->now();
  message.header.frame_id = "odom";
  message.semantic_child_frame = "base_link";
  message.pose.position.x = x;
  message.pose.orientation.w = 1.0;
  message.source_id = "cuvslam";
  message.source_contract_id = "d435i_fcu_cuvslam_shadow_20260723_v2";
  message.authorization = "source_pose_candidate_only";
  return message;
}

void SpinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

rclcpp::Subscription<DiagnosticArray>::SharedPtr ObserveSelectorDiagnostics(
  rclcpp::Node & observer,
  std::optional<DiagnosticStatus> & latest_status)
{
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  return observer.create_subscription<DiagnosticArray>(
    "/diagnostics", qos,
    [&latest_status](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name.find("localization source selector") != std::string::npos) {
          latest_status = status;
        }
      }
    });
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

TEST_F(SelectorLifecycle, StaleSourceRecoversOnlyOnTheBoundEpochWithoutRealignment)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(LifecycleOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selector_lifecycle_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  SpinFor(executor, 200ms);

  publisher->publish(Message(*source, 10.0));
  SpinFor(executor, 80ms);
  ASSERT_EQ(outputs.size(), 1U);
  const std::string epoch = outputs.front().localization_epoch_id;
  EXPECT_NEAR(outputs.front().pose.position.x, 0.0, 1.0e-9);
  EXPECT_EQ(DiagnosticValue(latest_status, "output_publisher_gid_valid"), "1");
  EXPECT_NE(
    DiagnosticValue(latest_status, "output_publisher_gid"),
    std::string(2U * RMW_GID_STORAGE_SIZE, '0'));

  SpinFor(executor, 300ms);
  const std::size_t before_recovery = outputs.size();
  for (int index = 0; index < 9; ++index) {
    publisher->publish(Message(*source, 10.6 + 0.01 * index));
    SpinFor(executor, 10ms);
    EXPECT_EQ(outputs.size(), before_recovery);
  }
  publisher->publish(Message(*source, 10.69));
  SpinFor(executor, 80ms);
  ASSERT_EQ(outputs.size(), before_recovery + 1U);
  EXPECT_EQ(outputs.back().localization_epoch_id, epoch);
  EXPECT_NEAR(outputs.back().pose.position.x, 0.69, 1.0e-8);
  EXPECT_EQ(DiagnosticValue(latest_status, "state"), "healthy");
  EXPECT_EQ(DiagnosticValue(latest_status, "reason_code"), "SOURCE_HEALTHY");
  EXPECT_EQ(DiagnosticValue(latest_status, "pose_reset"), "0");

  auto regressed = Message(*source, 10.4);
  regressed.header.stamp.sec -= 1;
  publisher->publish(regressed);
  SpinFor(executor, 80ms);
  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  EXPECT_EQ(DiagnosticValue(latest_status, "reason_code"), "SOURCE_TIME_REGRESSION");
  const std::size_t before_latched_followup = outputs.size();
  for (int index = 0; index < 12; ++index) {
    publisher->publish(Message(*source, 10.11 + 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(outputs.size(), before_latched_followup);
  (void)diagnostic_subscription;
  (void)subscription;
}

TEST_F(SelectorLifecycle, SourceContractMutationLatchesUntilRestart)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(LifecycleOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selector_contract_mutation_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  auto invalid = Message(*source, 0.0);
  invalid.source_contract_id = "unexpected_contract_epoch";
  publisher->publish(invalid);
  SpinFor(executor, 40ms);
  publisher->publish(Message(*source, 0.1));
  SpinFor(executor, 60ms);
  EXPECT_EQ(output_count, 0U);
  (void)subscription;
}

TEST_F(SelectorLifecycle, SamePublisherFqnWithANewGidCannotResumeTheEpoch)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(LifecycleOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selector_gid_epoch_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  publisher->publish(Message(*source, 0.0));
  SpinFor(executor, 60ms);
  ASSERT_EQ(output_count, 1U);

  executor.remove_node(source);
  publisher.reset();
  source.reset();
  SpinFor(executor, 300ms);

  auto restarted_source =
    std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto restarted_publisher =
    restarted_source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  executor.add_node(restarted_source);
  SpinFor(executor, 200ms);
  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  EXPECT_EQ(
    DiagnosticValue(latest_status, "reason_code"),
    "SOURCE_PUBLISHER_EPOCH_CHANGED");
  const std::size_t before_restarted_message = output_count;
  for (int index = 0; index < 12; ++index) {
    restarted_publisher->publish(Message(*restarted_source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, before_restarted_message);
  (void)diagnostic_subscription;
  (void)subscription;
}

TEST_F(SelectorLifecycle, PoseResetLatchesUntilRestart)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(LifecycleOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selector_pose_reset_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  SpinFor(executor, 200ms);

  publisher->publish(Message(*source, 0.0));
  SpinFor(executor, 80ms);
  ASSERT_EQ(output_count, 1U);
  publisher->publish(Message(*source, 1.0));
  SpinFor(executor, 80ms);
  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  EXPECT_EQ(
    DiagnosticValue(latest_status, "reason_code"),
    "SOURCE_POSE_RESET_DETECTED");

  for (int index = 1; index <= 12; ++index) {
    publisher->publish(Message(*source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, 1U);
  (void)diagnostic_subscription;
  (void)subscription;
}

TEST_F(SelectorLifecycle, NonfiniteAlignedPoseLatchesWithoutPublishing)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(LifecycleOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selector_overflow_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  SpinFor(executor, 200ms);

  auto extreme = Message(*source, std::numeric_limits<double>::max());
  extreme.pose.position.y = std::numeric_limits<double>::max();
  extreme.pose.orientation.z = std::sin(0.39269908169872414);
  extreme.pose.orientation.w = std::cos(0.39269908169872414);
  ASSERT_TRUE(PoseIsFiniteAndNormalized(extreme.pose));
  publisher->publish(extreme);
  SpinFor(executor, 80ms);

  EXPECT_EQ(output_count, 0U);
  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  EXPECT_EQ(DiagnosticValue(latest_status, "reason_code"), "NONFINITE_ALIGNED_POSE");
  (void)diagnostic_subscription;
  (void)subscription;
}

}  // namespace
}  // namespace localization_source_selector
