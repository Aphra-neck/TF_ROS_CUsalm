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

std::string DiagnosticValue(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "missing";
}

class AuthorityPipeline
{
public:
  AuthorityPipeline()
  : adapter(MakeAdapter()),
    visual_slam(std::make_shared<rclcpp::Node>("synthetic_visual_slam")),
    diagnostic_source(std::make_shared<rclcpp::Node>("synthetic_diagnostic_source")),
    observer(std::make_shared<rclcpp::Node>("input_authority_observer"))
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    const auto reliable_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    odometry_publisher = visual_slam->create_publisher<Odometry>(
      "/synthetic/odometry", sensor_qos);
    status_publisher = visual_slam->create_publisher<VisualSlamStatus>(
      "/synthetic/status", sensor_qos);
    diagnostic_publisher = diagnostic_source->create_publisher<DiagnosticArray>(
      "/diagnostics", reliable_qos);
    candidate_subscription = observer->create_subscription<LocalizationSourceCandidate>(
      "/localization/candidates/cuvslam/base_pose",
      reliable_qos,
      [this](const LocalizationSourceCandidate::ConstSharedPtr) {
        ++candidate_count;
      });
    const std::string status_name =
      std::string(adapter->get_fully_qualified_name()) + ": localization contract";
    diagnostics_subscription = observer->create_subscription<DiagnosticArray>(
      "/diagnostics",
      reliable_qos,
      [this, status_name](const DiagnosticArray::ConstSharedPtr message) {
        for (const auto & status : message->status) {
          if (status.name == status_name) {
            latest_adapter_status = status;
            ++adapter_diagnostic_count;
          }
        }
      });

    executor.add_node(adapter);
    executor.add_node(visual_slam);
    executor.add_node(diagnostic_source);
    executor.add_node(observer);
  }

  bool WaitForGraph(const std::size_t odometry_count, const std::size_t status_count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (adapter->get_publishers_info_by_topic("/synthetic/odometry").size() ==
        odometry_count &&
        adapter->get_publishers_info_by_topic("/synthetic/status").size() == status_count)
      {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void SpinFor(const std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void PublishEvidence()
  {
    const auto stamp = visual_slam->get_clock()->now();
    PublishDiagnostic(stamp);
    PublishStatus(status_publisher, stamp);
    PublishOdometry(odometry_publisher, stamp);
    SpinFor(20ms);
  }

  bool WarmUntilCandidate()
  {
    for (std::size_t index = 0U; index < 100U && candidate_count == 0U; ++index) {
      PublishEvidence();
    }
    return candidate_count > 0U;
  }

  void PublishDiagnostic(const rclcpp::Time & stamp)
  {
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
  }

  void PublishStatus(
    const rclcpp::Publisher<VisualSlamStatus>::SharedPtr & publisher,
    const rclcpp::Time & stamp)
  {
    VisualSlamStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = "map";
    status.vo_state = 1U;
    publisher->publish(status);
  }

  void PublishOdometry(
    const rclcpp::Publisher<Odometry>::SharedPtr & publisher,
    const rclcpp::Time & stamp)
  {
    constexpr double kSqrtHalf = 0.7071067811865476;
    Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "camera_link";
    odometry.pose.pose.position.x = 0.2;
    odometry.pose.pose.position.y = -0.1;
    odometry.pose.pose.position.z = 0.05;
    odometry.pose.pose.orientation.x = kSqrtHalf;
    odometry.pose.pose.orientation.w = kSqrtHalf;
    publisher->publish(odometry);
  }

  bool WaitForReason(const std::string & expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (latest_adapter_status.has_value() &&
        DiagnosticValue(latest_adapter_status.value(), "reason_code") == expected)
      {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  bool WaitForDiagnosticValue(const std::string & key, const std::string & expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (latest_adapter_status.has_value() &&
        DiagnosticValue(latest_adapter_status.value(), key) == expected)
      {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  bool WaitForDiagnosticAfter(const std::size_t previous_count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (adapter_diagnostic_count > previous_count) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  bool WaitForInputCounters(
    const std::uint64_t received,
    const std::uint64_t status_received,
    const std::uint64_t diagnostic_evidence_received)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (latest_adapter_status.has_value() &&
        std::stoull(DiagnosticValue(latest_adapter_status.value(), "received")) == received &&
        std::stoull(
          DiagnosticValue(latest_adapter_status.value(), "status_received")) ==
        status_received &&
        std::stoull(
          DiagnosticValue(
            latest_adapter_status.value(), "diagnostic_evidence_received")) ==
        diagnostic_evidence_received)
      {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  static std::shared_ptr<CuvslamLocalizationAdapter> MakeAdapter()
  {
    const std::string contract =
      std::string(TEST_FIXTURE_DIR) + "/approved_synthetic_contract.yaml";
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {
          rclcpp::Parameter("mode", "shadow"),
          rclcpp::Parameter("contract_file", contract),
        });
    return std::make_shared<CuvslamLocalizationAdapter>(options);
  }

  std::shared_ptr<CuvslamLocalizationAdapter> adapter;
  std::shared_ptr<rclcpp::Node> visual_slam;
  std::shared_ptr<rclcpp::Node> diagnostic_source;
  std::shared_ptr<rclcpp::Node> observer;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;
  rclcpp::Publisher<VisualSlamStatus>::SharedPtr status_publisher;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostic_publisher;
  rclcpp::Subscription<LocalizationSourceCandidate>::SharedPtr candidate_subscription;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription;
  std::optional<DiagnosticStatus> latest_adapter_status;
  std::size_t adapter_diagnostic_count{0U};
  std::size_t candidate_count{0U};
  rclcpp::executors::SingleThreadedExecutor executor;
};

class InputPublisherAuthority : public ::testing::Test
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

TEST_F(InputPublisherAuthority, DuplicateOdometryGidLatchesBeforeRepublishing)
{
  AuthorityPipeline pipeline;
  ASSERT_TRUE(pipeline.WaitForGraph(1U, 1U));
  ASSERT_TRUE(pipeline.WarmUntilCandidate());
  ASSERT_TRUE(
    pipeline.WaitForDiagnosticValue("source_candidate_publisher_gid_valid", "1"));

  auto spoof = std::make_shared<rclcpp::Node>("spoof_odometry_writer");
  auto spoof_publisher = spoof->create_publisher<Odometry>(
    "/synthetic/odometry", rclcpp::SensorDataQoS().keep_last(10));
  pipeline.executor.add_node(spoof);
  ASSERT_TRUE(pipeline.WaitForGraph(2U, 1U));
  ASSERT_TRUE(pipeline.WaitForReason("ODOMETRY_PUBLISHER_NOT_UNIQUE"));
  const std::size_t diagnostic_count_after_fault = pipeline.adapter_diagnostic_count;
  ASSERT_TRUE(pipeline.WaitForDiagnosticAfter(diagnostic_count_after_fault));

  const std::uint64_t published_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "published_source_candidates"));
  const std::uint64_t received_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "received"));
  const std::uint64_t status_received_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "status_received"));
  const std::uint64_t diagnostic_evidence_after_fault = std::stoull(
    DiagnosticValue(
      pipeline.latest_adapter_status.value(), "diagnostic_evidence_received"));
  pipeline.PublishEvidence();
  pipeline.PublishOdometry(spoof_publisher, spoof->get_clock()->now());
  ASSERT_TRUE(
    pipeline.WaitForInputCounters(
      received_after_fault + 2U,
      status_received_after_fault + 1U,
      diagnostic_evidence_after_fault + 1U));
  EXPECT_EQ(
    std::stoull(
      DiagnosticValue(pipeline.latest_adapter_status.value(), "published_source_candidates")),
    published_after_fault);
  EXPECT_GT(
    std::stoull(
      DiagnosticValue(
        pipeline.latest_adapter_status.value(),
        "odometry_publisher_authority_violation")),
    0U);
}

TEST_F(InputPublisherAuthority, StatusPublisherRestartChangesGidAndEndsEpoch)
{
  AuthorityPipeline pipeline;
  ASSERT_TRUE(pipeline.WaitForGraph(1U, 1U));
  ASSERT_TRUE(pipeline.WarmUntilCandidate());
  ASSERT_TRUE(
    pipeline.WaitForDiagnosticValue("source_candidate_publisher_gid_valid", "1"));

  pipeline.status_publisher.reset();
  ASSERT_TRUE(pipeline.WaitForGraph(1U, 0U));
  auto replacement = std::make_shared<rclcpp::Node>("synthetic_visual_slam");
  auto replacement_publisher = replacement->create_publisher<VisualSlamStatus>(
    "/synthetic/status", rclcpp::SensorDataQoS().keep_last(10));
  pipeline.executor.add_node(replacement);
  ASSERT_TRUE(pipeline.WaitForGraph(1U, 1U));
  ASSERT_TRUE(pipeline.WaitForReason("TRACKING_STATUS_PUBLISHER_GID_CHANGED"));
  const std::size_t diagnostic_count_after_fault = pipeline.adapter_diagnostic_count;
  ASSERT_TRUE(pipeline.WaitForDiagnosticAfter(diagnostic_count_after_fault));

  const std::uint64_t published_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "published_source_candidates"));
  const std::uint64_t received_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "received"));
  const std::uint64_t status_received_after_fault = std::stoull(
    DiagnosticValue(pipeline.latest_adapter_status.value(), "status_received"));
  const std::uint64_t diagnostic_evidence_after_fault = std::stoull(
    DiagnosticValue(
      pipeline.latest_adapter_status.value(), "diagnostic_evidence_received"));
  const auto stamp = replacement->get_clock()->now();
  pipeline.PublishStatus(replacement_publisher, stamp);
  pipeline.PublishDiagnostic(stamp);
  pipeline.PublishOdometry(pipeline.odometry_publisher, stamp);
  ASSERT_TRUE(
    pipeline.WaitForInputCounters(
      received_after_fault + 1U,
      status_received_after_fault + 1U,
      diagnostic_evidence_after_fault + 1U));
  EXPECT_EQ(
    std::stoull(
      DiagnosticValue(pipeline.latest_adapter_status.value(), "published_source_candidates")),
    published_after_fault);
  EXPECT_GT(
    std::stoull(
      DiagnosticValue(
        pipeline.latest_adapter_status.value(),
        "status_publisher_authority_violation")),
    0U);
}

}  // namespace
}  // namespace cuvslam_localization_adapter
