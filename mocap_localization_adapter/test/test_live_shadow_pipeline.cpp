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
#include <sstream>
#include <string>
#include <thread>

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

std::string DiagnosticSummary(
  const std::optional<DiagnosticStatus> & latest_status)
{
  if (!latest_status.has_value()) {
    return "mocap shadow diagnostic status was not received";
  }
  const auto & status = latest_status.value();
  std::ostringstream summary;
  summary << "level=" << static_cast<int>(status.level) <<
    " message=" << status.message <<
    " health_state=" << DiagnosticValue(status, "health_state") <<
    " reason_code=" << DiagnosticValue(status, "reason_code") <<
    " observed_pose_rate_hz=" << DiagnosticValue(status, "observed_pose_rate_hz") <<
    " accepted=" << DiagnosticValue(status, "accepted") <<
    " rejected=" << DiagnosticValue(status, "rejected") <<
    " recovery=" << DiagnosticValue(status, "recovery_progress") <<
    "/" << DiagnosticValue(status, "recovery_required") <<
    " publisher_count=" << DiagnosticValue(status, "publisher_count") <<
    " publisher_identity_valid=" << DiagnosticValue(status, "publisher_identity_valid") <<
    " publisher_qos_valid=" << DiagnosticValue(status, "publisher_qos_valid") <<
    " actual_qos_reliability=" << DiagnosticValue(status, "actual_qos_reliability") <<
    " actual_qos_durability=" << DiagnosticValue(status, "actual_qos_durability") <<
    " actual_qos_history=" << DiagnosticValue(status, "actual_qos_history") <<
    " output_publisher_count=" << DiagnosticValue(status, "output_publisher_count") <<
    " receive_gap_violation=" << DiagnosticValue(status, "receive_gap_violation") <<
    " stamp_gap_violation=" << DiagnosticValue(status, "stamp_gap_violation") <<
    " clock_domain_mismatch=" << DiagnosticValue(status, "clock_domain_mismatch");
  return summary.str();
}

class LiveShadowPipeline : public ::testing::Test
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

TEST_F(LiveShadowPipeline, PublishesOnlyTypedCandidateAfterHealthWarmup)
{
  auto adapter = std::make_shared<MocapLocalizationAdapter>();
  auto source = std::make_shared<rclcpp::Node>("vrpn_client_node");
  auto observer = std::make_shared<rclcpp::Node>("shadow_candidate_observer");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto pose_publisher = source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  std::optional<ShadowPoseCandidate> candidate;
  auto candidate_subscription = observer->create_subscription<ShadowPoseCandidate>(
    "/localization/shadow/mocap/assumed_base_pose",
    qos,
    [&candidate](const ShadowPoseCandidate::ConstSharedPtr message) {candidate = *message;});
  std::optional<LocalizationSourceCandidate> source_candidate;
  auto source_candidate_subscription =
    observer->create_subscription<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose",
    qos,
    [&source_candidate](const LocalizationSourceCandidate::ConstSharedPtr message) {
      source_candidate = *message;
    });
  const std::string diagnostic_status_name =
    std::string(adapter->get_fully_qualified_name()) + ": mocap shadow contract";
  std::optional<DiagnosticStatus> latest_diagnostic_status;
  auto diagnostics_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics",
    qos,
    [&latest_diagnostic_status, diagnostic_status_name](
      const DiagnosticArray::ConstSharedPtr message)
    {
      for (const auto & status : message->status) {
        if (status.name == diagnostic_status_name) {
          latest_diagnostic_status = status;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(source);
  executor.add_node(observer);

  const auto graph_is_ready = [&adapter]() {
      return
        adapter->get_publishers_info_by_topic("/droneyee207/pose").size() == 1U &&
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

  const auto diagnostic_authority_is_ready = [&latest_diagnostic_status]() {
      if (!latest_diagnostic_status.has_value()) {
        return false;
      }
      const auto & status = latest_diagnostic_status.value();
      return DiagnosticValue(status, "publisher_count") == "1" &&
             DiagnosticValue(status, "publisher_identity_valid") == "1" &&
             DiagnosticValue(status, "publisher_type_valid") == "1" &&
             DiagnosticValue(status, "publisher_qos_valid") == "1" &&
             DiagnosticValue(status, "output_publisher_count") == "1" &&
             DiagnosticValue(status, "output_publisher_identity_valid") == "1" &&
             DiagnosticValue(status, "output_publisher_type_valid") == "1" &&
             DiagnosticValue(status, "source_candidate_publisher_count") == "1" &&
             DiagnosticValue(status, "source_candidate_publisher_identity_valid") == "1" &&
             DiagnosticValue(status, "source_candidate_publisher_type_valid") == "1" &&
             DiagnosticValue(status, "source_candidate_publisher_gid_valid") == "1";
    };
  const auto evidence_deadline = std::chrono::steady_clock::now() + 5s;
  while (!diagnostic_authority_is_ready() &&
    std::chrono::steady_clock::now() < evidence_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(diagnostic_authority_is_ready()) <<
    DiagnosticSummary(latest_diagnostic_status);
  const auto & authority_status = latest_diagnostic_status.value();
  const std::string actual_qos_history =
    DiagnosticValue(authority_status, "actual_qos_history");
  ASSERT_TRUE(actual_qos_history == "keep_last" || actual_qos_history == "unknown") <<
    DiagnosticSummary(latest_diagnostic_status);
  EXPECT_EQ(
    DiagnosticValue(authority_status, "qos_history_keep_last_confirmed"),
    actual_qos_history == "keep_last" ? "1" : "0");
  EXPECT_EQ(
    DiagnosticValue(authority_status, "qos_history_unknown_shadow_fallback"),
    actual_qos_history == "unknown" ? "1" : "0");

  constexpr std::int64_t kPosePeriodNanoseconds = 8000000LL;
  const auto pose_period = std::chrono::nanoseconds(kPosePeriodNanoseconds);
  const auto first_pose_steady_time = std::chrono::steady_clock::now() + pose_period;
  const std::int64_t first_pose_stamp_ns =
    source->get_clock()->now().nanoseconds() + kPosePeriodNanoseconds;
  std::size_t pose_index = 0U;
  const auto publish_pose = [&]() {
      const auto pose_offset =
        std::chrono::nanoseconds(
        static_cast<std::int64_t>(pose_index) * kPosePeriodNanoseconds);
      const auto publish_deadline = first_pose_steady_time + pose_offset;
      while (std::chrono::steady_clock::now() < publish_deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = rclcpp::Time(
        first_pose_stamp_ns + pose_offset.count(),
        source->get_clock()->get_clock_type());
      pose.header.frame_id = "world";
      pose.pose.position.x = 1.0;
      pose.pose.position.y = 2.0;
      pose.pose.position.z = 3.0;
      pose.pose.orientation.w = 1.0;
      pose_publisher->publish(pose);
      ++pose_index;
      executor.spin_some();
    };
  for (std::size_t index = 0U; index < 119U; ++index) {
    publish_pose();
  }
  EXPECT_FALSE(candidate.has_value());
  EXPECT_FALSE(source_candidate.has_value());

  while (pose_index < 500U &&
    (!candidate.has_value() || !source_candidate.has_value()))
  {
    publish_pose();
  }
  const auto drain_deadline = std::chrono::steady_clock::now() + 150ms;
  while ((!candidate.has_value() || !source_candidate.has_value()) &&
    std::chrono::steady_clock::now() < drain_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  ASSERT_TRUE(candidate.has_value()) << DiagnosticSummary(latest_diagnostic_status);
  EXPECT_EQ(candidate->header.frame_id, "mocap_world");
  EXPECT_EQ(candidate->semantic_child_frame, "base_link");
  EXPECT_DOUBLE_EQ(candidate->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(candidate->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(candidate->pose.position.z, 3.0);
  EXPECT_EQ(candidate->contract_id, "droneyee207_mocap_shadow_20260722_v2");
  EXPECT_EQ(candidate->authorization, "shadow_candidate_only");
  EXPECT_EQ(candidate->source_topic, "/droneyee207/pose");
  EXPECT_EQ(candidate->source_parent_frame, "world");
  EXPECT_EQ(candidate->source_child_frame, "mocap_rigid_body");
  EXPECT_EQ(
    candidate->source_world_axes,
    "right_handed_x_reference_y_left_z_up_local_lab");
  EXPECT_EQ(candidate->source_body_axes, "x_forward_y_left_z_up");
  EXPECT_FALSE(candidate->geographic_alignment_validated);
  EXPECT_EQ(candidate->world_alignment_status, "local_lab_identity_not_geographic");
  EXPECT_FALSE(candidate->world_alignment_approved);
  EXPECT_EQ(candidate->extrinsic_status, "assumed_coincident_not_measured");
  EXPECT_EQ(
    candidate->extrinsic_assumption_id,
    "four_markers_centered_at_fcu_imu_20260722");
  EXPECT_FALSE(candidate->extrinsic_approved);
  EXPECT_EQ(candidate->expected_source_revision, "vrpn_client_ros2@1b9731c");
  EXPECT_EQ(
    candidate->expected_timestamp_semantics,
    "jetson_ros_callback_time_use_server_time_false");
  EXPECT_FALSE(candidate->source_configuration_validated);
  EXPECT_FALSE(candidate->capture_time_validated);

  ASSERT_TRUE(source_candidate.has_value()) << DiagnosticSummary(latest_diagnostic_status);
  EXPECT_EQ(source_candidate->header.stamp, candidate->header.stamp);
  EXPECT_EQ(source_candidate->header.frame_id, "mocap_world");
  EXPECT_EQ(source_candidate->semantic_child_frame, "base_link");
  EXPECT_DOUBLE_EQ(source_candidate->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(source_candidate->pose.orientation.w, 1.0);
  EXPECT_EQ(source_candidate->source_id, "mocap");
  EXPECT_EQ(
    source_candidate->source_contract_id,
    "droneyee207_mocap_shadow_20260722_v2");
  EXPECT_EQ(source_candidate->authorization, "source_pose_candidate_only");

  (void)candidate_subscription;
  (void)source_candidate_subscription;
  (void)diagnostics_subscription;
}

}  // namespace
}  // namespace mocap_localization_adapter
