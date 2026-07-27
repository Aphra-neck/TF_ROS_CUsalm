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

#include <algorithm>
#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "audit.hpp"

namespace bag_contract_probe
{
namespace detail
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

KeyValue Value(const std::string & key, const std::string & value)
{
  KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

DiagnosticArray DiagnosticMessage(const DiagnosticStatus & status)
{
  DiagnosticArray message;
  message.header.stamp.sec = 100;
  message.header.stamp.nanosec = 25U;
  message.status.push_back(status);
  return message;
}

DiagnosticStatus HealthyAlignedImuStatus()
{
  DiagnosticStatus status;
  status.level = DiagnosticStatus::OK;
  status.name = "/aligned_fcu_imu_relay: aligned FCU IMU";
  status.message = "aligned FCU IMU stream is healthy";
  status.hardware_id = "px4-highres-imu-105-ttyTHS2";
  status.values = {
    Value("input_topic", "/mavros/imu/data_raw"),
    Value("output_topic", "/fcu/imu/data_raw_aligned"),
    Value("expected_input_frame", "base_link"),
    Value("output_frame", "fcu_imu"),
    Value("imu_to_camera_offset_ns", "1737987"),
    Value("offset_equation", "t_aligned = t_imu_raw + offset"),
    Value("received", "100"),
    Value("published", "100"),
    Value("zero_stamp", "0"),
    Value("invalid_stamp", "0"),
    Value("duplicate", "0"),
    Value("nonmonotonic", "0"),
    Value("frame_mismatch", "0"),
    Value("nonfinite_measurement", "0"),
    Value("clock_domain_mismatch", "0"),
    Value("aligned_out_of_range", "0")};
  return status;
}

DiagnosticStatus HealthyRuntimeStatus()
{
  DiagnosticStatus status;
  status.level = DiagnosticStatus::OK;
  status.name = "/d435i_cuvslam_runtime_health_monitor: calibrated runtime";
  status.message = "calibrated D435i CameraInfo and cuVSLAM odometry are healthy";
  status.hardware_id = "243622070369";
  status.values = {
    Value("calibration_id", "d435i_243622070369_factory_rectified_px4_imu_20260720"),
    Value("left_camera_info_topic", "/camera/infra1/camera_info"),
    Value("right_camera_info_topic", "/camera/infra2/camera_info"),
    Value("camera_imu_topic", "/camera/imu"),
    Value("odometry_topic", "/visual_slam/tracking/odometry"),
    Value("expected_odometry_frame", "odom"),
    Value("expected_odometry_child_frame", "camera_link"),
    Value("known_right_frame_reuse_observed", "True"),
    Value("left_camera_info", "100"),
    Value("right_camera_info", "100"),
    Value("odometry", "100"),
    Value("forbidden_camera_imu", "0")};
  return status;
}

DiagnosticStatus HealthyMocapStatus()
{
  DiagnosticStatus status;
  status.level = DiagnosticStatus::WARN;
  status.name = "/mocap_localization_adapter: mocap shadow contract";
  status.message =
    "mocap input is healthy; output remains a shadow-only pose candidate";
  status.hardware_id = "droneyee207";
  status.values = {
    Value("contract_id", "droneyee207_mocap_shadow_20260722_v2"),
    Value("expected_source_revision", "vrpn_client_ros2@1b9731c"),
    Value("source_configuration_validated", "0"),
    Value("mode", "shadow"),
    Value("authorization", "shadow_candidate_only"),
    Value("health_state", "healthy"),
    Value("reason_code", "INPUT_HEALTHY"),
    Value("latched", "0"),
    Value("input_topic", "/droneyee207/pose"),
    Value("input_type", "geometry_msgs/msg/PoseStamped"),
    Value("expected_publisher", "/vrpn_client_node"),
    Value("actual_publisher", "/vrpn_client_node"),
    Value("publisher_count", "1"),
    Value("publisher_identity_valid", "1"),
    Value("publisher_type_valid", "1"),
    Value("publisher_qos_valid", "1"),
    Value("output_publisher_count", "1"),
    Value("output_publisher_identity_valid", "1"),
    Value("output_publisher_type_valid", "1"),
    Value("expected_input_frame", "world"),
    Value("semantic_input_child_frame", "mocap_rigid_body"),
    Value("world_axes", "right_handed_x_reference_y_left_z_up_local_lab"),
    Value("rigid_body_axes", "x_forward_y_left_z_up"),
    Value("expected_timestamp_semantics", "jetson_ros_callback_time_use_server_time_false"),
    Value("capture_time_validated", "0"),
    Value("output_topic", "/localization/shadow/mocap/assumed_base_pose"),
    Value("output_type", "localization_adapter_interfaces/msg/ShadowPoseCandidate"),
    Value("output_parent_frame", "mocap_world"),
    Value("output_child_frame_semantic", "base_link"),
    Value("extrinsic_status", "assumed_coincident_not_measured"),
    Value("extrinsic_assumption_id", "four_markers_centered_at_fcu_imu_20260722"),
    Value("world_alignment_status", "local_lab_identity_not_geographic"),
    Value("world_alignment_approved", "0"),
    Value("standard_odometry_authorized", "0"),
    Value("tf_authorized", "0"),
    Value("mavros_authorized", "0"),
    Value("vrpn_twist_consumed", "0"),
    Value("vrpn_accel_consumed", "0"),
    Value("received", "100"),
    Value("accepted", "100"),
    Value("published_shadow_candidates", "100"),
    Value("rejected", "0"),
    Value("zero_or_invalid_stamp", "0"),
    Value("duplicate", "0"),
    Value("nonmonotonic", "0"),
    Value("frame_mismatch", "0"),
    Value("nonfinite_position", "0"),
    Value("out_of_bounds_position", "0"),
    Value("invalid_quaternion", "0"),
    Value("clock_domain_mismatch", "0"),
    Value("stamp_gap_violation", "0"),
    Value("receive_gap_violation", "0"),
    Value("pose_reset_candidate", "0"),
    Value("publisher_authority_violation", "0"),
    Value("output_publisher_authority_violation", "0")};
  return status;
}

TEST(DiagnosticAudit, AcceptsHealthyRequiredContracts)
{
  AuditData audit;
  audit.ObserveDiagnostics(DiagnosticMessage(HealthyAlignedImuStatus()), 100000000025LL);
  audit.ObserveDiagnostics(DiagnosticMessage(HealthyRuntimeStatus()), 100000000025LL);
  audit.ObserveDiagnostics(DiagnosticMessage(HealthyMocapStatus()), 100000000025LL);

  EXPECT_EQ(audit.aligned_imu_diagnostics.samples, 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.malformed_samples, 0U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.contract_mismatches, 0U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.invariant_violations, 0U);
  EXPECT_EQ(audit.runtime_diagnostics.samples, 1U);
  EXPECT_EQ(audit.runtime_diagnostics.contract_mismatches, 0U);
  EXPECT_EQ(audit.mocap_diagnostics.contract_mismatches, 0U);
  EXPECT_EQ(audit.mocap_diagnostics.invariant_violations, 0U);
}

TEST(DiagnosticAudit, RejectsUnhealthyAlignedImuAndBrokenCounterInvariant)
{
  AuditData audit;
  auto status = HealthyAlignedImuStatus();
  status.level = DiagnosticStatus::WARN;
  for (auto & value : status.values) {
    if (value.key == "published") {
      value.value = "99";
    }
  }
  audit.ObserveDiagnostics(DiagnosticMessage(status), 100000000025LL);

  EXPECT_EQ(audit.aligned_imu_diagnostics.contract_mismatches, 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.invariant_violations, 1U);
}

TEST(DiagnosticAudit, PreservesPreexistingAlignedImuCounterOutsideStaticContract)
{
  AuditData audit;
  auto status = HealthyAlignedImuStatus();
  for (auto & value : status.values) {
    if (value.key == "received") {
      value.value = "101";
    } else if (value.key == "clock_domain_mismatch") {
      value.value = "1";
    }
  }
  audit.ObserveDiagnostics(DiagnosticMessage(status), 100000000025LL);

  EXPECT_EQ(audit.aligned_imu_diagnostics.samples, 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.malformed_samples, 0U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.contract_mismatches, 0U);
  ASSERT_EQ(audit.aligned_imu_diagnostics.counters.at("clock_domain_mismatch").size(), 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.counters.at("clock_domain_mismatch").front(), 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.invariant_violations, 0U);
}

TEST(DiagnosticAudit, RejectsMissingOrNonnumericAlignedImuCountersAsMalformed)
{
  AuditData audit;
  auto status = HealthyAlignedImuStatus();
  status.values.erase(
    std::remove_if(
      status.values.begin(), status.values.end(),
      [](const KeyValue & value) {return value.key == "duplicate";}),
    status.values.end());
  for (auto & value : status.values) {
    if (value.key == "nonmonotonic") {
      value.value = "not-a-number";
    }
  }
  audit.ObserveDiagnostics(DiagnosticMessage(status), 100000000025LL);

  EXPECT_EQ(audit.aligned_imu_diagnostics.malformed_samples, 1U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.contract_mismatches, 0U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.counters.count("duplicate"), 0U);
  EXPECT_EQ(audit.aligned_imu_diagnostics.counters.count("nonmonotonic"), 0U);
}

TEST(DiagnosticAudit, DoesNotConflateMissingCounterWithDuplicateKey)
{
  AuditData audit;
  auto status = HealthyRuntimeStatus();
  status.values.pop_back();
  audit.ObserveDiagnostics(DiagnosticMessage(status), 100000000025LL);

  EXPECT_EQ(audit.runtime_diagnostics.malformed_samples, 1U);
  EXPECT_EQ(audit.diagnostic_statuses.at(status.name).duplicate_keys, 0U);
}

}  // namespace
}  // namespace detail
}  // namespace bag_contract_probe
