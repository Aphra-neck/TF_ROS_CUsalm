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

#include "mocap_localization_adapter/adapter_config.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "localization_contracts/rigid_transform.hpp"

namespace mocap_localization_adapter
{
namespace
{

using localization_contracts::QuaternionXyzw;
using localization_contracts::RigidTransform;

rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

void RequireEqual(
  const std::string & parameter,
  const std::string & actual,
  const std::string & expected)
{
  if (actual != expected) {
    throw std::runtime_error(
            parameter + " must be exactly '" + expected + "' in schema version one");
  }
}

void RequirePositive(const std::string & parameter, const double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(parameter + " must be finite and positive");
  }
}

void RequireExact(const std::string & parameter, const double actual, const double expected)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1.0e-12) {
    throw std::runtime_error(
            parameter + " is immutable in schema version one; expected " +
            std::to_string(expected));
  }
}

std::size_t PositiveSize(const std::string & parameter, const std::int64_t value)
{
  if (value <= 0) {
    throw std::runtime_error(parameter + " must be positive");
  }
  return static_cast<std::size_t>(value);
}

RigidTransform TransformFromParameters(
  rclcpp::Node * node,
  const std::string & translation_parameter,
  const std::string & rotation_parameter,
  const std::string & parent_frame,
  const std::string & child_frame)
{
  const auto translation = node->declare_parameter<std::vector<double>>(
    translation_parameter,
    std::vector<double>{0.0, 0.0, 0.0},
    ReadOnlyDescriptor("Translation in metres, expressed in the parent frame."));
  const auto rotation = node->declare_parameter<std::vector<double>>(
    rotation_parameter,
    std::vector<double>{0.0, 0.0, 0.0, 1.0},
    ReadOnlyDescriptor("Quaternion in ROS xyzw order."));
  if (translation.size() != 3U || rotation.size() != 4U) {
    throw std::runtime_error(
            translation_parameter + " must have 3 values and " +
            rotation_parameter + " must have 4 values");
  }
  return RigidTransform::Create(
    parent_frame,
    child_frame,
    Eigen::Vector3d(translation[0], translation[1], translation[2]),
    QuaternionXyzw{rotation[0], rotation[1], rotation[2], rotation[3]});
}

}  // namespace

AdapterConfig DeclareAndValidateConfig(rclcpp::Node * node)
{
  if (node == nullptr) {
    throw std::invalid_argument("node must not be null");
  }

  const std::string contract_id = node->declare_parameter<std::string>(
    "contract_id",
    "droneyee207_mocap_shadow_20260722_v2",
    ReadOnlyDescriptor("Version-controlled shadow contract identifier."));
  const std::string source_revision = node->declare_parameter<std::string>(
    "source_revision",
    "vrpn_client_ros2@1b9731c",
    ReadOnlyDescriptor("Audited VRPN source revision."));
  const std::string mode = node->declare_parameter<std::string>(
    "mode", "shadow", ReadOnlyDescriptor("This revision accepts only shadow mode."));
  const std::string authorization = node->declare_parameter<std::string>(
    "authorization",
    "shadow_candidate_only",
    ReadOnlyDescriptor("Authorizes only the typed source-private shadow candidate."));
  const std::string input_topic = node->declare_parameter<std::string>(
    "input_topic",
    "/droneyee207/pose",
    ReadOnlyDescriptor("Explicitly selected VRPN rigid-body pose topic."));
  const std::string expected_publisher = node->declare_parameter<std::string>(
    "expected_publisher",
    "/vrpn_client_node",
    ReadOnlyDescriptor("Only publisher authorized for the input topic."));
  const std::string expected_input_frame = node->declare_parameter<std::string>(
    "expected_input_frame",
    "world",
    ReadOnlyDescriptor("Exact VRPN PoseStamped header frame."));
  const std::string semantic_input_child_frame = node->declare_parameter<std::string>(
    "semantic_input_child_frame",
    "mocap_rigid_body",
    ReadOnlyDescriptor("Contract child frame absent from PoseStamped itself."));
  const std::string output_topic = node->declare_parameter<std::string>(
    "output_topic",
    "/localization/shadow/mocap/assumed_base_pose",
    ReadOnlyDescriptor("Typed source-private candidate; never canonical state or PX4 input."));
  const std::string output_parent_frame = node->declare_parameter<std::string>(
    "output_parent_frame",
    "mocap_world",
    ReadOnlyDescriptor("Local laboratory world; not geographic ENU."));
  const std::string output_child_frame = node->declare_parameter<std::string>(
    "output_child_frame",
    "base_link",
    ReadOnlyDescriptor("Semantic child explicitly carried by the shadow message."));
  const std::string diagnostics_topic = node->declare_parameter<std::string>(
    "diagnostics_topic", "/diagnostics", ReadOnlyDescriptor("ROS diagnostics topic."));
  const std::string world_axes = node->declare_parameter<std::string>(
    "world_axes",
    "right_handed_x_reference_y_left_z_up_local_lab",
    ReadOnlyDescriptor("Observed fixed laboratory world-axis convention."));
  const std::string rigid_body_axes = node->declare_parameter<std::string>(
    "rigid_body_axes",
    "x_forward_y_left_z_up",
    ReadOnlyDescriptor("Configured droneyee207 rigid-body axis convention."));
  const std::string timestamp_semantics = node->declare_parameter<std::string>(
    "timestamp_semantics",
    "jetson_ros_callback_time_use_server_time_false",
    ReadOnlyDescriptor("Audited header timestamp source in the VRPN driver."));
  const std::string extrinsic_status = node->declare_parameter<std::string>(
    "extrinsic_status",
    "assumed_coincident_not_measured",
    ReadOnlyDescriptor("Identity installation assumption; not flight-approved calibration."));
  const std::string extrinsic_assumption_id = node->declare_parameter<std::string>(
    "extrinsic_assumption_id",
    "four_markers_centered_at_fcu_imu_20260722",
    ReadOnlyDescriptor("Physical rationale for the shadow-only identity assumption."));

  RequireEqual("mode", mode, "shadow");
  RequireEqual("authorization", authorization, "shadow_candidate_only");
  RequireEqual("input_topic", input_topic, "/droneyee207/pose");
  RequireEqual("expected_publisher", expected_publisher, "/vrpn_client_node");
  RequireEqual("expected_input_frame", expected_input_frame, "world");
  RequireEqual("semantic_input_child_frame", semantic_input_child_frame, "mocap_rigid_body");
  RequireEqual(
    "output_topic", output_topic, "/localization/shadow/mocap/assumed_base_pose");
  RequireEqual("output_parent_frame", output_parent_frame, "mocap_world");
  RequireEqual("output_child_frame", output_child_frame, "base_link");
  RequireEqual("diagnostics_topic", diagnostics_topic, "/diagnostics");
  RequireEqual(
    "world_axes", world_axes, "right_handed_x_reference_y_left_z_up_local_lab");
  RequireEqual("rigid_body_axes", rigid_body_axes, "x_forward_y_left_z_up");
  RequireEqual(
    "timestamp_semantics",
    timestamp_semantics,
    "jetson_ros_callback_time_use_server_time_false");
  RequireEqual("extrinsic_status", extrinsic_status, "assumed_coincident_not_measured");
  RequireEqual("contract_id", contract_id, "droneyee207_mocap_shadow_20260722_v2");
  RequireEqual("source_revision", source_revision, "vrpn_client_ros2@1b9731c");
  RequireEqual(
    "extrinsic_assumption_id",
    extrinsic_assumption_id,
    "four_markers_centered_at_fcu_imu_20260722");

  const auto output_world_from_input_world = TransformFromParameters(
    node,
    "mocap_world_from_world_translation_m",
    "mocap_world_from_world_rotation_xyzw",
    output_parent_frame,
    expected_input_frame);
  const auto base_from_rigid_body = TransformFromParameters(
    node,
    "base_from_rigid_body_translation_m",
    "base_from_rigid_body_rotation_xyzw",
    output_child_frame,
    semantic_input_child_frame);
  const auto expected_world_alignment = RigidTransform::Create(
    output_parent_frame,
    expected_input_frame,
    Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto expected_installation_assumption = RigidTransform::Create(
    output_child_frame,
    semantic_input_child_frame,
    Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  if (!output_world_from_input_world.IsEquivalent(expected_world_alignment) ||
    !base_from_rigid_body.IsEquivalent(expected_installation_assumption))
  {
    throw std::runtime_error(
            "schema version one is locked to the observed world alignment and "
            "coincident rigid-body installation assumption");
  }

  HealthConfig health{
    node->declare_parameter<double>(
      "health.diagnostic_period_sec", 0.1,
      ReadOnlyDescriptor("Diagnostic publication period.")),
    node->declare_parameter<double>(
      "health.startup_timeout_sec", 10.0,
      ReadOnlyDescriptor("Maximum initial wait for a valid publisher and pose.")),
    node->declare_parameter<double>(
      "health.stale_after_sec", 0.25,
      ReadOnlyDescriptor("Maximum steady-clock age of the last valid pose.")),
    node->declare_parameter<double>(
      "health.minimum_pose_rate_hz", 90.0,
      ReadOnlyDescriptor("Candidate minimum source-stamp rate.")),
    node->declare_parameter<double>(
      "health.maximum_stamp_gap_sec", 0.05,
      ReadOnlyDescriptor("Candidate maximum source timestamp gap.")),
    node->declare_parameter<double>(
      "health.maximum_receive_gap_sec", 0.05,
      ReadOnlyDescriptor("Candidate maximum steady-clock callback gap.")),
    node->declare_parameter<double>(
      "health.maximum_clock_residual_sec", 0.25,
      ReadOnlyDescriptor("Maximum local ROS clock minus header stamp magnitude.")),
    node->declare_parameter<double>(
      "health.maximum_position_norm_m", 20.0,
      ReadOnlyDescriptor("Shadow-only finite workspace sanity radius.")),
    node->declare_parameter<double>(
      "health.maximum_position_step_m", 0.5,
      ReadOnlyDescriptor("Shadow reset-candidate translation threshold.")),
    node->declare_parameter<double>(
      "health.maximum_rotation_step_rad", 0.7853981633974483,
      ReadOnlyDescriptor("Shadow reset-candidate rotation threshold.")),
    PositiveSize(
      "health.pose_rate_window_samples",
      node->declare_parameter<std::int64_t>(
        "health.pose_rate_window_samples", 120,
        ReadOnlyDescriptor("Number of accepted stamps in the rate window."))),
    PositiveSize(
      "health.recovery_consecutive_samples",
      node->declare_parameter<std::int64_t>(
        "health.recovery_consecutive_samples", 120,
        ReadOnlyDescriptor("Consecutive valid samples required after a transient fault.")))};

  RequirePositive("health.diagnostic_period_sec", health.diagnostic_period_sec);
  RequirePositive("health.startup_timeout_sec", health.startup_timeout_sec);
  RequirePositive("health.stale_after_sec", health.stale_after_sec);
  RequirePositive("health.minimum_pose_rate_hz", health.minimum_pose_rate_hz);
  RequirePositive("health.maximum_stamp_gap_sec", health.maximum_stamp_gap_sec);
  RequirePositive("health.maximum_receive_gap_sec", health.maximum_receive_gap_sec);
  RequirePositive("health.maximum_clock_residual_sec", health.maximum_clock_residual_sec);
  RequirePositive("health.maximum_position_norm_m", health.maximum_position_norm_m);
  RequirePositive("health.maximum_position_step_m", health.maximum_position_step_m);
  RequirePositive("health.maximum_rotation_step_rad", health.maximum_rotation_step_rad);
  if (health.pose_rate_window_samples < 2U) {
    throw std::runtime_error("health.pose_rate_window_samples must be at least two");
  }
  if (health.diagnostic_period_sec >= health.stale_after_sec ||
    health.maximum_stamp_gap_sec >= health.stale_after_sec ||
    health.maximum_receive_gap_sec >= health.stale_after_sec)
  {
    throw std::runtime_error("diagnostic period and gap thresholds must be below stale timeout");
  }
  RequireExact("health.diagnostic_period_sec", health.diagnostic_period_sec, 0.1);
  RequireExact("health.startup_timeout_sec", health.startup_timeout_sec, 10.0);
  RequireExact("health.stale_after_sec", health.stale_after_sec, 0.25);
  RequireExact("health.minimum_pose_rate_hz", health.minimum_pose_rate_hz, 90.0);
  RequireExact("health.maximum_stamp_gap_sec", health.maximum_stamp_gap_sec, 0.05);
  RequireExact("health.maximum_receive_gap_sec", health.maximum_receive_gap_sec, 0.05);
  RequireExact("health.maximum_clock_residual_sec", health.maximum_clock_residual_sec, 0.25);
  RequireExact("health.maximum_position_norm_m", health.maximum_position_norm_m, 20.0);
  RequireExact("health.maximum_position_step_m", health.maximum_position_step_m, 0.5);
  RequireExact(
    "health.maximum_rotation_step_rad",
    health.maximum_rotation_step_rad,
    0.7853981633974483);
  if (health.pose_rate_window_samples != 120U ||
    health.recovery_consecutive_samples != 120U)
  {
    throw std::runtime_error("schema version one rate and recovery windows must both be 120");
  }

  return AdapterConfig{
    contract_id,
    source_revision,
    mode,
    authorization,
    input_topic,
    expected_publisher,
    expected_input_frame,
    semantic_input_child_frame,
    output_topic,
    output_parent_frame,
    output_child_frame,
    diagnostics_topic,
    world_axes,
    rigid_body_axes,
    timestamp_semantics,
    extrinsic_status,
    extrinsic_assumption_id,
    output_world_from_input_world,
    base_from_rigid_body,
    health};
}

}  // namespace mocap_localization_adapter
