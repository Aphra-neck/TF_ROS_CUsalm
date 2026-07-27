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

#ifndef MOCAP_LOCALIZATION_ADAPTER__ADAPTER_CONFIG_HPP_
#define MOCAP_LOCALIZATION_ADAPTER__ADAPTER_CONFIG_HPP_

#include <cstddef>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "localization_contracts/rigid_transform.hpp"

namespace mocap_localization_adapter
{

struct HealthConfig
{
  double diagnostic_period_sec;
  double startup_timeout_sec;
  double stale_after_sec;
  double minimum_pose_rate_hz;
  double maximum_stamp_gap_sec;
  double maximum_receive_gap_sec;
  double maximum_clock_residual_sec;
  double maximum_position_norm_m;
  double maximum_position_step_m;
  double maximum_rotation_step_rad;
  std::size_t pose_rate_window_samples;
  std::size_t recovery_consecutive_samples;
};

struct AdapterConfig
{
  std::string contract_id;
  std::string source_revision;
  std::string mode;
  std::string authorization;
  std::string input_topic;
  std::string expected_publisher;
  std::string expected_input_frame;
  std::string semantic_input_child_frame;
  std::string output_topic;
  std::string output_parent_frame;
  std::string output_child_frame;
  std::string diagnostics_topic;
  std::string world_axes;
  std::string rigid_body_axes;
  std::string timestamp_semantics;
  std::string extrinsic_status;
  std::string extrinsic_assumption_id;
  localization_contracts::RigidTransform output_world_from_input_world;
  localization_contracts::RigidTransform base_from_rigid_body;
  HealthConfig health;
};

AdapterConfig DeclareAndValidateConfig(rclcpp::Node * node);

}  // namespace mocap_localization_adapter

#endif  // MOCAP_LOCALIZATION_ADAPTER__ADAPTER_CONFIG_HPP_
