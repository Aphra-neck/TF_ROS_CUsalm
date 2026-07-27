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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/rigid_transform.hpp"
#include "mocap_localization_adapter/pose_transform.hpp"

namespace mocap_localization_adapter
{
namespace
{

using localization_contracts::FrameContractViolation;
using localization_contracts::QuaternionToRosXyzw;
using localization_contracts::QuaternionXyzw;
using localization_contracts::RigidTransform;

TEST(MocapPoseTransform, IdentityAssumptionPreservesMeasuredPose)
{
  const auto normalized_world_from_world = RigidTransform::Create(
    "mocap_world", "world", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const Eigen::Quaterniond measured_rotation(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  const auto world_from_rigid_body = RigidTransform::Create(
    "world", "mocap_rigid_body", Eigen::Vector3d(1.0, -2.0, 0.5),
    QuaternionToRosXyzw(measured_rotation));
  const auto base_from_rigid_body = RigidTransform::Create(
    "base_link", "mocap_rigid_body", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});

  const auto result = MocapPoseToBasePose(
    normalized_world_from_world, world_from_rigid_body, base_from_rigid_body);

  EXPECT_EQ(result.ParentFrame(), "mocap_world");
  EXPECT_EQ(result.ChildFrame(), "base_link");
  EXPECT_TRUE(result.Translation().isApprox(Eigen::Vector3d(1.0, -2.0, 0.5), 1.0e-12));
  EXPECT_TRUE(result.Rotation().isApprox(measured_rotation, 1.0e-12));
}

TEST(MocapPoseTransform, RemovesApprovedRigidBodyLeverArm)
{
  const auto normalized_world_from_world = RigidTransform::Create(
    "mocap_world", "world", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ()));
  const auto world_from_rigid_body = RigidTransform::Create(
    "world", "mocap_rigid_body", Eigen::Vector3d(2.0, 3.0, 1.0),
    QuaternionToRosXyzw(yaw));
  const auto base_from_rigid_body = RigidTransform::Create(
    "base_link", "mocap_rigid_body", Eigen::Vector3d(0.1, 0.0, 0.0),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});

  const auto result = MocapPoseToBasePose(
    normalized_world_from_world, world_from_rigid_body, base_from_rigid_body);

  EXPECT_TRUE(result.Translation().isApprox(Eigen::Vector3d(2.0, 2.9, 1.0), 1.0e-12));
  EXPECT_TRUE(result.Rotation().isApprox(yaw, 1.0e-12));
}

TEST(MocapPoseTransform, RejectsDisconnectedFrameRoles)
{
  const auto normalized_world_from_world = RigidTransform::Create(
    "mocap_world", "world", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto wrong_input = RigidTransform::Create(
    "another_world", "mocap_rigid_body", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto base_from_rigid_body = RigidTransform::Create(
    "base_link", "mocap_rigid_body", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});

  EXPECT_THROW(
    MocapPoseToBasePose(normalized_world_from_world, wrong_input, base_from_rigid_body),
    FrameContractViolation);
}

}  // namespace
}  // namespace mocap_localization_adapter
