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

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

#include <geometry_msgs/msg/pose.hpp>

#include "localization_source_selector/yaw_alignment.hpp"

namespace localization_source_selector
{
namespace
{

geometry_msgs::msg::Pose Pose(
  const double x,
  const double y,
  const double z,
  const double roll,
  const double pitch,
  const double yaw)
{
  const Eigen::Quaterniond quaternion =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double YawOf(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

TEST(YawAlignment, InitialPositionAndYawBecomeZeroWithoutRemovingRollPitch)
{
  constexpr double kRoll = 0.17;
  constexpr double kPitch = -0.11;
  constexpr double kYaw = 1.2;
  const auto initial = Pose(3.0, -2.0, 0.8, kRoll, kPitch, kYaw);
  const YawAlignment alignment(initial);
  const auto output = alignment.Transform(initial);

  EXPECT_NEAR(output.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(output.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(output.position.z, 0.0, 1.0e-12);
  EXPECT_NEAR(YawOf(output), 0.0, 1.0e-12);

  const Eigen::Quaterniond expected =
    Eigen::AngleAxisd(kPitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(kRoll, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond actual(
    output.orientation.w, output.orientation.x,
    output.orientation.y, output.orientation.z);
  EXPECT_NEAR(std::abs(expected.dot(actual)), 1.0, 1.0e-12);
}

TEST(YawAlignment, IdentityPreservesGlobalPositionAndOrientation)
{
  const auto source = Pose(2.0, 4.0, 0.5, 0.17, -0.11, 0.6);
  const auto output = YawAlignment::Identity().Transform(source);

  EXPECT_NEAR(output.position.x, source.position.x, 1.0e-12);
  EXPECT_NEAR(output.position.y, source.position.y, 1.0e-12);
  EXPECT_NEAR(output.position.z, source.position.z, 1.0e-12);
  const Eigen::Quaterniond expected(
    source.orientation.w, source.orientation.x,
    source.orientation.y, source.orientation.z);
  const Eigen::Quaterniond actual(
    output.orientation.w, output.orientation.x,
    output.orientation.y, output.orientation.z);
  EXPECT_NEAR(std::abs(expected.dot(actual)), 1.0, 1.0e-12);
  EXPECT_NEAR(YawAlignment::Identity().YawMapFromSource(), 0.0, 1.0e-12);
  EXPECT_NEAR(
    YawAlignment::Identity().TranslationMapFromSource().norm(), 0.0, 1.0e-12);
}

TEST(YawAlignment, LockedTransformMapsInitialHeadingToMapPositiveX)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  const auto initial = Pose(10.0, 5.0, 2.0, 0.0, 0.0, kHalfPi);
  const YawAlignment alignment(initial);
  const auto forward = Pose(10.0, 6.0, 2.0, 0.0, 0.0, kHalfPi);
  const auto output = alignment.Transform(forward);

  EXPECT_NEAR(output.position.x, 1.0, 1.0e-12);
  EXPECT_NEAR(output.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(output.position.z, 0.0, 1.0e-12);
  EXPECT_NEAR(YawOf(output), 0.0, 1.0e-12);
  EXPECT_NEAR(alignment.YawMapFromSource(), -kHalfPi, 1.0e-12);
}

TEST(YawAlignment, PreservesLeftUpAndSubsequentYawSemantics)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  const auto initial = Pose(10.0, 5.0, 2.0, 0.0, 0.0, kHalfPi);
  const YawAlignment alignment(initial);

  // At +90 degree initial yaw, body-left points toward source-world -x.
  const auto left = alignment.Transform(
    Pose(9.0, 5.0, 2.0, 0.0, 0.0, kHalfPi));
  EXPECT_NEAR(left.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(left.position.y, 1.0, 1.0e-12);
  EXPECT_NEAR(left.position.z, 0.0, 1.0e-12);

  const auto up = alignment.Transform(
    Pose(10.0, 5.0, 3.0, 0.0, 0.0, kHalfPi));
  EXPECT_NEAR(up.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(up.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(up.position.z, 1.0, 1.0e-12);

  const auto yawed = alignment.Transform(
    Pose(10.0, 5.0, 2.0, 0.0, 0.0, kHalfPi + 0.4));
  EXPECT_NEAR(YawOf(yawed), 0.4, 1.0e-12);
}

TEST(YawAlignment, PoseStepTreatsQuaternionSignAsTheSameRotation)
{
  auto left = Pose(0.0, 0.0, 0.0, 0.1, 0.2, 0.3);
  auto right = left;
  right.orientation.x *= -1.0;
  right.orientation.y *= -1.0;
  right.orientation.z *= -1.0;
  right.orientation.w *= -1.0;
  EXPECT_NEAR(PoseStepRotation(left, right), 0.0, 1.0e-12);
}

TEST(YawAlignment, FiniteExtremeInputCanOverflowTheAlignedPosition)
{
  const double maximum = std::numeric_limits<double>::max();
  const auto initial = Pose(maximum, maximum, 0.0, 0.0, 0.0, 0.7853981633974483);
  ASSERT_TRUE(PoseIsFiniteAndNormalized(initial));

  const YawAlignment alignment(initial);
  const auto output = alignment.Transform(initial);

  EXPECT_FALSE(PoseIsFiniteAndNormalized(output));
}

}  // namespace
}  // namespace localization_source_selector
