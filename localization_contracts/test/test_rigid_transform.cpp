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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <string>

#include "localization_contracts/errors.hpp"
#include "localization_contracts/rigid_transform.hpp"

namespace localization_contracts
{
namespace
{

constexpr double kTolerance = 1.0e-9;

QuaternionXyzw ToRos(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Quaterniond normalized = quaternion.normalized();
  return QuaternionXyzw{
    normalized.x(), normalized.y(), normalized.z(), normalized.w()};
}

TEST(RigidTransform, KnownCameraPoseRecoversBasePose)
{
  const double sine = std::sqrt(0.5);
  const auto odom_from_base = RigidTransform::Create(
    "odom", "base_link", Eigen::Vector3d(1.0, 2.0, 3.0),
    QuaternionXyzw{0.0, 0.0, sine, sine});
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d(0.2, -0.1, 0.05),
    QuaternionXyzw{sine, 0.0, 0.0, sine});

  const auto odom_from_camera = odom_from_base.Compose(base_from_camera);
  EXPECT_TRUE(
    odom_from_camera.Translation().isApprox(
      Eigen::Vector3d(1.1, 2.2, 3.05), kTolerance));
  const auto expected = RigidTransform::Create(
    "odom", "camera_link", Eigen::Vector3d(1.1, 2.2, 3.05),
    QuaternionXyzw{0.5, 0.5, 0.5, 0.5});
  EXPECT_TRUE(odom_from_camera.IsEquivalent(expected));

  const auto recovered = CameraPoseToBasePose(odom_from_camera, base_from_camera);
  EXPECT_TRUE(recovered.IsEquivalent(odom_from_base));
}

TEST(RigidTransform, ApplyPointIncludesRotationAndTranslation)
{
  const double sine = std::sqrt(0.5);
  const auto transform = RigidTransform::Create(
    "odom", "body", Eigen::Vector3d(1.0, 2.0, 3.0),
    QuaternionXyzw{0.0, 0.0, sine, sine});
  EXPECT_TRUE(
    transform.ApplyPoint(Eigen::Vector3d::UnitX()).isApprox(
      Eigen::Vector3d(1.0, 3.0, 3.0), kTolerance));
}

TEST(RigidTransform, InverseClosesTheFrameChain)
{
  const auto transform = RigidTransform::Create(
    "a", "b", Eigen::Vector3d(0.3, -0.4, 1.2),
    QuaternionXyzw{0.2, -0.1, 0.3, std::sqrt(0.86)});
  const auto identity = transform.Compose(transform.Inverse());
  EXPECT_EQ(identity.ParentFrame(), "a");
  EXPECT_EQ(identity.ChildFrame(), "a");
  EXPECT_TRUE(identity.Translation().isZero(kTolerance));
}

TEST(RigidTransform, QuaternionSignIsEquivalent)
{
  const auto first = RigidTransform::Create(
    "a", "b", Eigen::Vector3d(1.0, 2.0, 3.0),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto second = RigidTransform::Create(
    "a", "b", Eigen::Vector3d(1.0, 2.0, 3.0),
    QuaternionXyzw{0.0, 0.0, 0.0, -1.0});
  EXPECT_TRUE(first.IsEquivalent(second));
  EXPECT_THROW(first.IsEquivalent(second, -1.0, 1.0), NumericContractViolation);
  EXPECT_THROW(
    first.IsEquivalent(
      second, 1.0, std::numeric_limits<double>::infinity()),
    NumericContractViolation);
}

TEST(RigidTransform, ResolvesSmallAnglesForBothQuaternionSigns)
{
  constexpr double small_angle_rad = 1.0e-10;
  const auto reference = RigidTransform::Create(
    "a", "b", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const Eigen::Quaterniond small_rotation(
    Eigen::AngleAxisd(small_angle_rad, Eigen::Vector3d::UnitX()));
  const QuaternionXyzw small_rotation_xyzw = ToRos(small_rotation);
  const auto positive = RigidTransform::Create(
    "a", "b", Eigen::Vector3d::Zero(), small_rotation_xyzw);
  const auto negative = RigidTransform::Create(
    "a", "b", Eigen::Vector3d::Zero(),
    QuaternionXyzw{
        -small_rotation_xyzw.x,
        -small_rotation_xyzw.y,
        -small_rotation_xyzw.z,
        -small_rotation_xyzw.w});

  EXPECT_TRUE(reference.IsEquivalent(positive, 0.0, 2.0e-10));
  EXPECT_TRUE(reference.IsEquivalent(negative, 0.0, 2.0e-10));
  EXPECT_FALSE(reference.IsEquivalent(positive, 0.0, 5.0e-11));
  EXPECT_FALSE(reference.IsEquivalent(negative, 0.0, 5.0e-11));
  EXPECT_TRUE(positive.IsEquivalent(negative, 0.0, 0.0));
}

TEST(RigidTransform, RejectsDisconnectedAndInconsistentChains)
{
  const auto first = RigidTransform::Create(
    "a", "b", Eigen::Vector3d::Zero(), QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto disconnected = RigidTransform::Create(
    "c", "d", Eigen::Vector3d::Zero(), QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(first.Compose(disconnected), FrameContractViolation);

  const auto inconsistent_return = RigidTransform::Create(
    "b", "a", Eigen::Vector3d::UnitX(), QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(first.Compose(inconsistent_return), FrameContractViolation);
}

TEST(RigidTransform, RejectsAmbiguousFramesAndBadQuaternions)
{
  EXPECT_THROW(
    RigidTransform::Create(
      "/odom", "camera", Eigen::Vector3d::Zero(),
      QuaternionXyzw{0.0, 0.0, 0.0, 1.0}),
    FrameContractViolation);
  EXPECT_THROW(
    RigidTransform::Create(
      "odom", "camera", Eigen::Vector3d::Zero(),
      QuaternionXyzw{0.0, 0.0, 0.0, 0.0}),
    NumericContractViolation);
  EXPECT_THROW(
    RigidTransform::Create(
      "odom", "camera", Eigen::Vector3d::Zero(),
      QuaternionXyzw{0.0, 0.0, 0.0, 2.0}),
    NumericContractViolation);
}

TEST(RigidTransform, RejectsInvalidQuaternionSerialization)
{
  EXPECT_THROW(
    QuaternionToRosXyzw(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)),
    NumericContractViolation);
  EXPECT_THROW(
    QuaternionToRosXyzw(Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0)),
    NumericContractViolation);
}

TEST(RigidTransform, CameraPoseConversionRejectsWrongFrameRoles)
{
  const auto map_from_camera = RigidTransform::Create(
    "map", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto base_from_camera = RigidTransform::Create(
    "base_link", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(
    CameraPoseToBasePose(map_from_camera, base_from_camera),
    FrameContractViolation);

  const auto odom_from_camera = RigidTransform::Create(
    "odom", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto sensor_from_camera = RigidTransform::Create(
    "sensor_mount", "camera_link", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(
    CameraPoseToBasePose(odom_from_camera, sensor_from_camera),
    FrameContractViolation);
}

TEST(RigidTransform, NormalizesSmallQuaternionRoundoff)
{
  const auto transform = RigidTransform::Create(
    "a", "b", Eigen::Vector3d::Zero(),
    QuaternionXyzw{0.0, 0.0, 0.7070, 0.7070});
  EXPECT_NEAR(transform.Rotation().norm(), 1.0, 1.0e-15);
}

TEST(RigidTransform, RejectsNonfiniteDerivedResults)
{
  const double maximum = std::numeric_limits<double>::max();
  const auto first = RigidTransform::Create(
    "a", "b", Eigen::Vector3d(maximum, 0.0, 0.0),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto second = RigidTransform::Create(
    "b", "c", Eigen::Vector3d(maximum, 0.0, 0.0),
    QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  EXPECT_THROW(first.Compose(second), NumericContractViolation);
  EXPECT_THROW(
    first.ApplyPoint(Eigen::Vector3d(maximum, 0.0, 0.0)),
    NumericContractViolation);
}

TEST(RigidTransform, RandomAssociativityInverseAndPointRoundtrip)
{
  std::mt19937_64 generator(20260722U);
  std::uniform_real_distribution<double> position_distribution(-5.0, 5.0);
  std::uniform_real_distribution<double> axis_distribution(-1.0, 1.0);
  const double pi = std::acos(-1.0);
  std::uniform_real_distribution<double> angle_distribution(-pi, pi);
  auto random_transform = [&](const std::string & parent, const std::string & child) {
      Eigen::Vector3d axis(
        axis_distribution(generator),
        axis_distribution(generator),
        axis_distribution(generator));
      if (axis.norm() < 1.0e-6) {
        axis = Eigen::Vector3d::UnitX();
      }
      axis.normalize();
      const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(angle_distribution(generator), axis));
      return RigidTransform::Create(
        parent,
        child,
        Eigen::Vector3d(
          position_distribution(generator),
          position_distribution(generator),
          position_distribution(generator)),
        ToRos(rotation));
    };

  for (int iteration = 0; iteration < 500; ++iteration) {
    const auto ab = random_transform("a", "b");
    const auto bc = random_transform("b", "c");
    const auto cd = random_transform("c", "d");
    const auto left = ab.Compose(bc).Compose(cd);
    const auto right = ab.Compose(bc.Compose(cd));
    EXPECT_TRUE(left.IsEquivalent(right, 1.0e-8, 1.0e-8));

    const Eigen::Vector3d point(
      position_distribution(generator),
      position_distribution(generator),
      position_distribution(generator));
    const Eigen::Vector3d recovered = left.Inverse().ApplyPoint(left.ApplyPoint(point));
    EXPECT_TRUE(recovered.isApprox(point, 1.0e-8));
  }
}

}  // namespace
}  // namespace localization_contracts
