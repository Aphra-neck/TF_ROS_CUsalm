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

#include "localization_source_selector/yaw_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace localization_source_selector
{
namespace
{

Eigen::Quaterniond QuaternionFromPose(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Quaterniond(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
}

Eigen::Vector3d PositionFromPose(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

bool Finite(const double value) noexcept
{
  return std::isfinite(value);
}

double Yaw(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Quaterniond q = quaternion.normalized();
  const double numerator = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double denominator = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(numerator, denominator);
}

}  // namespace

bool PoseIsFiniteAndNormalized(const geometry_msgs::msg::Pose & pose) noexcept
{
  if (!Finite(pose.position.x) || !Finite(pose.position.y) ||
    !Finite(pose.position.z) || !Finite(pose.orientation.x) ||
    !Finite(pose.orientation.y) || !Finite(pose.orientation.z) ||
    !Finite(pose.orientation.w))
  {
    return false;
  }
  const double squared_norm =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::abs(std::sqrt(squared_norm) - 1.0) <= 1.0e-3;
}

double PoseStepTranslation(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right) noexcept
{
  if (!PoseIsFiniteAndNormalized(left) || !PoseIsFiniteAndNormalized(right)) {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = right.position.x - left.position.x;
  const double dy = right.position.y - left.position.y;
  const double dz = right.position.z - left.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PoseStepRotation(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right) noexcept
{
  if (!PoseIsFiniteAndNormalized(left) || !PoseIsFiniteAndNormalized(right)) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::Quaterniond left_q = QuaternionFromPose(left).normalized();
  const Eigen::Quaterniond right_q = QuaternionFromPose(right).normalized();
  const double dot = std::clamp(std::abs(left_q.dot(right_q)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

YawAlignment YawAlignment::Identity()
{
  geometry_msgs::msg::Pose identity_pose;
  identity_pose.orientation.w = 1.0;
  return YawAlignment(identity_pose);
}

YawAlignment::YawAlignment(const geometry_msgs::msg::Pose & initial_pose)
: yaw_map_from_source_(0.0),
  rotation_map_from_source_(Eigen::Quaterniond::Identity()),
  translation_map_from_source_(Eigen::Vector3d::Zero())
{
  if (!PoseIsFiniteAndNormalized(initial_pose)) {
    throw std::invalid_argument("initial alignment pose is non-finite or not normalized");
  }
  const Eigen::Quaterniond initial_orientation = QuaternionFromPose(initial_pose).normalized();
  yaw_map_from_source_ = -Yaw(initial_orientation);
  rotation_map_from_source_ = Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw_map_from_source_, Eigen::Vector3d::UnitZ()));
  translation_map_from_source_ =
    -(rotation_map_from_source_ * PositionFromPose(initial_pose));
}

geometry_msgs::msg::Pose YawAlignment::Transform(
  const geometry_msgs::msg::Pose & source_pose) const
{
  if (!PoseIsFiniteAndNormalized(source_pose)) {
    throw std::invalid_argument("source pose is non-finite or not normalized");
  }
  const Eigen::Vector3d output_position =
    rotation_map_from_source_ * PositionFromPose(source_pose) +
    translation_map_from_source_;
  const Eigen::Quaterniond output_orientation =
    (rotation_map_from_source_ * QuaternionFromPose(source_pose).normalized()).normalized();

  geometry_msgs::msg::Pose output;
  output.position.x = output_position.x();
  output.position.y = output_position.y();
  output.position.z = output_position.z();
  output.orientation.x = output_orientation.x();
  output.orientation.y = output_orientation.y();
  output.orientation.z = output_orientation.z();
  output.orientation.w = output_orientation.w();
  return output;
}

double YawAlignment::YawMapFromSource() const noexcept
{
  return yaw_map_from_source_;
}

const Eigen::Vector3d & YawAlignment::TranslationMapFromSource() const noexcept
{
  return translation_map_from_source_;
}

}  // namespace localization_source_selector
