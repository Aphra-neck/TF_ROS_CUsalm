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

#include "localization_contracts/rigid_transform.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include "localization_contracts/errors.hpp"

namespace localization_contracts
{
namespace
{

constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr double kQuaternionZeroTolerance = 1.0e-12;

bool ContainsWhitespace(const std::string & value)
{
  return std::any_of(
    value.begin(), value.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;});
}

Eigen::Quaterniond ValidatedNormalizedQuaternion(const Eigen::Quaterniond & rotation)
{
  if (!rotation.coeffs().allFinite()) {
    throw NumericContractViolation("quaternion must contain only finite values");
  }
  const double norm = rotation.norm();
  if (norm <= kQuaternionZeroTolerance) {
    throw NumericContractViolation("quaternion norm is zero");
  }
  if (std::abs(norm - 1.0) > kQuaternionNormTolerance) {
    throw NumericContractViolation(
            "quaternion norm differs from one by more than the allowed tolerance");
  }
  return rotation.normalized();
}

}  // namespace

std::string ValidateFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    throw FrameContractViolation("frame_id must be non-empty");
  }
  if (frame_id.front() == '/') {
    throw FrameContractViolation("frame_id must not start with '/'");
  }
  if (ContainsWhitespace(frame_id)) {
    throw FrameContractViolation("frame_id must not contain whitespace");
  }
  if (frame_id.find("//") != std::string::npos) {
    throw FrameContractViolation("frame_id must not contain an empty path component");
  }
  return frame_id;
}

Eigen::Quaterniond QuaternionFromRosXyzw(const QuaternionXyzw & rotation_xyzw)
{
  return ValidatedNormalizedQuaternion(
    Eigen::Quaterniond(
      rotation_xyzw.w,
      rotation_xyzw.x,
      rotation_xyzw.y,
      rotation_xyzw.z));
}

QuaternionXyzw QuaternionToRosXyzw(const Eigen::Quaterniond & rotation)
{
  const Eigen::Quaterniond normalized = ValidatedNormalizedQuaternion(rotation);
  return QuaternionXyzw{
    normalized.x(), normalized.y(), normalized.z(), normalized.w()};
}

RigidTransform RigidTransform::Create(
  const std::string & parent_frame,
  const std::string & child_frame,
  const Eigen::Vector3d & translation,
  const QuaternionXyzw & rotation_xyzw)
{
  const std::string parent = ValidateFrameId(parent_frame);
  const std::string child = ValidateFrameId(child_frame);
  if (parent == child) {
    throw FrameContractViolation(
            "non-identity transform must use distinct parent and child frames");
  }
  if (!translation.allFinite()) {
    throw NumericContractViolation("translation must contain only finite values");
  }
  return RigidTransform(
    parent, child, translation, QuaternionFromRosXyzw(rotation_xyzw));
}

RigidTransform RigidTransform::Identity(const std::string & frame)
{
  const std::string validated_frame = ValidateFrameId(frame);
  return RigidTransform(
    validated_frame,
    validated_frame,
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity());
}

RigidTransform::RigidTransform(
  std::string parent_frame,
  std::string child_frame,
  Eigen::Vector3d translation,
  Eigen::Quaterniond rotation)
: parent_frame_(std::move(parent_frame)),
  child_frame_(std::move(child_frame)),
  translation_(std::move(translation)),
  rotation_(std::move(rotation))
{
}

RigidTransform RigidTransform::Compose(const RigidTransform & right) const
{
  if (child_frame_ != right.parent_frame_) {
    throw FrameContractViolation(
            "cannot compose disconnected transforms T[" + parent_frame_ + "," +
            child_frame_ + "] and T[" + right.parent_frame_ + "," +
            right.child_frame_ + "]");
  }
  const Eigen::Vector3d translation =
    translation_ + rotation_ * right.translation_;
  const Eigen::Quaterniond rotation = ValidatedNormalizedQuaternion(
    rotation_ * right.rotation_);
  if (!translation.allFinite()) {
    throw NumericContractViolation("composed transform translation is nonfinite");
  }
  if (parent_frame_ == right.child_frame_) {
    const double angular_error =
      2.0 * std::acos(std::clamp(std::abs(rotation.w()), 0.0, 1.0));
    if (translation.norm() > 1.0e-9 || angular_error > 1.0e-9) {
      throw FrameContractViolation(
              "closed transform chain is inconsistent with frame identity");
    }
    return Identity(parent_frame_);
  }
  return RigidTransform(
    parent_frame_, right.child_frame_, translation, rotation);
}

RigidTransform RigidTransform::Inverse() const
{
  const Eigen::Quaterniond inverse_rotation = rotation_.conjugate();
  const Eigen::Vector3d inverse_translation = -(inverse_rotation * translation_);
  if (!inverse_translation.allFinite()) {
    throw NumericContractViolation("inverse transform translation is nonfinite");
  }
  if (parent_frame_ == child_frame_) {
    return Identity(parent_frame_);
  }
  return RigidTransform(
    child_frame_, parent_frame_, inverse_translation, inverse_rotation);
}

Eigen::Vector3d RigidTransform::ApplyPoint(const Eigen::Vector3d & point_in_child) const
{
  if (!point_in_child.allFinite()) {
    throw NumericContractViolation("point must contain only finite values");
  }
  const Eigen::Vector3d point_in_parent = translation_ + rotation_ * point_in_child;
  if (!point_in_parent.allFinite()) {
    throw NumericContractViolation("transformed point is nonfinite");
  }
  return point_in_parent;
}

bool RigidTransform::IsEquivalent(
  const RigidTransform & other,
  const double translation_tolerance,
  const double angular_tolerance_rad) const
{
  if (!std::isfinite(translation_tolerance) || translation_tolerance < 0.0 ||
    !std::isfinite(angular_tolerance_rad) || angular_tolerance_rad < 0.0)
  {
    throw NumericContractViolation(
            "transform comparison tolerances must be finite and nonnegative");
  }
  if (parent_frame_ != other.parent_frame_ || child_frame_ != other.child_frame_) {
    return false;
  }
  if ((translation_ - other.translation_).norm() > translation_tolerance) {
    return false;
  }
  const Eigen::Quaterniond relative_rotation = rotation_.conjugate() * other.rotation_;
  const double angular_distance = 2.0 * std::atan2(
    relative_rotation.vec().norm(), std::abs(relative_rotation.w()));
  return angular_distance <= angular_tolerance_rad;
}

const std::string & RigidTransform::ParentFrame() const noexcept
{
  return parent_frame_;
}

const std::string & RigidTransform::ChildFrame() const noexcept
{
  return child_frame_;
}

const Eigen::Vector3d & RigidTransform::Translation() const noexcept
{
  return translation_;
}

const Eigen::Quaterniond & RigidTransform::Rotation() const noexcept
{
  return rotation_;
}

QuaternionXyzw RigidTransform::RotationXyzw() const
{
  return QuaternionToRosXyzw(rotation_);
}

RigidTransform CameraPoseToBasePose(
  const RigidTransform & odom_from_camera,
  const RigidTransform & base_from_camera)
{
  if (odom_from_camera.ParentFrame() != "odom" ||
    odom_from_camera.ChildFrame() != "camera_link")
  {
    throw FrameContractViolation(
            "camera pose must be exactly T[odom,camera_link]");
  }
  if (base_from_camera.ParentFrame() != "base_link" ||
    base_from_camera.ChildFrame() != "camera_link")
  {
    throw FrameContractViolation(
            "camera extrinsic must be exactly T[base_link,camera_link]");
  }
  return odom_from_camera.Compose(base_from_camera.Inverse());
}

}  // namespace localization_contracts
