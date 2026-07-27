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

#ifndef LOCALIZATION_SOURCE_SELECTOR__YAW_ALIGNMENT_HPP_
#define LOCALIZATION_SOURCE_SELECTOR__YAW_ALIGNMENT_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

namespace localization_source_selector
{

bool PoseIsFiniteAndNormalized(const geometry_msgs::msg::Pose & pose) noexcept;
double PoseStepTranslation(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right) noexcept;
double PoseStepRotation(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right) noexcept;

class YawAlignment final
{
public:
  static YawAlignment Identity();
  explicit YawAlignment(const geometry_msgs::msg::Pose & initial_pose);

  geometry_msgs::msg::Pose Transform(const geometry_msgs::msg::Pose & source_pose) const;
  double YawMapFromSource() const noexcept;
  const Eigen::Vector3d & TranslationMapFromSource() const noexcept;

private:
  double yaw_map_from_source_;
  Eigen::Quaterniond rotation_map_from_source_;
  Eigen::Vector3d translation_map_from_source_;
};

}  // namespace localization_source_selector

#endif  // LOCALIZATION_SOURCE_SELECTOR__YAW_ALIGNMENT_HPP_
