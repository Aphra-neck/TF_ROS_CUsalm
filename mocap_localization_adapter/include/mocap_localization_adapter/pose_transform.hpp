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

#ifndef MOCAP_LOCALIZATION_ADAPTER__POSE_TRANSFORM_HPP_
#define MOCAP_LOCALIZATION_ADAPTER__POSE_TRANSFORM_HPP_

#include "localization_contracts/rigid_transform.hpp"

namespace mocap_localization_adapter
{

localization_contracts::RigidTransform MocapPoseToBasePose(
  const localization_contracts::RigidTransform & output_world_from_input_world,
  const localization_contracts::RigidTransform & input_world_from_rigid_body,
  const localization_contracts::RigidTransform & base_from_rigid_body);

}  // namespace mocap_localization_adapter

#endif  // MOCAP_LOCALIZATION_ADAPTER__POSE_TRANSFORM_HPP_
