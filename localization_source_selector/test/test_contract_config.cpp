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

#include <stdexcept>
#include <string>

#include "localization_source_selector/contract_config.hpp"

namespace localization_source_selector
{
namespace
{

TEST(ContractConfig, LoadsBothImmutableProductionModes)
{
  const auto cuvslam = LoadContractConfig(
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml");
  EXPECT_EQ(cuvslam.schema_version, 1);
  EXPECT_EQ(cuvslam.mode, "cuvslam_primary");
  EXPECT_EQ(cuvslam.input.topic, "/localization/candidates/cuvslam/base_pose");
  EXPECT_EQ(cuvslam.input.expected_publisher, "/cuvslam_localization_adapter");
  EXPECT_EQ(cuvslam.input.authorization, "source_pose_candidate_only");
  EXPECT_EQ(cuvslam.output.topic, "/localization/selected/pose");
  EXPECT_EQ(cuvslam.output.authorization, "selected_pose_candidate_only");

  const auto mocap = LoadContractConfig(
    std::string(TEST_CONFIG_DIR) + "/mocap_primary.contract.yaml");
  EXPECT_EQ(mocap.mode, "mocap_primary");
  EXPECT_EQ(mocap.input.topic, "/localization/candidates/mocap/base_pose");
  EXPECT_EQ(mocap.input.expected_publisher, "/mocap_localization_adapter");
  EXPECT_EQ(mocap.input.source_id, "mocap");
}

TEST(ContractConfig, RequestedModeMustMatchTheVersionedContract)
{
  const auto contract = LoadContractConfig(
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml");
  EXPECT_NO_THROW(ValidateModeContract(contract, "cuvslam_primary"));
  EXPECT_THROW(ValidateModeContract(contract, "mocap_primary"), std::runtime_error);
  EXPECT_THROW(ValidateModeContract(contract, "automatic"), std::runtime_error);
}

TEST(ContractConfig, RejectsUnknownKeysAndModeTopicCrossWiring)
{
  EXPECT_THROW(
    LoadContractConfig(std::string(TEST_FIXTURE_DIR) + "/unknown_key.contract.yaml"),
    std::runtime_error);
  EXPECT_THROW(
    LoadContractConfig(std::string(TEST_FIXTURE_DIR) + "/mode_mismatch.contract.yaml"),
    std::runtime_error);
  EXPECT_THROW(
    LoadContractConfig(std::string(TEST_FIXTURE_DIR) + "/identity_mismatch.contract.yaml"),
    std::runtime_error);
}

TEST(ContractConfig, VersionedIdLocksQosAndEveryHealthThreshold)
{
  const std::string path =
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml";
  const auto expect_rejected = [&path](const auto & mutate) {
      auto contract = LoadContractConfig(path);
      mutate(contract);
      EXPECT_THROW(ValidateModeContract(contract, "cuvslam_primary"), std::runtime_error);
    };

  expect_rejected([](auto & contract) {contract.input.qos.depth = 11U;});
  expect_rejected([](auto & contract) {contract.input.qos.allow_rmw_unknown_history = false;});
  expect_rejected([](auto & contract) {contract.output.qos.depth = 11U;});
  expect_rejected([](auto & contract) {contract.output.qos.allow_rmw_unknown_history = false;});
  expect_rejected([](auto & contract) {contract.health.diagnostic_period_sec = 0.04;});
  expect_rejected([](auto & contract) {contract.health.stale_after_sec = 0.30;});
  expect_rejected([](auto & contract) {contract.health.maximum_clock_residual_sec = 0.30;});
  expect_rejected([](auto & contract) {contract.health.maximum_position_step_m = 0.60;});
  expect_rejected([](auto & contract) {contract.health.maximum_rotation_step_rad = 1.0;});
  expect_rejected([](auto & contract) {contract.health.recovery_consecutive_samples = 11U;});
}

}  // namespace
}  // namespace localization_source_selector
