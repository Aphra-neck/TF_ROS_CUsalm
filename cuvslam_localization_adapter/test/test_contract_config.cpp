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

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cuvslam_localization_adapter/contract_config.hpp"
#include "localization_contracts/gate.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

std::string Fixture(const std::string & name)
{
  return std::string(TEST_FIXTURE_DIR) + "/" + name;
}

std::string Config(const std::string & name)
{
  return std::string(TEST_CONFIG_DIR) + "/" + name;
}

TEST(ContractConfig, ProductionContractLocksApprovedAirframeExtrinsic)
{
  const ContractConfig contract =
    LoadContractConfig(Config("contract_blocked.yaml"));

  EXPECT_EQ(contract.contract_id, "d435i_fcu_cuvslam_shadow_20260723_v2");
  EXPECT_EQ(contract.status, "blocked");
  ASSERT_TRUE(contract.extrinsic.Approved());
  ASSERT_TRUE(contract.extrinsic.transform.has_value());
  ASSERT_TRUE(contract.extrinsic.provenance.has_value());
  EXPECT_EQ(contract.extrinsic.parent_frame, "base_link");
  EXPECT_EQ(contract.extrinsic.child_frame, "camera_link");
  EXPECT_EQ(
    contract.extrinsic.provenance.value(),
    "airframe_measurement/user_confirmed_20260723_camera_50mm_forward");

  const auto & transform = contract.extrinsic.transform.value();
  EXPECT_TRUE(transform.Translation().isApprox(Eigen::Vector3d(0.05, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(transform.RotationXyzw().x, 0.0);
  EXPECT_DOUBLE_EQ(transform.RotationXyzw().y, 0.0);
  EXPECT_DOUBLE_EQ(transform.RotationXyzw().z, 0.0);
  EXPECT_DOUBLE_EQ(transform.RotationXyzw().w, 1.0);

  EXPECT_FALSE(contract.twist.Approved());
  EXPECT_FALSE(contract.covariance.Approved());
  EXPECT_EQ(
    contract.authorization,
    localization_contracts::AuthorizationState::kShadowOnly);

  const auto decision = localization_contracts::EvaluateLocalizationPublishGate(
    "shadow",
    localization_contracts::HealthState::kHealthy,
    contract.authorization,
    contract.Approvals());
  EXPECT_FALSE(decision.publish);
  EXPECT_EQ(
    std::find(
      decision.reasons.begin(), decision.reasons.end(),
      "EXTRINSIC_UNAPPROVED"),
    decision.reasons.end());
  EXPECT_NE(
    std::find(
      decision.reasons.begin(), decision.reasons.end(),
      "AUTHORIZATION_SHADOW_ONLY"),
    decision.reasons.end());
  EXPECT_NE(
    std::find(
      decision.reasons.begin(), decision.reasons.end(),
      "TWIST_SEMANTICS_UNAPPROVED"),
    decision.reasons.end());
  EXPECT_NE(
    std::find(
      decision.reasons.begin(), decision.reasons.end(),
      "COVARIANCE_UNAPPROVED"),
    decision.reasons.end());
}

TEST(ContractConfig, ProductionExtrinsicConvertsCameraPoseWithCorrectSign)
{
  const ContractConfig contract =
    LoadContractConfig(Config("contract_blocked.yaml"));
  ASSERT_TRUE(contract.extrinsic.transform.has_value());

  const auto odom_from_camera = localization_contracts::RigidTransform::Create(
    "odom", "camera_link", Eigen::Vector3d(1.05, 2.0, 3.0),
    localization_contracts::QuaternionXyzw{0.0, 0.0, 0.0, 1.0});
  const auto odom_from_base = localization_contracts::CameraPoseToBasePose(
    odom_from_camera, contract.extrinsic.transform.value());

  EXPECT_EQ(odom_from_base.ParentFrame(), "odom");
  EXPECT_EQ(odom_from_base.ChildFrame(), "base_link");
  EXPECT_TRUE(
    odom_from_base.Translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(ContractConfig, LoadsSyntheticExtrinsicInShadowOnlyContract)
{
  const ContractConfig contract =
    LoadContractConfig(Fixture("approved_synthetic_contract.yaml"));
  EXPECT_EQ(contract.status, "blocked");
  EXPECT_TRUE(contract.extrinsic.Approved());
  EXPECT_FALSE(contract.twist.Approved());
  EXPECT_FALSE(contract.covariance.Approved());
  EXPECT_EQ(
    contract.authorization,
    localization_contracts::AuthorizationState::kShadowOnly);
  EXPECT_EQ(contract.output_topic, "/localization/odometry");
  EXPECT_EQ(contract.output_parent_frame, "odom");
  EXPECT_EQ(contract.output_child_frame, "base_link");
  ASSERT_EQ(contract.input.required_diagnostics.size(), 1U);
  EXPECT_EQ(
    contract.input.required_diagnostics.front().hardware_id,
    "synthetic-hardware");
  EXPECT_EQ(
    contract.input.required_diagnostics.front().expected_publisher,
    "/synthetic_diagnostic_source");
  EXPECT_EQ(contract.source_sha256.size(), 64U);
}

TEST(ContractConfig, RejectsBlockedPlaceholderExtrinsic)
{
  try {
    (void)LoadContractConfig(Fixture("blocked_placeholder_contract.yaml"));
    FAIL() << "blocked placeholder extrinsic was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(
      std::string(error.what()).find("placeholder transform"),
      std::string::npos);
  }
}

TEST(ContractConfig, RejectsDuplicateYamlKeys)
{
  try {
    (void)LoadContractConfig(Fixture("duplicate_key_contract.yaml"));
    FAIL() << "duplicate YAML key was accepted";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("duplicate"), std::string::npos);
  }
}

TEST(ContractConfig, ChecksDiagnosticIdentityValuesAndDuplicateKeys)
{
  const ContractConfig contract =
    LoadContractConfig(Fixture("approved_synthetic_contract.yaml"));
  const auto & required = contract.input.required_diagnostics.front();
  const std::vector<std::pair<std::string, std::string>> valid{
    {"state", "ready"},
    {"extra", "allowed"}};
  EXPECT_FALSE(
    CheckDiagnosticEvidence(
      required, 0U, "synthetic-hardware", valid).has_value());

  EXPECT_EQ(
    CheckDiagnosticEvidence(required, 1U, "synthetic-hardware", valid).value(),
    "UPSTREAM_DIAGNOSTIC_IDENTITY_OR_LEVEL_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(required, 0U, "wrong-hardware", valid).value(),
    "UPSTREAM_DIAGNOSTIC_IDENTITY_OR_LEVEL_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(
      required, 0U, "synthetic-hardware", {{"state", "not-ready"}}).value(),
    "UPSTREAM_DIAGNOSTIC_VALUE_MISMATCH");
  EXPECT_EQ(
    CheckDiagnosticEvidence(
      required,
      0U,
      "synthetic-hardware",
      {{"state", "ready"}, {"state", "ready"}}).value(),
    "UPSTREAM_DIAGNOSTIC_DUPLICATE_VALUE_KEY");
}

}  // namespace
}  // namespace cuvslam_localization_adapter
