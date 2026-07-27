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

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cuvslam_localization_adapter/adapter_node.hpp"

namespace cuvslam_localization_adapter
{
namespace
{

class ShadowNodeContract : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ShadowNodeContract, DefaultGraphHasOnlyUnprivilegedSourceCandidateOutput)
{
  const std::string contract =
    std::string(TEST_FIXTURE_DIR) + "/../../config/contract_blocked.yaml";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  auto node = std::make_shared<CuvslamLocalizationAdapter>(options);

  const auto publishers = node->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(node->get_name(), node->get_namespace());
  const auto candidate = publishers.find("/localization/candidates/cuvslam/base_pose");
  ASSERT_NE(candidate, publishers.end());
  ASSERT_EQ(candidate->second.size(), 1U);
  EXPECT_EQ(
    candidate->second.front(),
    "localization_adapter_interfaces/msg/LocalizationSourceCandidate");
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/state/odom"), 0U);
  EXPECT_EQ(publishers.count("/localization/mavros_candidate"), 0U);
  EXPECT_EQ(publishers.count("/mavros/odometry/out"), 0U);
  EXPECT_EQ(publishers.count("/mavros/vision_pose/pose_cov"), 0U);
  EXPECT_EQ(publishers.count("/tf"), 0U);
  EXPECT_EQ(publishers.count("/tf_static"), 0U);
  EXPECT_EQ(publishers.count("/diagnostics"), 1U);
}

TEST_F(ShadowNodeContract, RejectsAnyNonShadowRuntimeMode)
{
  const std::string contract =
    std::string(TEST_FIXTURE_DIR) + "/approved_synthetic_contract.yaml";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "passive"),
        rclcpp::Parameter("contract_file", contract),
      });
  EXPECT_THROW(
    std::make_shared<CuvslamLocalizationAdapter>(options),
    std::runtime_error);
}

TEST_F(ShadowNodeContract, ApprovedSyntheticExtrinsicStillCreatesNoPrivilegedPublisher)
{
  const std::string contract =
    std::string(TEST_FIXTURE_DIR) + "/approved_synthetic_contract.yaml";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  auto node = std::make_shared<CuvslamLocalizationAdapter>(options);

  const auto publishers = node->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(node->get_name(), node->get_namespace());
  EXPECT_EQ(publishers.count("/localization/candidates/cuvslam/base_pose"), 1U);
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/diagnostics"), 1U);
}

TEST_F(ShadowNodeContract, RejectsRemappingOfContractBoundTopics)
{
  const std::string contract =
    std::string(TEST_FIXTURE_DIR) + "/approved_synthetic_contract.yaml";
  rclcpp::NodeOptions options;
  options.arguments(
      {
        "--ros-args",
        "--remap",
        "/synthetic/odometry:=/untrusted/odometry",
      });
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  EXPECT_THROW(
    std::make_shared<CuvslamLocalizationAdapter>(options),
    std::runtime_error);

  rclcpp::NodeOptions diagnostics_options;
  diagnostics_options.arguments(
      {
        "--ros-args",
        "--remap",
        "/diagnostics:=/mavros/odometry/out",
      });
  diagnostics_options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  EXPECT_THROW(
    std::make_shared<CuvslamLocalizationAdapter>(diagnostics_options),
    std::runtime_error);

  rclcpp::NodeOptions candidate_options;
  candidate_options.arguments(
      {
        "--ros-args",
        "--remap",
        "/localization/candidates/cuvslam/base_pose:=/mavros/vision_pose/pose_cov",
      });
  candidate_options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "shadow"),
        rclcpp::Parameter("contract_file", contract),
      });
  EXPECT_THROW(
    std::make_shared<CuvslamLocalizationAdapter>(candidate_options),
    std::runtime_error);
}

}  // namespace
}  // namespace cuvslam_localization_adapter
