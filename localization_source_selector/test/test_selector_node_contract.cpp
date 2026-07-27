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

#include "localization_source_selector/selector_node.hpp"

namespace localization_source_selector
{
namespace
{

class SelectorNodeContract : public ::testing::Test
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

rclcpp::NodeOptions Options(const std::string & mode, const std::string & contract)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", mode),
        rclcpp::Parameter("contract_file", contract),
      });
  return options;
}

TEST_F(SelectorNodeContract, CuvslamModeCreatesOnlyTheSelectedPoseSubscription)
{
  const std::string contract =
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml";
  auto node = std::make_shared<LocalizationSourceSelector>(
    Options("cuvslam_primary", contract));
  const auto subscriptions = node->get_node_graph_interface()
    ->get_subscriber_names_and_types_by_node(node->get_name(), node->get_namespace());
  EXPECT_EQ(subscriptions.count("/localization/candidates/cuvslam/base_pose"), 1U);
  EXPECT_EQ(subscriptions.count("/localization/candidates/mocap/base_pose"), 0U);
  EXPECT_EQ(subscriptions.count("/visual_slam/tracking/odometry"), 0U);
  EXPECT_EQ(subscriptions.count("/droneyee207/pose"), 0U);
}

TEST_F(SelectorNodeContract, MocapModeCreatesOnlyTheSelectedPoseSubscription)
{
  const std::string contract =
    std::string(TEST_CONFIG_DIR) + "/mocap_primary.contract.yaml";
  auto node = std::make_shared<LocalizationSourceSelector>(
    Options("mocap_primary", contract));
  const auto subscriptions = node->get_node_graph_interface()
    ->get_subscriber_names_and_types_by_node(node->get_name(), node->get_namespace());
  EXPECT_EQ(subscriptions.count("/localization/candidates/mocap/base_pose"), 1U);
  EXPECT_EQ(subscriptions.count("/localization/candidates/cuvslam/base_pose"), 0U);
  EXPECT_EQ(subscriptions.count("/visual_slam/tracking/odometry"), 0U);
  EXPECT_EQ(subscriptions.count("/droneyee207/pose"), 0U);
}

TEST_F(SelectorNodeContract, GraphContainsNoPrivilegedPublisher)
{
  const std::string contract =
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml";
  auto node = std::make_shared<LocalizationSourceSelector>(
    Options("cuvslam_primary", contract));
  const auto publishers = node->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(node->get_name(), node->get_namespace());

  const auto selected = publishers.find("/localization/selected/pose");
  ASSERT_NE(selected, publishers.end());
  ASSERT_EQ(selected->second.size(), 1U);
  EXPECT_EQ(
    selected->second.front(),
    "localization_adapter_interfaces/msg/SelectedPoseCandidate");
  EXPECT_EQ(publishers.count("/diagnostics"), 1U);
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/state/odom"), 0U);
  EXPECT_EQ(publishers.count("/mavros/vision_pose/pose_cov"), 0U);
  EXPECT_EQ(publishers.count("/tf"), 0U);
  EXPECT_EQ(publishers.count("/tf_static"), 0U);
  EXPECT_EQ(publishers.count("/yopo/control"), 0U);
}

TEST_F(SelectorNodeContract, RuntimeModeAndContractParametersAreReadOnly)
{
  const std::string contract =
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml";
  auto node = std::make_shared<LocalizationSourceSelector>(
    Options("cuvslam_primary", contract));
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("mode", "mocap_primary")).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("contract_file", "other.yaml")).successful);
}

TEST_F(SelectorNodeContract, RejectsUnknownModeContractMismatchAndTopicRemapping)
{
  const std::string cuvslam_contract =
    std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml";
  const std::string mocap_contract =
    std::string(TEST_CONFIG_DIR) + "/mocap_primary.contract.yaml";
  EXPECT_THROW(
    std::make_shared<LocalizationSourceSelector>(
      Options("automatic", cuvslam_contract)),
    std::runtime_error);
  EXPECT_THROW(
    std::make_shared<LocalizationSourceSelector>(
      Options("mocap_primary", cuvslam_contract)),
    std::runtime_error);

  auto remapped = Options("mocap_primary", mocap_contract);
  remapped.arguments(
    {"--ros-args", "--remap",
      "/localization/candidates/mocap/base_pose:=/localization/candidates/cuvslam/base_pose"});
  EXPECT_THROW(
    std::make_shared<LocalizationSourceSelector>(remapped),
    std::runtime_error);

  auto privileged = Options("cuvslam_primary", cuvslam_contract);
  privileged.arguments(
    {"--ros-args", "--remap",
      "/localization/selected/pose:=/mavros/vision_pose/pose_cov"});
  EXPECT_THROW(
    std::make_shared<LocalizationSourceSelector>(privileged),
    std::runtime_error);

  auto diagnostic_remap = Options("cuvslam_primary", cuvslam_contract);
  diagnostic_remap.arguments(
    {"--ros-args", "--remap", "/diagnostics:=/mavros/vision_pose/pose_cov"});
  EXPECT_THROW(
    std::make_shared<LocalizationSourceSelector>(diagnostic_remap),
    std::runtime_error);
}

}  // namespace
}  // namespace localization_source_selector
