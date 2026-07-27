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
#include <vector>

#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mocap_localization_adapter/adapter_node.hpp"

namespace mocap_localization_adapter
{
namespace
{

rmw_qos_profile_t CompatibleShadowInputQos()
{
  rmw_qos_profile_t qos{};
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return qos;
}

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

TEST(ShadowInputQosContract, AcceptsKeepLastAndUnreportedHistoryOnly)
{
  auto qos = CompatibleShadowInputQos();
  EXPECT_TRUE(ShadowInputPublisherQosIsCompatible(qos));

  qos.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
  EXPECT_TRUE(ShadowInputPublisherQosIsCompatible(qos));

  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));
}

TEST(ShadowInputQosContract, KeepsReliabilityAndDurabilityStrict)
{
  auto qos = CompatibleShadowInputQos();
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));

  qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  EXPECT_FALSE(ShadowInputPublisherQosIsCompatible(qos));
}

TEST_F(ShadowNodeContract, GraphContainsOnlyUnprivilegedPoseAndDiagnosticsPublishers)
{
  auto node = std::make_shared<MocapLocalizationAdapter>();
  const auto publishers = node->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(node->get_name(), node->get_namespace());

  EXPECT_EQ(publishers.count("/localization/shadow/mocap/assumed_base_pose"), 1U);
  ASSERT_EQ(publishers.at("/localization/shadow/mocap/assumed_base_pose").size(), 1U);
  EXPECT_EQ(
    publishers.at("/localization/shadow/mocap/assumed_base_pose").front(),
    "localization_adapter_interfaces/msg/ShadowPoseCandidate");
  EXPECT_EQ(publishers.count("/localization/candidates/mocap/base_pose"), 1U);
  ASSERT_EQ(publishers.at("/localization/candidates/mocap/base_pose").size(), 1U);
  EXPECT_EQ(
    publishers.at("/localization/candidates/mocap/base_pose").front(),
    "localization_adapter_interfaces/msg/LocalizationSourceCandidate");
  EXPECT_EQ(publishers.count("/diagnostics"), 1U);
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/state/odom"), 0U);
  EXPECT_EQ(publishers.count("/localization/mavros_candidate"), 0U);
  EXPECT_EQ(publishers.count("/mavros/odometry/out"), 0U);
  EXPECT_EQ(publishers.count("/mavros/odometry/in"), 0U);
  EXPECT_EQ(publishers.count("/mavros/vision_pose/pose"), 0U);
  EXPECT_EQ(publishers.count("/mavros/mocap/pose"), 0U);
  EXPECT_EQ(publishers.count("/tf"), 0U);
  EXPECT_EQ(publishers.count("/tf_static"), 0U);
}

TEST_F(ShadowNodeContract, RejectsAnyNonShadowMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mode", "mocap_primary")});
  EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(options), std::runtime_error);
}

TEST_F(ShadowNodeContract, RejectsSimulationTime)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(options), std::runtime_error);
}

TEST_F(ShadowNodeContract, RejectsNonIdentityInstallationAssumption)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter(
          "base_from_rigid_body_translation_m",
          std::vector<double>{0.0, 0.0, 0.1})
      });
  EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(options), std::runtime_error);
}

TEST_F(ShadowNodeContract, RejectsWeakenedHealthContract)
{
  rclcpp::NodeOptions rate_options;
  rate_options.parameter_overrides(
      {
        rclcpp::Parameter("health.minimum_pose_rate_hz", 1.0)
      });
  EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(rate_options), std::runtime_error);

  rclcpp::NodeOptions window_options;
  window_options.parameter_overrides(
      {
        rclcpp::Parameter("health.pose_rate_window_samples", 1)
      });
  EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(window_options), std::runtime_error);
}

TEST_F(ShadowNodeContract, RejectsRemappingOfContractBoundTopics)
{
  for (const auto & remapping :
      {
        "/droneyee207/pose:=/untrusted/pose",
        "/localization/shadow/mocap/assumed_base_pose:=/localization/odometry",
        "/localization/shadow/mocap/assumed_base_pose:=/mavros/vision_pose/pose",
        "/localization/candidates/mocap/base_pose:=/localization/odometry",
        "/diagnostics:=/mavros/odometry/out"
      })
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--remap", remapping});
    EXPECT_THROW(std::make_shared<MocapLocalizationAdapter>(options), std::runtime_error);
  }
}

}  // namespace
}  // namespace mocap_localization_adapter
