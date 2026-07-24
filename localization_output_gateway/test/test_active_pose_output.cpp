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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_output_gateway/gateway_node.hpp"

#ifndef TEST_PROFILE
#error "TEST_PROFILE must name one immutable active profile"
#endif

namespace localization_output_gateway
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;
using mavros_msgs::msg::State;
using mavros_msgs::msg::TimesyncStatus;
using namespace std::chrono_literals;

class ActivePoseOutput : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

enum class EndpointIdentity
{
  kReported,
  kUnknown,
  kMismatchedGid,
  kIncorrect,
};

class ScriptedGateway : public LocalizationOutputGateway
{
public:
  explicit ScriptedGateway(
    const rclcpp::NodeOptions & options,
    const EndpointIdentity selected_endpoint_identity)
  : LocalizationOutputGateway(options),
    selected_endpoint_identity_(selected_endpoint_identity)
  {
  }

  void SetSelectedEndpointIdentity(const EndpointIdentity identity)
  {
    selected_endpoint_identity_ = identity;
  }

  void SetOutputEndpointIdentity(const EndpointIdentity identity)
  {
    output_endpoint_identity_ = identity;
  }

protected:
  TopicEndpointInfoList GetPublishersInfoByTopic(const std::string & topic) override
  {
    auto endpoints = LocalizationOutputGateway::GetPublishersInfoByTopic(topic);
    const EndpointIdentity * identity = nullptr;
    if (topic == "/localization/selected/pose") {
      identity = &selected_endpoint_identity_;
    } else if (topic == "/mavros/vision_pose/pose_cov") {
      identity = &output_endpoint_identity_;
    } else {
      return endpoints;
    }
    for (auto & endpoint : endpoints) {
      if (*identity == EndpointIdentity::kUnknown) {
        endpoint.node_name("_NODE_NAME_UNKNOWN_");
        endpoint.node_namespace("_NODE_NAMESPACE_UNKNOWN_");
      } else if (*identity == EndpointIdentity::kMismatchedGid) {
        auto gid = endpoint.endpoint_gid();
        gid.front() = static_cast<std::uint8_t>(gid.front() ^ 0xffU);
        endpoint.endpoint_gid(gid);
      } else if (*identity == EndpointIdentity::kIncorrect) {
        endpoint.node_name("incorrect_localization_source_selector");
        endpoint.node_namespace("/");
      }
    }
    return endpoints;
  }

private:
  EndpointIdentity selected_endpoint_identity_;
  EndpointIdentity output_endpoint_identity_{EndpointIdentity::kReported};
};

rclcpp::NodeOptions Options()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter(
          "contract_file", std::string(TEST_CONFIG_DIR) + "/" +
          TEST_PROFILE + ".contract.yaml"),
      });
  return options;
}

SelectedPoseCandidate Candidate(rclcpp::Node & source)
{
  const std::string profile = TEST_PROFILE;
  SelectedPoseCandidate message;
  message.header.stamp = source.get_clock()->now();
  message.header.frame_id = "map";
  message.semantic_child_frame = "base_link";
  message.pose.position.x = 1.25;
  message.pose.position.y = -0.50;
  message.pose.position.z = 0.75;
  message.pose.orientation.z = 0.25881904510252074;
  message.pose.orientation.w = 0.96592582628906831;
  message.localization_epoch_id = "test-localization-epoch";
  message.mode = profile;
  message.authorization = "selected_pose_candidate_only";
  if (profile == "cuvslam_primary") {
    message.selector_contract_id = "yopo_cuvslam_primary_selector_20260724_v1";
    message.source_contract_id = "d435i_fcu_cuvslam_shadow_20260723_v2";
  } else {
    message.selector_contract_id = "yopo_mocap_primary_selector_20260724_v1";
    message.source_contract_id = "droneyee207_mocap_shadow_20260722_v2";
  }
  return message;
}

bool SpinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  const std::function<void()> & tick = {},
  const std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (tick) {
      tick();
    }
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  executor.spin_some();
  return predicate();
}

bool DiagnosticHasValue(
  const std::vector<DiagnosticArray> & diagnostics,
  const std::string & key,
  const std::string & expected)
{
  return std::any_of(
    diagnostics.begin(), diagnostics.end(),
    [&key, &expected](const DiagnosticArray & array) {
      return std::any_of(
        array.status.begin(), array.status.end(),
        [&key, &expected](const auto & status) {
          if (status.name !=
          "/localization_output_gateway: localization output gateway")
          {
            return false;
          }
          return std::any_of(
            status.values.begin(), status.values.end(),
            [&key, &expected](const auto & value) {
              return value.key == key && value.value == expected;
            });
        });
    });
}

TEST_F(ActivePoseOutput, RecoversFromUnknownSelectedIdentityThenCopiesPose)
{
  auto gateway = std::make_shared<ScriptedGateway>(
    Options(), EndpointIdentity::kUnknown);
  auto selector = std::make_shared<rclcpp::Node>(
    "localization_source_selector", "/");
  auto mavros_sys = std::make_shared<rclcpp::Node>("sys", "/mavros");
  auto mavros_time = std::make_shared<rclcpp::Node>("time", "/mavros");
  auto mavros_vision = std::make_shared<rclcpp::Node>(
    "vision_pose", "/mavros");

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().durability_volatile();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().transient_local();
  const auto timesync_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort().durability_volatile();
  auto selected_publisher = selector->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", reliable_qos);
  auto state_publisher = mavros_sys->create_publisher<State>(
    "/mavros/state", state_qos);
  auto timesync_publisher = mavros_time->create_publisher<TimesyncStatus>(
    "/mavros/timesync_status", timesync_qos);

  std::vector<PoseWithCovarianceStamped> outputs;
  auto output_subscription =
    mavros_vision->create_subscription<PoseWithCovarianceStamped>(
    "/mavros/vision_pose/pose_cov", reliable_qos,
    [&outputs](const PoseWithCovarianceStamped::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::vector<DiagnosticArray> diagnostics;
  auto diagnostic_subscription = mavros_vision->create_subscription<DiagnosticArray>(
    "/diagnostics", reliable_qos,
    [&diagnostics](const DiagnosticArray::ConstSharedPtr message) {
      diagnostics.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gateway);
  executor.add_node(selector);
  executor.add_node(mavros_sys);
  executor.add_node(mavros_time);
  executor.add_node(mavros_vision);

  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {
        return selected_publisher->get_subscription_count() == 1U &&
        state_publisher->get_subscription_count() == 1U &&
        timesync_publisher->get_subscription_count() == 1U;
      }));

  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "reason_code",
          "WAITING_FOR_SELECTED_PUBLISHER_GRAPH_STABILITY");
      }));
  EXPECT_EQ(output_subscription->get_publisher_count(), 0U);
  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kReported);
  diagnostics.clear();

  State state;
  state.connected = true;
  state.armed = false;
  state.manual_input = true;
  state.mode = "OFFBOARD";
  state.system_status = 3U;
  TimesyncStatus timesync;
  timesync.remote_timestamp_ns = 1000000000ULL;
  timesync.observed_offset_ns = 1000LL;
  timesync.estimated_offset_ns = 1000LL;
  timesync.round_trip_time_ms = 1.0F;
  const auto publish_readiness = [&]() {
      state.header.stamp = mavros_sys->get_clock()->now();
      timesync.header.stamp = mavros_time->get_clock()->now();
      state_publisher->publish(state);
      timesync_publisher->publish(timesync);
    };

  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "reason_code", "WAITING_FOR_NON_OFFBOARD_STARTUP_MODE");
      },
      publish_readiness));
  EXPECT_EQ(output_subscription->get_publisher_count(), 0U);

  state.mode = "STABILIZED";
  state.armed = true;
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "reason_code", "WAITING_FOR_PX4_DISARMED_STARTUP");
      },
      publish_readiness));
  EXPECT_EQ(output_subscription->get_publisher_count(), 0U);

  state.armed = false;
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return output_subscription->get_publisher_count() == 1U;},
      publish_readiness));

  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kMismatchedGid);
  diagnostics.clear();
  selected_publisher->publish(Candidate(*selector));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "reason_code",
          "WAITING_FOR_SELECTED_PUBLISHER_GRAPH_STABILITY") &&
        DiagnosticHasValue(diagnostics, "state", "active_starting");
      }));
  EXPECT_TRUE(outputs.empty());
  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kReported);
  diagnostics.clear();

  const auto first_input = Candidate(*selector);
  selected_publisher->publish(first_input);
  ASSERT_TRUE(SpinUntil(executor, [&outputs]() {return outputs.size() == 1U;}));

  const auto & first_output = outputs.front();
  EXPECT_EQ(first_output.header.stamp.sec, first_input.header.stamp.sec);
  EXPECT_EQ(first_output.header.stamp.nanosec, first_input.header.stamp.nanosec);
  EXPECT_EQ(first_output.header.frame_id, first_input.header.frame_id);
  EXPECT_EQ(first_output.pose.pose.position.x, first_input.pose.position.x);
  EXPECT_EQ(first_output.pose.pose.position.y, first_input.pose.position.y);
  EXPECT_EQ(first_output.pose.pose.position.z, first_input.pose.position.z);
  EXPECT_EQ(first_output.pose.pose.orientation.x, first_input.pose.orientation.x);
  EXPECT_EQ(first_output.pose.pose.orientation.y, first_input.pose.orientation.y);
  EXPECT_EQ(first_output.pose.pose.orientation.z, first_input.pose.orientation.z);
  EXPECT_EQ(first_output.pose.pose.orientation.w, first_input.pose.orientation.w);
  EXPECT_TRUE(
    std::all_of(
      first_output.pose.covariance.begin(), first_output.pose.covariance.end(),
      [](const double value) {return std::isnan(value);}));

  diagnostics.clear();
  state.armed = true;
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "mavros_armed", "1");
      },
      [&]() {
        state.header.stamp = mavros_sys->get_clock()->now();
        state_publisher->publish(state);
      }));
  auto second_input = Candidate(*selector);
  second_input.pose.position.x = 2.50;
  selected_publisher->publish(second_input);
  ASSERT_TRUE(SpinUntil(executor, [&outputs]() {return outputs.size() == 2U;}));
  EXPECT_EQ(outputs.back().pose.pose.position.x, 2.50);
  EXPECT_EQ(output_subscription->get_publisher_count(), 1U);

  auto monitor = std::make_shared<rclcpp::Node>(
    std::string(TEST_PROFILE) + "_evidence_monitor");
  std::vector<PoseWithCovarianceStamped> monitored_outputs;
  auto monitor_subscription =
    monitor->create_subscription<PoseWithCovarianceStamped>(
    "/mavros/vision_pose/pose_cov", reliable_qos,
    [&monitored_outputs](const PoseWithCovarianceStamped::ConstSharedPtr message) {
      monitored_outputs.push_back(*message);
    });
  executor.add_node(monitor);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return monitor_subscription->get_publisher_count() == 1U;}));
  auto third_input = Candidate(*selector);
  third_input.pose.position.x = 3.50;
  selected_publisher->publish(third_input);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return outputs.size() == 3U && monitored_outputs.size() == 1U;}));

  const auto publishers = gateway->get_node_graph_interface()
    ->get_publisher_names_and_types_by_node(
    gateway->get_name(), gateway->get_namespace());
  EXPECT_EQ(publishers.count("/mavros/vision_pose/pose_cov"), 1U);
  EXPECT_EQ(publishers.count("/localization/odometry"), 0U);
  EXPECT_EQ(publishers.count("/state/odom"), 0U);
  EXPECT_EQ(publishers.count("/mavros/setpoint_raw/attitude"), 0U);

  state.connected = false;
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return output_subscription->get_publisher_count() == 0U;},
      [&]() {
        state.header.stamp = mavros_sys->get_clock()->now();
        state_publisher->publish(state);
      }));
  state.connected = true;
  state.armed = false;
  publish_readiness();
  selected_publisher->publish(Candidate(*selector));
  EXPECT_FALSE(
    SpinUntil(
      executor, [&outputs]() {return outputs.size() > 3U;}, {}, 250ms));

  (void)diagnostic_subscription;
}

TEST_F(ActivePoseOutput, IncorrectSelectedIdentityStillLatches)
{
  auto gateway = std::make_shared<ScriptedGateway>(
    Options(), EndpointIdentity::kIncorrect);
  auto selector = std::make_shared<rclcpp::Node>(
    "localization_source_selector", "/");
  auto observer = std::make_shared<rclcpp::Node>(
    std::string(TEST_PROFILE) + "_incorrect_identity_observer");
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().durability_volatile();
  auto selected_publisher = selector->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", reliable_qos);
  std::vector<DiagnosticArray> diagnostics;
  auto diagnostic_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics", reliable_qos,
    [&diagnostics](const DiagnosticArray::ConstSharedPtr message) {
      diagnostics.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gateway);
  executor.add_node(selector);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return selected_publisher->get_subscription_count() == 1U;}));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "reason_code", "SELECTED_PUBLISHER_AUTHORITY_MISMATCH") &&
        DiagnosticHasValue(diagnostics, "state", "latched_fault");
      }));

  diagnostics.clear();
  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kReported);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "state", "latched_fault");
      }));

  (void)diagnostic_subscription;
}

TEST_F(ActivePoseOutput, SelectedGraphMismatchAfterMessageBindingLatches)
{
  auto gateway = std::make_shared<ScriptedGateway>(
    Options(), EndpointIdentity::kReported);
  auto selector = std::make_shared<rclcpp::Node>(
    "localization_source_selector", "/");
  auto observer = std::make_shared<rclcpp::Node>(
    std::string(TEST_PROFILE) + "_bound_identity_observer");
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().durability_volatile();
  auto selected_publisher = selector->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", reliable_qos);
  std::vector<DiagnosticArray> diagnostics;
  auto diagnostic_subscription = observer->create_subscription<DiagnosticArray>(
    "/diagnostics", reliable_qos,
    [&diagnostics](const DiagnosticArray::ConstSharedPtr message) {
      diagnostics.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gateway);
  executor.add_node(selector);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return selected_publisher->get_subscription_count() == 1U;}));

  selected_publisher->publish(Candidate(*selector));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(
          diagnostics, "localization_epoch_id", "test-localization-epoch");
      }));

  diagnostics.clear();
  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kMismatchedGid);
  selected_publisher->publish(Candidate(*selector));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "state", "latched_fault") &&
        (DiagnosticHasValue(
          diagnostics, "reason_code",
          "SELECTED_PUBLISHER_AUTHORITY_MISMATCH") ||
        DiagnosticHasValue(
          diagnostics, "reason_code", "SELECTED_PUBLISHER_GID_CHANGED"));
      }));

  diagnostics.clear();
  gateway->SetSelectedEndpointIdentity(EndpointIdentity::kReported);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "state", "latched_fault");
      }));

  (void)diagnostic_subscription;
}

TEST_F(ActivePoseOutput, OutputGraphMismatchAfterInitialValidationLatches)
{
  auto gateway = std::make_shared<ScriptedGateway>(
    Options(), EndpointIdentity::kReported);
  auto selector = std::make_shared<rclcpp::Node>(
    "localization_source_selector", "/");
  auto mavros_sys = std::make_shared<rclcpp::Node>("sys", "/mavros");
  auto mavros_time = std::make_shared<rclcpp::Node>("time", "/mavros");
  auto mavros_vision = std::make_shared<rclcpp::Node>(
    "vision_pose", "/mavros");

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().durability_volatile();
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable().transient_local();
  const auto timesync_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort().durability_volatile();
  auto selected_publisher = selector->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", reliable_qos);
  auto state_publisher = mavros_sys->create_publisher<State>(
    "/mavros/state", state_qos);
  auto timesync_publisher = mavros_time->create_publisher<TimesyncStatus>(
    "/mavros/timesync_status", timesync_qos);
  auto output_subscription =
    mavros_vision->create_subscription<PoseWithCovarianceStamped>(
    "/mavros/vision_pose/pose_cov", reliable_qos,
    [](const PoseWithCovarianceStamped::ConstSharedPtr) {});
  std::vector<DiagnosticArray> diagnostics;
  auto diagnostic_subscription = mavros_vision->create_subscription<DiagnosticArray>(
    "/diagnostics", reliable_qos,
    [&diagnostics](const DiagnosticArray::ConstSharedPtr message) {
      diagnostics.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gateway);
  executor.add_node(selector);
  executor.add_node(mavros_sys);
  executor.add_node(mavros_time);
  executor.add_node(mavros_vision);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {
        return selected_publisher->get_subscription_count() == 1U &&
        state_publisher->get_subscription_count() == 1U &&
        timesync_publisher->get_subscription_count() == 1U;
      }));

  State state;
  state.connected = true;
  state.armed = false;
  state.manual_input = true;
  state.mode = "STABILIZED";
  state.system_status = 3U;
  TimesyncStatus timesync;
  timesync.remote_timestamp_ns = 1000000000ULL;
  timesync.observed_offset_ns = 1000LL;
  timesync.estimated_offset_ns = 1000LL;
  timesync.round_trip_time_ms = 1.0F;
  const auto publish_readiness = [&]() {
      state.header.stamp = mavros_sys->get_clock()->now();
      timesync.header.stamp = mavros_time->get_clock()->now();
      state_publisher->publish(state);
      timesync_publisher->publish(timesync);
    };

  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {
        return output_subscription->get_publisher_count() == 1U &&
        DiagnosticHasValue(diagnostics, "output_publisher_gid_valid", "1");
      },
      publish_readiness));

  diagnostics.clear();
  gateway->SetOutputEndpointIdentity(EndpointIdentity::kMismatchedGid);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "state", "latched_fault") &&
        DiagnosticHasValue(
          diagnostics, "reason_code",
          "EXTERNAL_VISION_PUBLISHER_AUTHORITY_MISMATCH");
      }));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&]() {return output_subscription->get_publisher_count() == 0U;}));

  diagnostics.clear();
  gateway->SetOutputEndpointIdentity(EndpointIdentity::kReported);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&diagnostics]() {
        return DiagnosticHasValue(diagnostics, "state", "latched_fault");
      }));
  EXPECT_EQ(output_subscription->get_publisher_count(), 0U);

  (void)diagnostic_subscription;
}

}  // namespace
}  // namespace localization_output_gateway
