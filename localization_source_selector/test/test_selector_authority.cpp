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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_source_selector/selector_node.hpp"

namespace localization_source_selector
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;
using namespace std::chrono_literals;

class SelectorAuthority : public ::testing::Test
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

enum class SelectedEndpointIdentity
{
  kReported,
  kUnknown,
  kMismatchedGid,
};

class ScriptedSelector : public LocalizationSourceSelector
{
public:
  explicit ScriptedSelector(
    const rclcpp::NodeOptions & options,
    const SelectedEndpointIdentity selected_endpoint_identity)
  : LocalizationSourceSelector(options),
    selected_endpoint_identity_(selected_endpoint_identity)
  {
  }

  void SetSelectedEndpointIdentity(const SelectedEndpointIdentity identity)
  {
    selected_endpoint_identity_ = identity;
  }

protected:
  TopicEndpointInfoList GetPublishersInfoByTopic(const std::string & topic) override
  {
    auto endpoints = LocalizationSourceSelector::GetPublishersInfoByTopic(topic);
    if (topic != "/localization/selected/pose") {
      return endpoints;
    }
    for (auto & endpoint : endpoints) {
      if (selected_endpoint_identity_ == SelectedEndpointIdentity::kUnknown) {
        endpoint.node_name() = "_NODE_NAME_UNKNOWN_";
        endpoint.node_namespace() = "_NODE_NAMESPACE_UNKNOWN_";
      } else if (selected_endpoint_identity_ == SelectedEndpointIdentity::kMismatchedGid) {
        auto gid = endpoint.endpoint_gid();
        gid.front() = static_cast<std::uint8_t>(gid.front() ^ 0xffU);
        endpoint.endpoint_gid() = gid;
      }
    }
    return endpoints;
  }

private:
  SelectedEndpointIdentity selected_endpoint_identity_;
};

rclcpp::NodeOptions TestOptions()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "cuvslam_primary"),
        rclcpp::Parameter(
          "contract_file",
          std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml"),
      });
  return options;
}

LocalizationSourceCandidate Candidate(
  rclcpp::Node & source,
  const double x = 3.0)
{
  LocalizationSourceCandidate message;
  message.header.stamp = source.get_clock()->now();
  message.header.frame_id = "odom";
  message.semantic_child_frame = "base_link";
  message.pose.position.x = x;
  message.pose.orientation.w = 1.0;
  message.source_id = "cuvslam";
  message.source_contract_id = "d435i_fcu_cuvslam_shadow_20260723_v2";
  message.authorization = "source_pose_candidate_only";
  return message;
}

void SpinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

template<typename Predicate>
bool SpinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  const std::chrono::milliseconds timeout = 5000ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  executor.spin_some();
  return predicate();
}

template<typename Predicate, typename Tick>
bool SpinUntilWithTick(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  Tick tick,
  const std::chrono::milliseconds timeout = 5000ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    tick();
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  executor.spin_some();
  return predicate();
}

rclcpp::Subscription<DiagnosticArray>::SharedPtr ObserveSelectorDiagnostics(
  rclcpp::Node & observer,
  std::optional<DiagnosticStatus> & latest_status)
{
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  return observer.create_subscription<DiagnosticArray>(
    "/diagnostics", qos,
    [&latest_status](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        if (status.name.find("localization source selector") != std::string::npos) {
          latest_status = status;
        }
      }
    });
}

std::string DiagnosticValue(
  const std::optional<DiagnosticStatus> & status,
  const std::string & key)
{
  if (!status.has_value()) {
    return "missing_status";
  }
  for (const auto & value : status->values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "missing_key";
}

bool LastOutputMatches(
  const std::vector<SelectedPoseCandidate> & outputs,
  const LocalizationSourceCandidate & input)
{
  return !outputs.empty() &&
         outputs.back().header.stamp.sec == input.header.stamp.sec &&
         outputs.back().header.stamp.nanosec == input.header.stamp.nanosec;
}

TEST_F(SelectorAuthority, SelectedOutputGraphConvergesBeforeFirstValidation)
{
  auto selector = std::make_shared<ScriptedSelector>(
    TestOptions(), SelectedEndpointIdentity::kUnknown);
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_output_convergence_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto output_subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector, &source_publisher]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 1U &&
        source_publisher->get_subscription_count() == 1U;
      }));

  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "starting" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "WAITING_FOR_SELECTED_OUTPUT_GRAPH";
      },
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  ASSERT_TRUE(outputs.empty());
  const auto accepted_with_unknown =
    std::stoull(DiagnosticValue(latest_status, "accepted"));

  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kMismatchedGid);
  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&latest_status, accepted_with_unknown]() {
        return DiagnosticValue(latest_status, "state") == "starting" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "WAITING_FOR_SELECTED_OUTPUT_GRAPH" &&
        std::stoull(DiagnosticValue(latest_status, "accepted")) >
        accepted_with_unknown;
      },
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  ASSERT_TRUE(outputs.empty());

  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kReported);
  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&outputs]() {return !outputs.empty();},
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  EXPECT_EQ(DiagnosticValue(latest_status, "latched"), "0");
  (void)diagnostic_subscription;
  (void)output_subscription;
}

TEST_F(SelectorAuthority, SelectedOutputUnknownIdentityAfterValidationLatches)
{
  auto selector = std::make_shared<ScriptedSelector>(
    TestOptions(), SelectedEndpointIdentity::kReported);
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_output_unknown_epoch_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto output_subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&outputs]() {return !outputs.empty();},
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  std::this_thread::sleep_for(1ms);
  const auto barrier = Candidate(*source, 3.1);
  source_publisher->publish(barrier);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&outputs, &barrier]() {return LastOutputMatches(outputs, barrier);}));
  const std::size_t published_before_change = outputs.size();

  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kUnknown);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "latched_fault" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "SELECTED_OUTPUT_AUTHORITY_MISMATCH";
      }));
  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kReported);
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(outputs.size(), published_before_change);
  (void)diagnostic_subscription;
  (void)output_subscription;
}

TEST_F(SelectorAuthority, SelectedOutputGidChangeAfterValidationLatches)
{
  auto selector = std::make_shared<ScriptedSelector>(
    TestOptions(), SelectedEndpointIdentity::kReported);
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_output_gid_epoch_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto output_subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&outputs]() {return !outputs.empty();},
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  std::this_thread::sleep_for(1ms);
  const auto barrier = Candidate(*source, 3.1);
  source_publisher->publish(barrier);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&outputs, &barrier]() {return LastOutputMatches(outputs, barrier);}));
  const std::size_t published_before_change = outputs.size();

  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kMismatchedGid);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "latched_fault" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "SELECTED_OUTPUT_PUBLISHER_GID_MISMATCH";
      }));
  selector->SetSelectedEndpointIdentity(SelectedEndpointIdentity::kReported);
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(outputs.size(), published_before_change);
  (void)diagnostic_subscription;
  (void)output_subscription;
}

TEST_F(SelectorAuthority, PersistentSelectedOutputUnknownIdentityTimesOut)
{
  auto selector = std::make_shared<ScriptedSelector>(
    TestOptions(), SelectedEndpointIdentity::kUnknown);
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_output_timeout_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntilWithTick(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "reason_code") ==
        "WAITING_FOR_SELECTED_OUTPUT_GRAPH";
      },
      [&source_publisher, &source]() {
        source_publisher->publish(Candidate(*source));
      }));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "latched_fault" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "SELECTED_OUTPUT_AUTHORITY_MISMATCH";
      },
      6000ms));
  (void)diagnostic_subscription;
}

TEST_F(SelectorAuthority, UnknownGraphIdentityDuringStartupWaitsForExpectedPublisher)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto unknown_source = std::make_shared<rclcpp::Node>("_NODE_NAME_UNKNOWN_");
  auto observer = std::make_shared<rclcpp::Node>("unknown_identity_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto unknown_publisher = unknown_source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto output_subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(unknown_source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector, &unknown_publisher]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 1U &&
        unknown_publisher->get_subscription_count() == 1U;
      }));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "starting" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "WAITING_FOR_SOURCE_PUBLISHER_GRAPH";
      }));
  const std::string received_before = DiagnosticValue(latest_status, "received");
  unknown_publisher->publish(Candidate(*unknown_source));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status, &received_before]() {
        return DiagnosticValue(latest_status, "received") != received_before;
      }));

  EXPECT_TRUE(outputs.empty());
  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "starting");
  EXPECT_EQ(
    DiagnosticValue(latest_status, "reason_code"),
    "WAITING_FOR_SOURCE_PUBLISHER_GRAPH");
  EXPECT_EQ(DiagnosticValue(latest_status, "bound_publisher_gid"), "unbound");
  EXPECT_EQ(DiagnosticValue(latest_status, "input_authority_violation"), "0");

  executor.remove_node(unknown_source);
  unknown_publisher.reset();
  unknown_source.reset();
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 0U;
      }));

  auto expected_source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto expected_publisher = expected_source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  executor.add_node(expected_source);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector, &expected_publisher]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 1U &&
        expected_publisher->get_subscription_count() == 1U;
      }));
  expected_publisher->publish(Candidate(*expected_source));
  ASSERT_TRUE(SpinUntil(executor, [&outputs]() {return outputs.size() == 1U;}));

  ASSERT_EQ(outputs.size(), 1U);
  (void)diagnostic_subscription;
  (void)output_subscription;
}

TEST_F(SelectorAuthority, ConcreteWrongPublisherIdentityLatchesUntilRestart)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto wrong_source = std::make_shared<rclcpp::Node>("wrong_cuvslam_adapter");
  auto observer = std::make_shared<rclcpp::Node>("wrong_identity_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto wrong_publisher = wrong_source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto output_subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});
  std::optional<DiagnosticStatus> latest_status;
  auto diagnostic_subscription = ObserveSelectorDiagnostics(*observer, latest_status);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(wrong_source);
  executor.add_node(observer);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector, &wrong_publisher]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 1U &&
        wrong_publisher->get_subscription_count() == 1U;
      }));
  wrong_publisher->publish(Candidate(*wrong_source));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status]() {
        return DiagnosticValue(latest_status, "state") == "latched_fault" &&
        DiagnosticValue(latest_status, "reason_code") ==
        "SOURCE_PUBLISHER_AUTHORITY_MISMATCH";
      }));

  ASSERT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  EXPECT_EQ(
    DiagnosticValue(latest_status, "reason_code"),
    "SOURCE_PUBLISHER_AUTHORITY_MISMATCH");
  EXPECT_EQ(output_count, 0U);

  executor.remove_node(wrong_source);
  wrong_publisher.reset();
  wrong_source.reset();
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 0U;
      }));

  auto expected_source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto expected_publisher = expected_source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  executor.add_node(expected_source);
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&selector, &expected_publisher]() {
        return selector->count_publishers(
          "/localization/candidates/cuvslam/base_pose") == 1U &&
        expected_publisher->get_subscription_count() == 1U;
      }));
  const std::string rejected_before = DiagnosticValue(latest_status, "rejected");
  expected_publisher->publish(Candidate(*expected_source));
  ASSERT_TRUE(
    SpinUntil(
      executor,
      [&latest_status, &rejected_before]() {
        return DiagnosticValue(latest_status, "rejected") != rejected_before;
      }));

  EXPECT_EQ(output_count, 0U);
  EXPECT_EQ(DiagnosticValue(latest_status, "state"), "latched_fault");
  (void)diagnostic_subscription;
  (void)output_subscription;
}

TEST_F(SelectorAuthority, OnlyTheLaunchSelectedSourceCanProduceSelectedOutput)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto unselected = std::make_shared<rclcpp::Node>("mocap_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_pose_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto unselected_publisher = unselected->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(unselected);
  executor.add_node(observer);

  // Exercise the graph-cache startup window before normal discovery settles.
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  const std::size_t after_early_candidate = outputs.size();

  auto wrong = Candidate(*unselected);
  wrong.source_id = "mocap";
  wrong.header.frame_id = "mocap_world";
  wrong.source_contract_id = "droneyee207_mocap_shadow_20260722_v2";
  unselected_publisher->publish(wrong);
  SpinFor(executor, 20ms);
  EXPECT_EQ(outputs.size(), after_early_candidate);

  const auto first = Candidate(*source, 3.0);
  source_publisher->publish(first);
  SpinFor(executor, 80ms);
  ASSERT_GT(outputs.size(), after_early_candidate);
  const auto & selected = outputs.back();
  EXPECT_EQ(selected.header.stamp.sec, first.header.stamp.sec);
  EXPECT_EQ(selected.header.stamp.nanosec, first.header.stamp.nanosec);
  EXPECT_EQ(selected.header.frame_id, "map");
  EXPECT_EQ(selected.semantic_child_frame, "base_link");
  EXPECT_EQ(selected.mode, "cuvslam_primary");
  EXPECT_EQ(selected.authorization, "selected_pose_candidate_only");
  EXPECT_FALSE(selected.localization_epoch_id.empty());
  EXPECT_NEAR(selected.pose.position.x, 0.0, 1.0e-9);
  (void)subscription;
}

TEST_F(SelectorAuthority, DuplicateInputPublishersLatchWithoutFallback)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto intruder = std::make_shared<rclcpp::Node>("candidate_intruder");
  auto observer = std::make_shared<rclcpp::Node>("duplicate_input_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto expected_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto duplicate_publisher = intruder->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(intruder);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  expected_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(output_count, 0U);

  duplicate_publisher.reset();
  executor.remove_node(intruder);
  intruder.reset();
  SpinFor(executor, 200ms);
  for (int index = 0; index < 12; ++index) {
    expected_publisher->publish(Candidate(*source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, 0U);
  (void)subscription;
}

TEST_F(SelectorAuthority, DuplicateSelectedOutputPublisherStopsSelectorOutput)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto intruder = std::make_shared<rclcpp::Node>("selected_output_intruder");
  auto observer = std::make_shared<rclcpp::Node>("duplicate_output_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto duplicate_output = intruder->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(intruder);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(output_count, 0U);

  duplicate_output.reset();
  executor.remove_node(intruder);
  intruder.reset();
  SpinFor(executor, 200ms);
  for (int index = 0; index < 12; ++index) {
    source_publisher->publish(Candidate(*source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, 0U);
  (void)subscription;
}

}  // namespace
}  // namespace localization_source_selector
