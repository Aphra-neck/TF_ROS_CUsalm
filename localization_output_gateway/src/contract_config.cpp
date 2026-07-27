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

#include "localization_output_gateway/contract_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace localization_output_gateway
{
namespace
{

constexpr char kDisabledGatewayContractId[] =
  "yopo_localization_output_gateway_disabled_20260724_v1";
constexpr char kCuvslamGatewayContractId[] =
  "yopo_cuvslam_primary_output_gateway_20260724_v1";
constexpr char kMocapGatewayContractId[] =
  "yopo_mocap_primary_output_gateway_20260727_v2";
constexpr char kSelectedPoseType[] =
  "localization_adapter_interfaces/msg/SelectedPoseCandidate";

void RequireExactKeys(
  const YAML::Node & node,
  const std::initializer_list<const char *> expected,
  const std::string & label)
{
  if (!node.IsMap()) {
    throw std::runtime_error(label + " must be a mapping");
  }
  std::set<std::string> expected_keys;
  for (const auto * key : expected) {
    expected_keys.insert(key);
  }
  std::set<std::string> actual_keys;
  for (const auto & entry : node) {
    if (!entry.first.IsScalar()) {
      throw std::runtime_error(label + " contains a non-scalar key");
    }
    const std::string key = entry.first.as<std::string>();
    if (!actual_keys.insert(key).second) {
      throw std::runtime_error(label + " contains duplicate key '" + key + "'");
    }
  }
  if (actual_keys != expected_keys) {
    throw std::runtime_error(label + " keys do not match the versioned schema");
  }
}

std::string NonemptyString(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be a scalar string");
  }
  const std::string value = node.as<std::string>();
  if (value.empty() || value.find_first_not_of(" \t\r\n") == std::string::npos ||
    value.front() == ' ' || value.back() == ' ')
  {
    throw std::runtime_error(label + " must be a non-empty trimmed string");
  }
  return value;
}

std::string AbsoluteRosName(const YAML::Node & node, const std::string & label)
{
  const std::string value = NonemptyString(node, label);
  if (value.size() < 2U || value.front() != '/' || value.find("//") != std::string::npos ||
    std::any_of(
      value.begin(), value.end(), [](const unsigned char character) {
        return std::isspace(character) != 0;
      }))
  {
    throw std::runtime_error(label + " must be a non-root absolute ROS name");
  }
  return value;
}

std::string FrameId(const YAML::Node & node, const std::string & label)
{
  const std::string value = NonemptyString(node, label);
  if (value.front() == '/' || value.find("//") != std::string::npos ||
    std::any_of(
      value.begin(), value.end(), [](const unsigned char character) {
        return std::isspace(character) != 0;
      }))
  {
    throw std::runtime_error(label + " must be a relative, whitespace-free frame id");
  }
  return value;
}

std::size_t PositiveSize(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be a positive integer");
  }
  const std::int64_t value = node.as<std::int64_t>();
  if (value <= 0) {
    throw std::runtime_error(label + " must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

void RequireEqual(
  const std::string & label,
  const std::string & actual,
  const std::string & expected)
{
  if (actual != expected) {
    throw std::runtime_error(label + " must be exactly '" + expected + "'");
  }
}

QosContractConfig ParseQos(const YAML::Node & node, const std::string & label)
{
  RequireExactKeys(
    node,
    {"allow_rmw_unknown_history", "depth", "durability", "history", "reliability"},
    label);
  QosContractConfig qos{
    NonemptyString(node["reliability"], label + ".reliability"),
    NonemptyString(node["durability"], label + ".durability"),
    NonemptyString(node["history"], label + ".history"),
    PositiveSize(node["depth"], label + ".depth"),
    node["allow_rmw_unknown_history"].as<bool>()};
  RequireEqual(label + ".reliability", qos.reliability, "reliable");
  RequireEqual(label + ".durability", qos.durability, "volatile");
  RequireEqual(label + ".history", qos.history, "keep_last");
  if (qos.depth != 10U || !qos.allow_rmw_unknown_history) {
    throw std::runtime_error(label + " does not match the versioned contract");
  }
  return qos;
}

InputContractConfig ParseInput(const YAML::Node & node, const bool active)
{
  if (active) {
    RequireExactKeys(
      node,
      {"authorization", "expected_mode", "expected_publisher",
        "expected_selector_contract_id", "expected_source_contract_id", "parent_frame",
        "qos", "semantic_child_frame", "topic", "type"},
      "input");
  } else {
    RequireExactKeys(
      node,
      {"authorization", "expected_publisher", "parent_frame", "qos",
        "semantic_child_frame", "topic", "type"},
      "input");
  }
  return InputContractConfig{
    AbsoluteRosName(node["topic"], "input.topic"),
    NonemptyString(node["type"], "input.type"),
    AbsoluteRosName(node["expected_publisher"], "input.expected_publisher"),
    FrameId(node["parent_frame"], "input.parent_frame"),
    FrameId(node["semantic_child_frame"], "input.semantic_child_frame"),
    NonemptyString(node["authorization"], "input.authorization"),
    active ? NonemptyString(node["expected_mode"], "input.expected_mode") : "",
    active ? NonemptyString(
      node["expected_selector_contract_id"], "input.expected_selector_contract_id") : "",
    active ? NonemptyString(
      node["expected_source_contract_id"], "input.expected_source_contract_id") : "",
    ParseQos(node["qos"], "input.qos")};
}

DiagnosticsContractConfig ParseDiagnostics(const YAML::Node & node)
{
  RequireExactKeys(
    node, {"authorization", "expected_publisher", "topic", "type"}, "diagnostics");
  return DiagnosticsContractConfig{
    AbsoluteRosName(node["topic"], "diagnostics.topic"),
    NonemptyString(node["type"], "diagnostics.type"),
    AbsoluteRosName(node["expected_publisher"], "diagnostics.expected_publisher"),
    NonemptyString(node["authorization"], "diagnostics.authorization")};
}

OutputContractConfig ParseExternalVisionOutput(const YAML::Node & node)
{
  RequireExactKeys(
    node, {"authorization", "publisher_creation", "qos", "topic", "type"},
    "external_vision_output");
  return OutputContractConfig{
    AbsoluteRosName(node["topic"], "external_vision_output.topic"),
    NonemptyString(node["type"], "external_vision_output.type"),
    NonemptyString(node["authorization"], "external_vision_output.authorization"),
    NonemptyString(node["publisher_creation"], "external_vision_output.publisher_creation"),
    ParseQos(node["qos"], "external_vision_output.qos")};
}

CanonicalOdometryContractConfig ParseCanonicalOdometry(const YAML::Node & node)
{
  RequireExactKeys(
    node, {"authorization", "publisher_creation", "topic"}, "canonical_odometry");
  return CanonicalOdometryContractConfig{
    AbsoluteRosName(node["topic"], "canonical_odometry.topic"),
    NonemptyString(node["authorization"], "canonical_odometry.authorization"),
    NonemptyString(node["publisher_creation"], "canonical_odometry.publisher_creation")};
}

MavrosContractConfig ParseMavros(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"expected_external_vision_subscriber", "expected_state_publisher",
      "expected_timesync_publisher", "state_topic", "state_type", "timesync_topic",
      "timesync_type"},
    "mavros");
  return MavrosContractConfig{
    AbsoluteRosName(node["state_topic"], "mavros.state_topic"),
    NonemptyString(node["state_type"], "mavros.state_type"),
    AbsoluteRosName(
      node["expected_state_publisher"],
      "mavros.expected_state_publisher"),
    AbsoluteRosName(node["timesync_topic"], "mavros.timesync_topic"),
    NonemptyString(node["timesync_type"], "mavros.timesync_type"),
    AbsoluteRosName(
      node["expected_timesync_publisher"],
      "mavros.expected_timesync_publisher"),
    AbsoluteRosName(
      node["expected_external_vision_subscriber"],
      "mavros.expected_external_vision_subscriber")};
}

void ValidateContract(const ContractConfig & contract)
{
  if (contract.schema_version != 1) {
    throw std::runtime_error("gateway contract schema_version must be exactly 1");
  }
  RequireEqual(
    "node.expected_fqn", contract.expected_node_fqn,
    "/localization_output_gateway");
  RequireEqual("input.topic", contract.input.topic, "/localization/selected/pose");
  RequireEqual("input.type", contract.input.type, kSelectedPoseType);
  RequireEqual(
    "input.expected_publisher", contract.input.expected_publisher,
    "/localization_source_selector");
  RequireEqual("input.parent_frame", contract.input.parent_frame, "map");
  RequireEqual(
    "input.semantic_child_frame", contract.input.semantic_child_frame,
    "base_link");
  RequireEqual(
    "input.authorization", contract.input.authorization,
    "selected_pose_candidate_only");
  RequireEqual("diagnostics.topic", contract.diagnostics.topic, "/diagnostics");
  RequireEqual(
    "diagnostics.type", contract.diagnostics.type,
    "diagnostic_msgs/msg/DiagnosticArray");
  RequireEqual(
    "diagnostics.expected_publisher", contract.diagnostics.expected_publisher,
    "/localization_output_gateway");
  RequireEqual(
    "diagnostics.authorization", contract.diagnostics.authorization,
    "diagnostics_only");
  RequireEqual(
    "external_vision_output.topic", contract.external_vision_output.topic,
    "/mavros/vision_pose/pose_cov");
  RequireEqual(
    "external_vision_output.type", contract.external_vision_output.type,
    "geometry_msgs/msg/PoseWithCovarianceStamped");
  RequireEqual(
    "canonical_odometry.topic", contract.canonical_odometry.topic,
    "/localization/odometry");
  RequireEqual(
    "canonical_odometry.authorization",
    contract.canonical_odometry.authorization, "deferred");
  RequireEqual(
    "canonical_odometry.publisher_creation",
    contract.canonical_odometry.publisher_creation, "forbidden");

  if (contract.profile == "disabled") {
    RequireEqual(
      "gateway_contract_id", contract.gateway_contract_id, kDisabledGatewayContractId);
    RequireEqual(
      "external_vision_output.authorization",
      contract.external_vision_output.authorization, "denied");
    RequireEqual(
      "external_vision_output.publisher_creation",
      contract.external_vision_output.publisher_creation, "forbidden");
    if (!contract.input.expected_mode.empty() ||
      !contract.input.expected_selector_contract_id.empty() ||
      !contract.input.expected_source_contract_id.empty())
    {
      throw std::runtime_error("disabled input must not contain an active mode tuple");
    }
    if (!contract.mavros.state_topic.empty()) {
      throw std::runtime_error("disabled contract must not contain MAVROS endpoints");
    }
    return;
  }

  RequireEqual(
    "external_vision_output.authorization",
    contract.external_vision_output.authorization, "explicit_pose_only");
  RequireEqual(
    "external_vision_output.publisher_creation",
    contract.external_vision_output.publisher_creation, "required");
  RequireEqual("mavros.state_topic", contract.mavros.state_topic, "/mavros/state");
  RequireEqual("mavros.state_type", contract.mavros.state_type, "mavros_msgs/msg/State");
  RequireEqual(
    "mavros.expected_state_publisher",
    contract.mavros.expected_state_publisher, "/mavros/sys");
  RequireEqual(
    "mavros.timesync_topic", contract.mavros.timesync_topic,
    "/mavros/timesync_status");
  RequireEqual(
    "mavros.timesync_type", contract.mavros.timesync_type,
    "mavros_msgs/msg/TimesyncStatus");
  RequireEqual(
    "mavros.expected_timesync_publisher",
    contract.mavros.expected_timesync_publisher, "/mavros/time");
  RequireEqual(
    "mavros.expected_external_vision_subscriber",
    contract.mavros.expected_external_vision_subscriber, "/mavros/vision_pose");
  if (contract.profile == "cuvslam_primary") {
    RequireEqual(
      "gateway_contract_id", contract.gateway_contract_id, kCuvslamGatewayContractId);
    RequireEqual("input.expected_mode", contract.input.expected_mode, "cuvslam_primary");
    RequireEqual(
      "input.expected_selector_contract_id",
      contract.input.expected_selector_contract_id,
      "yopo_cuvslam_primary_selector_20260724_v1");
    RequireEqual(
      "input.expected_source_contract_id",
      contract.input.expected_source_contract_id,
      "d435i_fcu_cuvslam_shadow_20260723_v2");
    return;
  }
  if (contract.profile == "mocap_primary") {
    RequireEqual(
      "gateway_contract_id", contract.gateway_contract_id, kMocapGatewayContractId);
    RequireEqual("input.expected_mode", contract.input.expected_mode, "mocap_primary");
    RequireEqual(
      "input.expected_selector_contract_id",
      contract.input.expected_selector_contract_id,
      "yopo_mocap_primary_selector_20260727_v2");
    RequireEqual(
      "input.expected_source_contract_id",
      contract.input.expected_source_contract_id,
      "droneyee207_mocap_shadow_20260722_v2");
    return;
  }
  throw std::runtime_error("profile must be disabled, cuvslam_primary, or mocap_primary");
}

}  // namespace

ContractConfig LoadContractConfig(const std::string & path)
{
  std::vector<YAML::Node> documents;
  try {
    documents = YAML::LoadAllFromFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "cannot parse gateway contract '" + path + "': " + error.what());
  }
  if (documents.size() != 1U) {
    throw std::runtime_error("gateway contract must contain exactly one YAML document");
  }
  const YAML::Node & root = documents.front();
  const std::string profile = NonemptyString(root["profile"], "profile");
  const bool active = profile == "cuvslam_primary" || profile == "mocap_primary";
  if (!active && profile != "disabled") {
    throw std::runtime_error("profile must be disabled, cuvslam_primary, or mocap_primary");
  }
  if (active) {
    RequireExactKeys(
      root,
      {"canonical_odometry", "diagnostics", "external_vision_output",
        "gateway_contract_id", "input", "mavros", "node", "profile", "schema_version"},
      "gateway contract");
  } else {
    RequireExactKeys(
      root,
      {"canonical_odometry", "diagnostics", "external_vision_output",
        "gateway_contract_id", "input", "node", "profile", "schema_version"},
      "gateway contract");
  }
  RequireExactKeys(root["node"], {"expected_fqn"}, "node");
  ContractConfig contract{
    root["schema_version"].as<int>(),
    NonemptyString(root["gateway_contract_id"], "gateway_contract_id"),
    profile,
    AbsoluteRosName(root["node"]["expected_fqn"], "node.expected_fqn"),
    ParseInput(root["input"], active),
    ParseDiagnostics(root["diagnostics"]),
    ParseExternalVisionOutput(root["external_vision_output"]),
    ParseCanonicalOdometry(root["canonical_odometry"]),
    active ? ParseMavros(root["mavros"]) : MavrosContractConfig{}};
  ValidateContract(contract);
  return contract;
}

bool ContractConfig::IsActive() const noexcept
{
  return profile == "cuvslam_primary" || profile == "mocap_primary";
}

}  // namespace localization_output_gateway
