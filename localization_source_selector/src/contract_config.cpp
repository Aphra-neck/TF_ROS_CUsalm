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

#include "localization_source_selector/contract_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace localization_source_selector
{
namespace
{

constexpr double kDegreesToRadians = 0.017453292519943295769;
constexpr char kCandidateType[] =
  "localization_adapter_interfaces/msg/LocalizationSourceCandidate";
constexpr char kSelectedType[] =
  "localization_adapter_interfaces/msg/SelectedPoseCandidate";
constexpr char kSourceAuthorization[] = "source_pose_candidate_only";
constexpr char kSelectedAuthorization[] = "selected_pose_candidate_only";
constexpr std::size_t kContractQosDepth = 10U;
constexpr double kDiagnosticPeriodSec = 0.05;
constexpr double kStaleAfterSec = 0.25;
constexpr double kMaximumClockResidualSec = 0.25;
constexpr double kMaximumPositionStepM = 0.50;
constexpr double kMaximumRotationStepRad = 45.0 * kDegreesToRadians;
constexpr std::size_t kRecoveryConsecutiveSamples = 10U;

void RequireMapping(const YAML::Node & node, const std::string & label)
{
  if (!node.IsMap()) {
    throw std::runtime_error(label + " must be a mapping");
  }
}

void RequireExactKeys(
  const YAML::Node & node,
  const std::set<std::string> & expected,
  const std::string & label)
{
  RequireMapping(node, label);
  std::set<std::string> actual;
  for (const auto & item : node) {
    if (!item.first.IsScalar()) {
      throw std::runtime_error(label + " contains a non-scalar key");
    }
    const std::string key = item.first.as<std::string>();
    if (!actual.insert(key).second) {
      throw std::runtime_error(label + " contains duplicate key '" + key + "'");
    }
  }
  if (actual != expected) {
    std::ostringstream message;
    message << label << " keys mismatch";
    throw std::runtime_error(message.str());
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

double PositiveDouble(const YAML::Node & node, const std::string & label)
{
  if (!node.IsScalar()) {
    throw std::runtime_error(label + " must be numeric");
  }
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(label + " must be finite and positive");
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

template<typename T>
void RequireEqualValue(const std::string & label, const T & actual, const T & expected)
{
  if (actual != expected) {
    throw std::runtime_error(label + " does not match the versioned contract");
  }
}

void ValidateQosContract(const QosContractConfig & qos, const std::string & label)
{
  RequireEqualValue(label + ".depth", qos.depth, kContractQosDepth);
  RequireEqualValue(
    label + ".allow_rmw_unknown_history", qos.allow_rmw_unknown_history, true);
}

void ValidateHealthContract(const HealthContractConfig & health)
{
  RequireEqualValue(
    "health.diagnostic_period_sec", health.diagnostic_period_sec, kDiagnosticPeriodSec);
  RequireEqualValue("health.stale_after_sec", health.stale_after_sec, kStaleAfterSec);
  RequireEqualValue(
    "health.maximum_clock_residual_sec",
    health.maximum_clock_residual_sec,
    kMaximumClockResidualSec);
  RequireEqualValue(
    "health.maximum_position_step_m", health.maximum_position_step_m, kMaximumPositionStepM);
  RequireEqualValue(
    "health.maximum_rotation_step_deg",
    health.maximum_rotation_step_rad,
    kMaximumRotationStepRad);
  RequireEqualValue(
    "health.recovery_consecutive_samples",
    health.recovery_consecutive_samples,
    kRecoveryConsecutiveSamples);
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
  return qos;
}

InputContractConfig ParseInput(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"authorization", "expected_publisher", "parent_frame", "qos",
      "semantic_child_frame", "source_contract_id", "source_id", "topic", "type"},
    "input");
  return InputContractConfig{
    AbsoluteRosName(node["topic"], "input.topic"),
    NonemptyString(node["type"], "input.type"),
    AbsoluteRosName(node["expected_publisher"], "input.expected_publisher"),
    NonemptyString(node["source_id"], "input.source_id"),
    NonemptyString(node["source_contract_id"], "input.source_contract_id"),
    FrameId(node["parent_frame"], "input.parent_frame"),
    FrameId(node["semantic_child_frame"], "input.semantic_child_frame"),
    NonemptyString(node["authorization"], "input.authorization"),
    ParseQos(node["qos"], "input.qos")};
}

OutputContractConfig ParseOutput(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"authorization", "expected_publisher", "parent_frame", "qos",
      "semantic_child_frame", "topic", "type"},
    "output");
  return OutputContractConfig{
    AbsoluteRosName(node["topic"], "output.topic"),
    NonemptyString(node["type"], "output.type"),
    AbsoluteRosName(node["expected_publisher"], "output.expected_publisher"),
    FrameId(node["parent_frame"], "output.parent_frame"),
    FrameId(node["semantic_child_frame"], "output.semantic_child_frame"),
    NonemptyString(node["authorization"], "output.authorization"),
    ParseQos(node["qos"], "output.qos")};
}

HealthContractConfig ParseHealth(const YAML::Node & node)
{
  RequireExactKeys(
    node,
    {"diagnostic_period_sec", "maximum_clock_residual_sec",
      "maximum_position_step_m", "maximum_rotation_step_deg",
      "recovery_consecutive_samples", "stale_after_sec"},
    "health");
  HealthContractConfig health{
    PositiveDouble(node["diagnostic_period_sec"], "health.diagnostic_period_sec"),
    PositiveDouble(node["stale_after_sec"], "health.stale_after_sec"),
    PositiveDouble(
      node["maximum_clock_residual_sec"], "health.maximum_clock_residual_sec"),
    PositiveDouble(node["maximum_position_step_m"], "health.maximum_position_step_m"),
    PositiveDouble(
      node["maximum_rotation_step_deg"], "health.maximum_rotation_step_deg") *
    kDegreesToRadians,
    PositiveSize(
      node["recovery_consecutive_samples"], "health.recovery_consecutive_samples")};
  if (health.diagnostic_period_sec >= health.stale_after_sec) {
    throw std::runtime_error("diagnostic_period_sec must be smaller than stale_after_sec");
  }
  if (health.maximum_rotation_step_rad > 3.14159265358979323846) {
    throw std::runtime_error("maximum_rotation_step_deg must not exceed 180 degrees");
  }
  return health;
}

}  // namespace

ContractConfig LoadContractConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "cannot parse selector contract '" + path + "': " + error.what());
  }
  RequireExactKeys(
    root,
    {"health", "input", "mode", "output", "schema_version", "selector_contract_id"},
    "selector contract");
  const int schema_version = root["schema_version"].as<int>();
  if (schema_version != 1) {
    throw std::runtime_error("selector contract schema_version must be exactly 1");
  }
  ContractConfig contract{
    schema_version,
    NonemptyString(root["selector_contract_id"], "selector_contract_id"),
    NonemptyString(root["mode"], "mode"),
    ParseInput(root["input"]),
    ParseOutput(root["output"]),
    ParseHealth(root["health"])};
  ValidateModeContract(contract, contract.mode);
  return contract;
}

void ValidateModeContract(const ContractConfig & contract, const std::string & requested_mode)
{
  if (requested_mode != "cuvslam_primary" && requested_mode != "mocap_primary") {
    throw std::runtime_error(
            "mode must be exactly 'cuvslam_primary' or 'mocap_primary'");
  }
  RequireEqual("contract mode", contract.mode, requested_mode);
  RequireEqual("input.type", contract.input.type, kCandidateType);
  RequireEqual("input.authorization", contract.input.authorization, kSourceAuthorization);
  RequireEqual("input.semantic_child_frame", contract.input.semantic_child_frame, "base_link");
  ValidateQosContract(contract.input.qos, "input.qos");

  if (requested_mode == "cuvslam_primary") {
    RequireEqual(
      "selector_contract_id", contract.selector_contract_id,
      "yopo_cuvslam_primary_selector_20260724_v1");
    RequireEqual(
      "input.topic", contract.input.topic,
      "/localization/candidates/cuvslam/base_pose");
    RequireEqual(
      "input.expected_publisher", contract.input.expected_publisher,
      "/cuvslam_localization_adapter");
    RequireEqual("input.source_id", contract.input.source_id, "cuvslam");
    RequireEqual(
      "input.source_contract_id", contract.input.source_contract_id,
      "d435i_fcu_cuvslam_shadow_20260723_v2");
    RequireEqual("input.parent_frame", contract.input.parent_frame, "odom");
  } else {
    RequireEqual(
      "selector_contract_id", contract.selector_contract_id,
      "yopo_mocap_primary_selector_20260727_v2");
    RequireEqual(
      "input.topic", contract.input.topic,
      "/localization/candidates/mocap/base_pose");
    RequireEqual(
      "input.expected_publisher", contract.input.expected_publisher,
      "/mocap_localization_adapter");
    RequireEqual("input.source_id", contract.input.source_id, "mocap");
    RequireEqual(
      "input.source_contract_id", contract.input.source_contract_id,
      "droneyee207_mocap_shadow_20260722_v2");
    RequireEqual("input.parent_frame", contract.input.parent_frame, "mocap_world");
  }

  RequireEqual("output.topic", contract.output.topic, "/localization/selected/pose");
  RequireEqual("output.type", contract.output.type, kSelectedType);
  RequireEqual(
    "output.expected_publisher", contract.output.expected_publisher,
    "/localization_source_selector");
  RequireEqual("output.parent_frame", contract.output.parent_frame, "map");
  RequireEqual("output.semantic_child_frame", contract.output.semantic_child_frame, "base_link");
  RequireEqual("output.authorization", contract.output.authorization, kSelectedAuthorization);
  ValidateQosContract(contract.output.qos, "output.qos");
  ValidateHealthContract(contract.health);
}

}  // namespace localization_source_selector
