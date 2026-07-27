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

#ifndef LOCALIZATION_OUTPUT_GATEWAY__CONTRACT_CONFIG_HPP_
#define LOCALIZATION_OUTPUT_GATEWAY__CONTRACT_CONFIG_HPP_

#include <cstddef>
#include <string>

namespace localization_output_gateway
{

struct QosContractConfig
{
  std::string reliability;
  std::string durability;
  std::string history;
  std::size_t depth;
  bool allow_rmw_unknown_history;
};

struct InputContractConfig
{
  std::string topic;
  std::string type;
  std::string expected_publisher;
  std::string parent_frame;
  std::string semantic_child_frame;
  std::string authorization;
  std::string expected_mode;
  std::string expected_selector_contract_id;
  std::string expected_source_contract_id;
  QosContractConfig qos;
};

struct DiagnosticsContractConfig
{
  std::string topic;
  std::string type;
  std::string expected_publisher;
  std::string authorization;
};

struct OutputContractConfig
{
  std::string topic;
  std::string type;
  std::string authorization;
  std::string publisher_creation;
  QosContractConfig qos;
};

struct CanonicalOdometryContractConfig
{
  std::string topic;
  std::string authorization;
  std::string publisher_creation;
};

struct MavrosContractConfig
{
  std::string state_topic;
  std::string state_type;
  std::string expected_state_publisher;
  std::string timesync_topic;
  std::string timesync_type;
  std::string expected_timesync_publisher;
  std::string expected_external_vision_subscriber;
};

struct ContractConfig
{
  int schema_version;
  std::string gateway_contract_id;
  std::string profile;
  std::string expected_node_fqn;
  InputContractConfig input;
  DiagnosticsContractConfig diagnostics;
  OutputContractConfig external_vision_output;
  CanonicalOdometryContractConfig canonical_odometry;
  MavrosContractConfig mavros;

  bool IsActive() const noexcept;
};

ContractConfig LoadContractConfig(const std::string & path);

}  // namespace localization_output_gateway

#endif  // LOCALIZATION_OUTPUT_GATEWAY__CONTRACT_CONFIG_HPP_
