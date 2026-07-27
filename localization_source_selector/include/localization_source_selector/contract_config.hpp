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

#ifndef LOCALIZATION_SOURCE_SELECTOR__CONTRACT_CONFIG_HPP_
#define LOCALIZATION_SOURCE_SELECTOR__CONTRACT_CONFIG_HPP_

#include <cstddef>
#include <string>

namespace localization_source_selector
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
  std::string source_id;
  std::string source_contract_id;
  std::string parent_frame;
  std::string semantic_child_frame;
  std::string authorization;
  QosContractConfig qos;
};

struct OutputContractConfig
{
  std::string topic;
  std::string type;
  std::string expected_publisher;
  std::string parent_frame;
  std::string semantic_child_frame;
  std::string authorization;
  QosContractConfig qos;
};

struct HealthContractConfig
{
  double diagnostic_period_sec;
  double stale_after_sec;
  double maximum_clock_residual_sec;
  double maximum_position_step_m;
  double maximum_rotation_step_rad;
  std::size_t recovery_consecutive_samples;
};

struct ContractConfig
{
  int schema_version;
  std::string selector_contract_id;
  std::string mode;
  InputContractConfig input;
  OutputContractConfig output;
  HealthContractConfig health;
};

ContractConfig LoadContractConfig(const std::string & path);
void ValidateModeContract(const ContractConfig & contract, const std::string & requested_mode);

}  // namespace localization_source_selector

#endif  // LOCALIZATION_SOURCE_SELECTOR__CONTRACT_CONFIG_HPP_
