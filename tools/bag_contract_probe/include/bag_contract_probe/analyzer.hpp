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

#ifndef BAG_CONTRACT_PROBE__ANALYZER_HPP_
#define BAG_CONTRACT_PROBE__ANALYZER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace bag_contract_probe
{

class EvidenceContractError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

enum class RuntimeContractVerdict
{
  kPass,
  kReviewRequired,
  kFail,
};

struct AnalysisRequest
{
  std::filesystem::path bag_uri;
  std::filesystem::path report_directory;
};

struct AnalysisOutcome
{
  RuntimeContractVerdict runtime_contract_verdict;
  std::uint64_t messages_read{0U};
  std::size_t finding_count{0U};
  std::filesystem::path summary_path;
  std::filesystem::path topic_statistics_path;
  std::filesystem::path missing_intervals_path;
  std::filesystem::path orphan_shadows_path;
  std::filesystem::path diagnostic_statuses_path;
  std::filesystem::path diagnostic_counters_path;
  std::filesystem::path findings_path;
};

AnalysisOutcome AnalyzeBag(const AnalysisRequest & request);
std::string ToString(RuntimeContractVerdict verdict);

}  // namespace bag_contract_probe

#endif  // BAG_CONTRACT_PROBE__ANALYZER_HPP_
