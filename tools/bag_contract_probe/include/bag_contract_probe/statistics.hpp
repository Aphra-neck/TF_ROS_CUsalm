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

#ifndef BAG_CONTRACT_PROBE__STATISTICS_HPP_
#define BAG_CONTRACT_PROBE__STATISTICS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace bag_contract_probe
{

struct StampSeriesSummary
{
  std::size_t observations{0U};
  std::size_t valid_stamps{0U};
  std::size_t zero_or_invalid{0U};
  std::size_t duplicate{0U};
  std::size_t nonmonotonic{0U};
  std::size_t positive_gaps{0U};
  std::optional<std::int64_t> first_stamp_ns;
  std::optional<std::int64_t> last_stamp_ns;
  std::optional<double> rate_hz;
  std::optional<double> minimum_gap_sec;
  std::optional<double> p50_gap_sec;
  std::optional<double> p95_gap_sec;
  std::optional<double> p99_gap_sec;
  std::optional<double> maximum_gap_sec;
  std::optional<double> submillisecond_positive_gap_ratio;
};

StampSeriesSummary SummarizeStampSeries(
  const std::vector<std::int64_t> & stamps_ns);

struct EvidenceWindowCoverageSummary
{
  std::optional<double> start_gap_sec;
  std::optional<double> end_gap_sec;
  std::optional<double> covered_duration_sec;
};

EvidenceWindowCoverageSummary SummarizeEvidenceWindowCoverage(
  const std::vector<std::int64_t> & stamps_ns,
  std::int64_t evidence_first_stamp_ns,
  std::int64_t evidence_last_stamp_ns);

struct MissingStampInterval
{
  std::int64_t first_stamp_ns{0};
  std::int64_t last_stamp_ns{0};
  std::size_t missing_samples{0U};
};

struct StampPairingSummary
{
  std::size_t valid_raw_stamps{0U};
  std::size_t valid_shadow_stamps{0U};
  std::size_t matched{0U};
  std::size_t raw_without_shadow{0U};
  std::size_t shadow_without_raw{0U};
  std::optional<double> raw_coverage_ratio;
  std::vector<MissingStampInterval> missing_intervals;
  std::vector<std::int64_t> orphan_shadow_stamps_ns;
};

StampPairingSummary PairExactStampMultisets(
  const std::vector<std::int64_t> & raw_stamps_ns,
  const std::vector<std::int64_t> & shadow_stamps_ns);

struct CounterSeriesSummary
{
  std::size_t observations{0U};
  std::size_t epochs{0U};
  std::size_t resets{0U};
  std::optional<std::uint64_t> first;
  std::optional<std::uint64_t> last;
  std::optional<std::uint64_t> delta_without_reset;
  std::uint64_t accumulated_increase{0U};
};

enum class CounterSeriesState
{
  kClean,
  kPreexistingNonzero,
  kIncreased,
  kReset,
};

CounterSeriesSummary SummarizeCounterSeries(
  const std::vector<std::uint64_t> & values);
CounterSeriesState ClassifyCounterSeries(const CounterSeriesSummary & summary);

}  // namespace bag_contract_probe

#endif  // BAG_CONTRACT_PROBE__STATISTICS_HPP_
