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

#include "bag_contract_probe/statistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bag_contract_probe
{
namespace
{

constexpr double kNanosecondsPerSecond = 1000000000.0;

double Percentile(const std::vector<std::int64_t> & sorted_values, const double fraction)
{
  const double index = fraction * static_cast<double>(sorted_values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double weight = index - static_cast<double>(lower);
  return static_cast<double>(sorted_values[lower]) * (1.0 - weight) +
         static_cast<double>(sorted_values[upper]) * weight;
}

void CloseMissingInterval(
  std::optional<MissingStampInterval> * current,
  std::vector<MissingStampInterval> * completed)
{
  if (current->has_value()) {
    completed->push_back(current->value());
    current->reset();
  }
}

std::uint64_t SaturatingAdd(const std::uint64_t left, const std::uint64_t right)
{
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return left + right;
}

}  // namespace

StampSeriesSummary SummarizeStampSeries(
  const std::vector<std::int64_t> & stamps_ns)
{
  StampSeriesSummary summary;
  summary.observations = stamps_ns.size();
  std::optional<std::int64_t> previous;
  std::vector<std::int64_t> positive_gaps;
  positive_gaps.reserve(stamps_ns.size());

  for (const std::int64_t stamp_ns : stamps_ns) {
    if (stamp_ns <= 0) {
      ++summary.zero_or_invalid;
      continue;
    }
    ++summary.valid_stamps;
    if (!summary.first_stamp_ns.has_value()) {
      summary.first_stamp_ns = stamp_ns;
    }
    summary.last_stamp_ns = stamp_ns;

    if (previous.has_value()) {
      if (stamp_ns == previous.value()) {
        ++summary.duplicate;
      } else if (stamp_ns < previous.value()) {
        ++summary.nonmonotonic;
      } else {
        positive_gaps.push_back(stamp_ns - previous.value());
      }
    }
    previous = stamp_ns;
  }

  summary.positive_gaps = positive_gaps.size();
  if (summary.duplicate == 0U && summary.nonmonotonic == 0U &&
    summary.valid_stamps >= 2U && summary.first_stamp_ns.has_value() &&
    summary.last_stamp_ns.value() > summary.first_stamp_ns.value())
  {
    const double duration_sec = static_cast<double>(
      summary.last_stamp_ns.value() - summary.first_stamp_ns.value()) /
      kNanosecondsPerSecond;
    summary.rate_hz = static_cast<double>(summary.valid_stamps - 1U) / duration_sec;
  }

  if (!positive_gaps.empty()) {
    const std::size_t submillisecond = static_cast<std::size_t>(std::count_if(
        positive_gaps.begin(), positive_gaps.end(),
        [](const std::int64_t gap_ns) {return gap_ns < 1000000LL;}));
    summary.submillisecond_positive_gap_ratio =
      static_cast<double>(submillisecond) /
      static_cast<double>(positive_gaps.size());
    std::sort(positive_gaps.begin(), positive_gaps.end());
    summary.minimum_gap_sec =
      static_cast<double>(positive_gaps.front()) / kNanosecondsPerSecond;
    summary.p50_gap_sec = Percentile(positive_gaps, 0.50) / kNanosecondsPerSecond;
    summary.p95_gap_sec = Percentile(positive_gaps, 0.95) / kNanosecondsPerSecond;
    summary.p99_gap_sec = Percentile(positive_gaps, 0.99) / kNanosecondsPerSecond;
    summary.maximum_gap_sec =
      static_cast<double>(positive_gaps.back()) / kNanosecondsPerSecond;
  }
  return summary;
}

EvidenceWindowCoverageSummary SummarizeEvidenceWindowCoverage(
  const std::vector<std::int64_t> & stamps_ns,
  const std::int64_t evidence_first_stamp_ns,
  const std::int64_t evidence_last_stamp_ns)
{
  EvidenceWindowCoverageSummary summary;
  if (evidence_first_stamp_ns <= 0 ||
    evidence_last_stamp_ns < evidence_first_stamp_ns)
  {
    return summary;
  }

  std::optional<std::int64_t> first;
  std::optional<std::int64_t> last;
  for (const std::int64_t stamp_ns : stamps_ns) {
    if (stamp_ns <= 0) {
      continue;
    }
    first = first.has_value() ? std::min(first.value(), stamp_ns) : stamp_ns;
    last = last.has_value() ? std::max(last.value(), stamp_ns) : stamp_ns;
  }
  if (!first.has_value()) {
    return summary;
  }

  summary.start_gap_sec = static_cast<double>(
    first.value() - evidence_first_stamp_ns) / kNanosecondsPerSecond;
  summary.end_gap_sec = static_cast<double>(
    evidence_last_stamp_ns - last.value()) / kNanosecondsPerSecond;
  summary.covered_duration_sec = static_cast<double>(
    last.value() - first.value()) / kNanosecondsPerSecond;
  return summary;
}

StampPairingSummary PairExactStampMultisets(
  const std::vector<std::int64_t> & raw_stamps_ns,
  const std::vector<std::int64_t> & shadow_stamps_ns)
{
  StampPairingSummary summary;
  std::unordered_map<std::int64_t, std::size_t> remaining_shadow;
  remaining_shadow.reserve(shadow_stamps_ns.size());
  for (const std::int64_t stamp_ns : shadow_stamps_ns) {
    if (stamp_ns <= 0) {
      continue;
    }
    ++summary.valid_shadow_stamps;
    ++remaining_shadow[stamp_ns];
  }

  std::optional<MissingStampInterval> current_interval;
  for (const std::int64_t stamp_ns : raw_stamps_ns) {
    if (stamp_ns <= 0) {
      continue;
    }
    ++summary.valid_raw_stamps;
    const auto shadow = remaining_shadow.find(stamp_ns);
    if (shadow != remaining_shadow.end() && shadow->second > 0U) {
      ++summary.matched;
      --shadow->second;
      CloseMissingInterval(&current_interval, &summary.missing_intervals);
      continue;
    }

    ++summary.raw_without_shadow;
    if (!current_interval.has_value()) {
      current_interval = MissingStampInterval{stamp_ns, stamp_ns, 1U};
    } else {
      current_interval->last_stamp_ns = stamp_ns;
      ++current_interval->missing_samples;
    }
  }
  CloseMissingInterval(&current_interval, &summary.missing_intervals);

  for (const auto & entry : remaining_shadow) {
    summary.shadow_without_raw += entry.second;
    summary.orphan_shadow_stamps_ns.insert(
      summary.orphan_shadow_stamps_ns.end(), entry.second, entry.first);
  }
  std::sort(
    summary.orphan_shadow_stamps_ns.begin(),
    summary.orphan_shadow_stamps_ns.end());
  if (summary.valid_raw_stamps > 0U) {
    summary.raw_coverage_ratio = static_cast<double>(summary.matched) /
      static_cast<double>(summary.valid_raw_stamps);
  }
  return summary;
}

CounterSeriesSummary SummarizeCounterSeries(
  const std::vector<std::uint64_t> & values)
{
  CounterSeriesSummary summary;
  summary.observations = values.size();
  if (values.empty()) {
    return summary;
  }

  summary.epochs = 1U;
  summary.first = values.front();
  summary.last = values.back();
  for (std::size_t index = 1U; index < values.size(); ++index) {
    if (values[index] < values[index - 1U]) {
      ++summary.resets;
      ++summary.epochs;
      continue;
    }
    summary.accumulated_increase = SaturatingAdd(
      summary.accumulated_increase, values[index] - values[index - 1U]);
  }
  if (summary.resets == 0U) {
    summary.delta_without_reset = values.back() - values.front();
  }
  return summary;
}

CounterSeriesState ClassifyCounterSeries(const CounterSeriesSummary & summary)
{
  if (summary.resets > 0U) {
    return CounterSeriesState::kReset;
  }
  if (summary.delta_without_reset.value_or(0U) > 0U) {
    return CounterSeriesState::kIncreased;
  }
  if (summary.first.value_or(0U) > 0U) {
    return CounterSeriesState::kPreexistingNonzero;
  }
  return CounterSeriesState::kClean;
}

}  // namespace bag_contract_probe
