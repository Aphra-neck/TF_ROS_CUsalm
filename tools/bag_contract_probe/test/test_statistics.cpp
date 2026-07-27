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

#include <cstdint>
#include <vector>

#include "bag_contract_probe/statistics.hpp"

namespace bag_contract_probe
{
namespace
{

constexpr std::int64_t kSecondNs = 1000000000LL;

TEST(StampSeries, ReportsStrictOrderAndExactGapPercentiles)
{
  const auto summary = SummarizeStampSeries(
    {1 * kSecondNs, 2 * kSecondNs, 4 * kSecondNs, 7 * kSecondNs,
      11 * kSecondNs});

  EXPECT_EQ(summary.observations, 5U);
  EXPECT_EQ(summary.valid_stamps, 5U);
  EXPECT_EQ(summary.zero_or_invalid, 0U);
  EXPECT_EQ(summary.duplicate, 0U);
  EXPECT_EQ(summary.nonmonotonic, 0U);
  ASSERT_TRUE(summary.rate_hz.has_value());
  EXPECT_DOUBLE_EQ(summary.rate_hz.value(), 0.4);
  EXPECT_DOUBLE_EQ(summary.minimum_gap_sec.value(), 1.0);
  EXPECT_DOUBLE_EQ(summary.p50_gap_sec.value(), 2.5);
  EXPECT_NEAR(summary.p95_gap_sec.value(), 3.85, 1.0e-12);
  EXPECT_NEAR(summary.p99_gap_sec.value(), 3.97, 1.0e-12);
  EXPECT_DOUBLE_EQ(summary.maximum_gap_sec.value(), 4.0);
  EXPECT_DOUBLE_EQ(summary.submillisecond_positive_gap_ratio.value(), 0.0);
}

TEST(StampSeries, CountsInvalidDuplicateAndRegressingSamples)
{
  const auto summary = SummarizeStampSeries({0, 10, 10, 9, 20});

  EXPECT_EQ(summary.observations, 5U);
  EXPECT_EQ(summary.valid_stamps, 4U);
  EXPECT_EQ(summary.zero_or_invalid, 1U);
  EXPECT_EQ(summary.duplicate, 1U);
  EXPECT_EQ(summary.nonmonotonic, 1U);
  EXPECT_EQ(summary.positive_gaps, 1U);
}

TEST(StampSeries, ReportsSubmillisecondPositiveGapRatio)
{
  const auto summary = SummarizeStampSeries(
    {1000000000LL, 1000500000LL, 1010000000LL, 1010250000LL});

  ASSERT_TRUE(summary.submillisecond_positive_gap_ratio.has_value());
  EXPECT_DOUBLE_EQ(summary.submillisecond_positive_gap_ratio.value(), 2.0 / 3.0);
}

TEST(EvidenceWindowCoverage, ReportsLateStart)
{
  const auto summary = SummarizeEvidenceWindowCoverage(
    {3 * kSecondNs, 5 * kSecondNs, 10 * kSecondNs},
    1 * kSecondNs, 10 * kSecondNs);

  ASSERT_TRUE(summary.start_gap_sec.has_value());
  ASSERT_TRUE(summary.end_gap_sec.has_value());
  ASSERT_TRUE(summary.covered_duration_sec.has_value());
  EXPECT_DOUBLE_EQ(summary.start_gap_sec.value(), 2.0);
  EXPECT_DOUBLE_EQ(summary.end_gap_sec.value(), 0.0);
  EXPECT_DOUBLE_EQ(summary.covered_duration_sec.value(), 7.0);
}

TEST(EvidenceWindowCoverage, ReportsEarlyStop)
{
  const auto summary = SummarizeEvidenceWindowCoverage(
    {1 * kSecondNs, 5 * kSecondNs, 7 * kSecondNs},
    1 * kSecondNs, 10 * kSecondNs);

  ASSERT_TRUE(summary.start_gap_sec.has_value());
  ASSERT_TRUE(summary.end_gap_sec.has_value());
  EXPECT_DOUBLE_EQ(summary.start_gap_sec.value(), 0.0);
  EXPECT_DOUBLE_EQ(summary.end_gap_sec.value(), 3.0);
}

TEST(EvidenceWindowCoverage, ReportsWindowCroppedAtBothEdges)
{
  const auto summary = SummarizeEvidenceWindowCoverage(
    {4 * kSecondNs, 5 * kSecondNs, 6 * kSecondNs},
    1 * kSecondNs, 10 * kSecondNs);

  ASSERT_TRUE(summary.start_gap_sec.has_value());
  ASSERT_TRUE(summary.end_gap_sec.has_value());
  EXPECT_DOUBLE_EQ(summary.start_gap_sec.value(), 3.0);
  EXPECT_DOUBLE_EQ(summary.end_gap_sec.value(), 4.0);
}

TEST(StampPairing, UsesExactStampMultiplicityAndReportsMissingRuns)
{
  const auto summary = PairExactStampMultisets(
    {10, 20, 30, 40, 50, 60}, {20, 30, 60, 70});

  EXPECT_EQ(summary.valid_raw_stamps, 6U);
  EXPECT_EQ(summary.valid_shadow_stamps, 4U);
  EXPECT_EQ(summary.matched, 3U);
  EXPECT_EQ(summary.raw_without_shadow, 3U);
  EXPECT_EQ(summary.shadow_without_raw, 1U);
  ASSERT_TRUE(summary.raw_coverage_ratio.has_value());
  EXPECT_DOUBLE_EQ(summary.raw_coverage_ratio.value(), 0.5);
  ASSERT_EQ(summary.missing_intervals.size(), 2U);
  EXPECT_EQ(summary.missing_intervals[0].first_stamp_ns, 10);
  EXPECT_EQ(summary.missing_intervals[0].last_stamp_ns, 10);
  EXPECT_EQ(summary.missing_intervals[0].missing_samples, 1U);
  EXPECT_EQ(summary.missing_intervals[1].first_stamp_ns, 40);
  EXPECT_EQ(summary.missing_intervals[1].last_stamp_ns, 50);
  EXPECT_EQ(summary.missing_intervals[1].missing_samples, 2U);
}

TEST(StampPairing, PreservesDuplicateMultiplicity)
{
  const auto summary = PairExactStampMultisets({10, 10, 20}, {10, 20, 20});

  EXPECT_EQ(summary.matched, 2U);
  EXPECT_EQ(summary.raw_without_shadow, 1U);
  EXPECT_EQ(summary.shadow_without_raw, 1U);
}

TEST(StampPairing, ReportsOrphanShadowStampsWithMultiplicityInSortedOrder)
{
  const auto summary = PairExactStampMultisets(
    {20, 20, 40}, {50, 10, 20, 50, 20, 20, 0, -5, 30});

  EXPECT_EQ(summary.valid_raw_stamps, 3U);
  EXPECT_EQ(summary.valid_shadow_stamps, 7U);
  EXPECT_EQ(summary.matched, 2U);
  EXPECT_EQ(summary.raw_without_shadow, 1U);
  EXPECT_EQ(summary.shadow_without_raw, 5U);
  EXPECT_EQ(
    summary.orphan_shadow_stamps_ns,
    (std::vector<std::int64_t>{10, 20, 30, 50, 50}));
}

TEST(CounterSeries, ComputesDeltaOnlyWithinOneEpoch)
{
  const auto summary = SummarizeCounterSeries({100U, 102U, 105U});

  EXPECT_EQ(summary.observations, 3U);
  EXPECT_EQ(summary.epochs, 1U);
  EXPECT_EQ(summary.resets, 0U);
  ASSERT_TRUE(summary.delta_without_reset.has_value());
  EXPECT_EQ(summary.delta_without_reset.value(), 5U);
  EXPECT_EQ(summary.accumulated_increase, 5U);
}

TEST(CounterSeries, ClassifiesLeftCensoredActivityWithoutLosingSafetyMeaning)
{
  EXPECT_EQ(
    ClassifyCounterSeries(SummarizeCounterSeries({0U, 0U})),
    CounterSeriesState::kClean);
  EXPECT_EQ(
    ClassifyCounterSeries(SummarizeCounterSeries({4U, 4U})),
    CounterSeriesState::kPreexistingNonzero);
  EXPECT_EQ(
    ClassifyCounterSeries(SummarizeCounterSeries({4U, 5U})),
    CounterSeriesState::kIncreased);
  EXPECT_EQ(
    ClassifyCounterSeries(SummarizeCounterSeries({4U, 1U})),
    CounterSeriesState::kReset);
}

TEST(CounterSeries, CounterRegressionStartsANewEpoch)
{
  const auto summary = SummarizeCounterSeries({100U, 102U, 4U, 9U});

  EXPECT_EQ(summary.epochs, 2U);
  EXPECT_EQ(summary.resets, 1U);
  EXPECT_FALSE(summary.delta_without_reset.has_value());
  EXPECT_EQ(summary.accumulated_increase, 7U);
}

}  // namespace
}  // namespace bag_contract_probe
