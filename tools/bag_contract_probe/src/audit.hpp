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

#ifndef AUDIT_HPP_
#define AUDIT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <localization_adapter_interfaces/msg/shadow_pose_candidate.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "bag_contract_probe/statistics.hpp"

namespace bag_contract_probe
{
namespace detail
{

constexpr std::int64_t kExpectedImuOffsetNs = 1737987LL;

enum class FindingSeverity
{
  kObserved,
  kReview,
  kFail,
};

struct Finding
{
  FindingSeverity severity;
  std::string code;
  std::string detail;
};

struct NumericMoments
{
  std::uint64_t observations{0U};
  std::uint64_t nonfinite{0U};
  std::optional<double> minimum;
  std::optional<double> maximum;
  long double sum{0.0L};

  void Observe(double value);
  std::optional<double> Mean() const;
};

struct StreamAudit
{
  std::string type;
  std::string serialization_format;
  bool payload_audited{false};
  std::uint64_t messages{0U};
  std::vector<std::int64_t> bag_stamps_ns;
  std::vector<std::int64_t> header_stamps_ns;
  std::map<std::string, std::uint64_t> parent_frames;
  std::map<std::string, std::uint64_t> child_frames;
  std::uint64_t invalid_payload{0U};
};

struct DiagnosticStatusAudit
{
  std::uint64_t samples{0U};
  std::uint64_t duplicate_keys{0U};
  std::vector<std::int64_t> bag_stamps_ns;
  std::vector<std::int64_t> header_stamps_ns;
  std::map<std::uint8_t, std::uint64_t> levels;
  std::map<std::string, std::uint64_t> messages;
  std::map<std::string, std::uint64_t> hardware_ids;
};

struct RequiredDiagnosticAudit
{
  std::uint64_t samples{0U};
  std::uint64_t malformed_samples{0U};
  std::uint64_t contract_mismatches{0U};
  std::uint64_t invariant_violations{0U};
  std::map<std::string, std::vector<std::uint64_t>> counters;
};

struct MocapHealthObservation
{
  std::int64_t header_stamp_ns{0};
  std::int64_t bag_stamp_ns{0};
  std::string health_state;
  std::string reason_code;
};

struct MocapDiagnosticAudit
{
  std::uint64_t samples{0U};
  std::uint64_t malformed_samples{0U};
  std::uint64_t contract_mismatches{0U};
  std::uint64_t invariant_violations{0U};
  std::map<std::string, std::uint64_t> health_states;
  std::map<std::string, std::uint64_t> reasons;
  std::map<std::string, std::vector<std::uint64_t>> counters;
  std::vector<MocapHealthObservation> health_observations;
};

struct PoseFields
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{};
};

struct PosePairAudit
{
  std::uint64_t paired{0U};
  std::uint64_t value_mismatches{0U};
  std::unordered_map<std::int64_t, std::deque<PoseFields>> pending_raw;
  std::unordered_map<std::int64_t, std::deque<PoseFields>> pending_shadow;
};

struct ImuFields
{
  std::array<double, 4> orientation{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 9> orientation_covariance{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 9> linear_acceleration_covariance{};

  bool operator==(const ImuFields & other) const;
};

struct ImuPairAudit
{
  std::uint64_t paired{0U};
  std::uint64_t payload_mismatches{0U};
  std::uint64_t duplicate_pending_keys{0U};
  std::unordered_map<std::int64_t, ImuFields> pending_raw_by_aligned_stamp;
  std::unordered_map<std::int64_t, ImuFields> pending_aligned;
};

struct AuditData
{
  std::map<std::string, StreamAudit> streams;
  std::map<std::string, DiagnosticStatusAudit> diagnostic_statuses;
  MocapDiagnosticAudit mocap_diagnostics;
  PosePairAudit pose_pairs;
  ImuPairAudit imu_pairs;
  std::vector<std::int64_t> raw_mocap_stamps_ns;
  std::vector<std::int64_t> shadow_stamps_ns;
  std::vector<std::int64_t> visual_status_stamps_ns;
  std::vector<std::int64_t> odometry_stamps_ns;
  std::map<std::uint8_t, std::uint64_t> vo_states;
  NumericMoments track_execution_time_sec;
  NumericMoments timesync_round_trip_ms;
  NumericMoments timesync_observed_offset_ns;
  NumericMoments timesync_estimated_offset_ns;
  NumericMoments timesync_offset_innovation_ns;
  NumericMoments timesync_estimated_step_ns;
  std::optional<std::int64_t> previous_timesync_estimated_offset_ns;
  std::vector<std::int64_t> timesync_remote_stamps_ns;
  RequiredDiagnosticAudit aligned_imu_diagnostics;
  RequiredDiagnosticAudit runtime_diagnostics;
  std::vector<Finding> findings;
  std::uint64_t total_messages{0U};
  std::uint64_t duplicate_diagnostic_status_names{0U};
  std::uint64_t shadow_contract_mismatches{0U};

  void ObserveBagStamp(const std::string & topic, std::int64_t bag_stamp_ns);
  void ObserveRawMocap(const geometry_msgs::msg::PoseStamped & message);
  void ObserveShadow(
    const localization_adapter_interfaces::msg::ShadowPoseCandidate & message);
  void ObserveRawImu(const sensor_msgs::msg::Imu & message);
  void ObserveAlignedImu(const sensor_msgs::msg::Imu & message);
  void ObserveVisualStatus(
    const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus & message);
  void ObserveOdometry(const nav_msgs::msg::Odometry & message);
  void ObserveTimesync(const mavros_msgs::msg::TimesyncStatus & message);
  void ObserveDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray & message,
    std::int64_t bag_stamp_ns);
};

std::size_t PendingPoseCount(
  const std::unordered_map<std::int64_t, std::deque<PoseFields>> & pending);
std::string ToString(FindingSeverity severity);

}  // namespace detail
}  // namespace bag_contract_probe

#endif  // AUDIT_HPP_
