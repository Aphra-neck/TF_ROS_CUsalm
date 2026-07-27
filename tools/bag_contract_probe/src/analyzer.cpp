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

#include "bag_contract_probe/analyzer.hpp"

#include <fcntl.h>
#include <linux/fs.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include "audit.hpp"
#include "bag_contract_probe/statistics.hpp"

#ifndef BAG_CONTRACT_PROBE_SOURCE_REVISION
#define BAG_CONTRACT_PROBE_SOURCE_REVISION "unknown"
#endif

namespace bag_contract_probe
{
namespace
{

namespace fs = std::filesystem;

constexpr char kAnalysisContractId[] =
  "dual_shadow_transport_audit_20260723_v1";
constexpr char kAnalyzerVersion[] = "bag_contract_probe_0.2.0";
constexpr char kExpectedMetadataSha256[] =
  "938ae8784f3401c057d27d904b21dcd71b9d1e092b45fe970efb07c45f303dd4";
constexpr char kExpectedDatabaseSha256[] =
  "37c0cf891789dba0072811d62a3b96b3f0d0a89a9df2c29ac5fcb780c4219cec";
constexpr std::int64_t kExpectedBagStartNs = 1784776014459438325LL;
constexpr std::uint64_t kExpectedBagDurationNs = 1799328205027U;
constexpr std::uint64_t kExpectedBagMessageCount = 1729830U;
constexpr double kMaximumEvidenceEdgeGapSec = 2.5;
constexpr std::int64_t kMaximumEvidenceEdgeGapNs = 2500000000LL;
constexpr std::int64_t kMaximumDiagnosticAgeNs = 500000000LL;

struct OrphanShadowRecord
{
  std::int64_t header_stamp_ns{0};
  std::optional<std::int64_t> bag_stamp_ns;
  std::string evidence_position;
};

struct MocapIntervalDiagnostics
{
  std::map<std::string, std::uint64_t> health_states;
  std::map<std::string, std::uint64_t> reason_codes;
};

struct ExpectedTopic
{
  const char * name;
  const char * type;
  std::uint64_t messages;
};

constexpr std::array<ExpectedTopic, 10> kExpectedTopics{{
  {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", 186890U},
  {"/droneyee207/pose", "geometry_msgs/msg/PoseStamped", 215903U},
  {"/localization/shadow/mocap/assumed_base_pose",
    "localization_adapter_interfaces/msg/ShadowPoseCandidate", 209567U},
  {"/fcu/imu/data_raw_aligned", "sensor_msgs/msg/Imu", 307262U},
  {"/tf", "tf2_msgs/msg/TFMessage", 161703U},
  {"/mavros/imu/data_raw", "sensor_msgs/msg/Imu", 307113U},
  {"/tf_static", "tf2_msgs/msg/TFMessage", 3U},
  {"/visual_slam/status",
    "isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus", 161704U},
  {"/mavros/timesync_status", "mavros_msgs/msg/TimesyncStatus", 17981U},
  {"/visual_slam/tracking/odometry", "nav_msgs/msg/Odometry", 161704U},
}};

constexpr std::uint64_t kMaximumMessages = 1800000U;

struct FileState
{
  std::uintmax_t size{0U};
  fs::file_time_type modified;
  fs::perms permissions{fs::perms::unknown};
  fs::file_type type{fs::file_type::none};

  bool operator==(const FileState & other) const
  {
    return size == other.size && modified == other.modified &&
           permissions == other.permissions && type == other.type;
  }
};

using BagSnapshot = std::map<std::string, FileState>;

BagSnapshot SnapshotBagDirectory(const fs::path & bag_uri)
{
  BagSnapshot snapshot;
  for (const auto & entry : fs::recursive_directory_iterator(bag_uri)) {
    const fs::file_status status = entry.symlink_status();
    FileState state;
    state.type = status.type();
    state.permissions = status.permissions();
    if (entry.is_regular_file()) {
      state.size = entry.file_size();
      state.modified = entry.last_write_time();
    }
    snapshot.emplace(fs::relative(entry.path(), bag_uri).generic_string(), state);
  }
  return snapshot;
}

bool IsSameOrDescendant(const fs::path & candidate, const fs::path & parent)
{
  auto candidate_part = candidate.begin();
  for (auto parent_part = parent.begin(); parent_part != parent.end(); ++parent_part) {
    if (candidate_part == candidate.end() || *candidate_part != *parent_part) {
      return false;
    }
    ++candidate_part;
  }
  return true;
}

bool PathEntryExists(const fs::path & path)
{
  return fs::symlink_status(path).type() != fs::file_type::not_found;
}

void RenameNoReplace(const fs::path & source, const fs::path & destination)
{
  if (::renameat2(
      AT_FDCWD, source.c_str(), AT_FDCWD, destination.c_str(),
      RENAME_NOREPLACE) == 0)
  {
    return;
  }
  const int error_number = errno;
  if (error_number == EEXIST || error_number == ENOTEMPTY) {
    throw std::runtime_error(
            "report directory appeared during atomic publication");
  }
  throw std::runtime_error(
          "cannot publish report directory atomically: " +
          std::string(std::strerror(error_number)));
}

void ValidateStorageLayout(const fs::path & bag_uri)
{
  const BagSnapshot snapshot = SnapshotBagDirectory(bag_uri);
  const std::set<std::string> expected_entries{"metadata.yaml", "rosbag2_0.db3"};
  if (snapshot.size() != expected_entries.size()) {
    throw EvidenceContractError(
            "bag directory must contain only metadata.yaml and rosbag2_0.db3");
  }
  for (const auto & expected : expected_entries) {
    const auto entry = snapshot.find(expected);
    if (entry == snapshot.end() || entry->second.type != fs::file_type::regular) {
      throw EvidenceContractError(
              "bag entry must be a non-symlink regular file: " + expected);
    }
  }

  try {
    const YAML::Node metadata = YAML::LoadFile((bag_uri / "metadata.yaml").string());
    const YAML::Node information = metadata["rosbag2_bagfile_information"];
    const YAML::Node relative_files = information["relative_file_paths"];
    if (!information || !relative_files || !relative_files.IsSequence() ||
      relative_files.size() != 1U ||
      information["storage_identifier"].as<std::string>() != "sqlite3" ||
      information["duration"]["nanoseconds"].as<std::uint64_t>() !=
      kExpectedBagDurationNs ||
      information["starting_time"]["nanoseconds_since_epoch"].as<std::int64_t>() !=
      kExpectedBagStartNs ||
      information["message_count"].as<std::uint64_t>() !=
      kExpectedBagMessageCount)
    {
      throw EvidenceContractError(
              "metadata start, duration, message count, or storage layout "
              "does not match the pinned bag");
    }
    const fs::path relative_database = relative_files[0].as<std::string>();
    if (relative_database.is_absolute() || relative_database != "rosbag2_0.db3") {
      throw EvidenceContractError(
              "metadata must reference only the local rosbag2_0.db3 file");
    }
  } catch (const YAML::Exception & error) {
    throw EvidenceContractError(
            "cannot parse rosbag metadata layout: " + std::string(error.what()));
  }
}

std::pair<fs::path, fs::path> ValidatePaths(const AnalysisRequest & request)
{
  if (request.bag_uri.empty() || request.report_directory.empty()) {
    throw std::invalid_argument("bag URI and report directory are required");
  }
  const fs::path bag_uri = fs::canonical(request.bag_uri);
  if (!fs::is_directory(bag_uri) || !fs::is_regular_file(bag_uri / "metadata.yaml")) {
    throw std::invalid_argument(
            "bag URI must directly contain metadata.yaml: " + bag_uri.string());
  }
  ValidateStorageLayout(bag_uri);
  const fs::path report_directory = fs::weakly_canonical(request.report_directory);
  if (IsSameOrDescendant(report_directory, bag_uri)) {
    throw std::invalid_argument("report directory must be outside the rosbag directory");
  }
  if (PathEntryExists(report_directory)) {
    throw std::invalid_argument("report directory must not already exist");
  }
  fs::path staging_directory = report_directory;
  staging_directory += ".incomplete";
  if (PathEntryExists(staging_directory)) {
    throw std::invalid_argument("report staging directory must not already exist");
  }
  if (!fs::is_directory(report_directory.parent_path())) {
    throw std::invalid_argument("report parent directory must already exist");
  }
  return {bag_uri, report_directory};
}

template<typename MessageT>
MessageT Deserialize(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serializer;
  MessageT message;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

void RegisterAndValidateTopics(
  const std::vector<rosbag2_storage::TopicMetadata> & metadata,
  detail::AuditData * audit)
{
  std::map<std::string, rosbag2_storage::TopicMetadata> actual;
  for (const auto & topic : metadata) {
    if (!actual.emplace(topic.name, topic).second) {
      throw std::runtime_error("duplicate topic metadata for " + topic.name);
    }
    detail::StreamAudit & stream = audit->streams[topic.name];
    stream.type = topic.type;
    stream.serialization_format = topic.serialization_format;
    stream.payload_audited = topic.name != "/tf" && topic.name != "/tf_static";
  }

  if (actual.size() != kExpectedTopics.size()) {
    throw EvidenceContractError(
            "topic set size mismatch: expected " +
            std::to_string(kExpectedTopics.size()) + ", got " +
            std::to_string(actual.size()));
  }

  for (const auto & expected : kExpectedTopics) {
    const auto topic = actual.find(expected.name);
    if (topic == actual.end()) {
      throw EvidenceContractError(
              std::string("required topic is missing: ") + expected.name);
    }
    if (topic->second.type != expected.type) {
      throw EvidenceContractError(
              std::string("type mismatch for ") + expected.name + ": expected " +
              expected.type + ", got " + topic->second.type);
    }
    if (topic->second.serialization_format != "cdr") {
      throw EvidenceContractError(
              std::string("serialization mismatch for ") + expected.name +
              ": expected cdr, got " + topic->second.serialization_format);
    }
  }
}

void DispatchMessage(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & message,
  detail::AuditData * audit)
{
  const std::string & topic = message->topic_name;
  if (audit->total_messages >= kMaximumMessages) {
    throw EvidenceContractError(
            "bag exceeds the 1,800,000-message contract scope limit");
  }
  audit->ObserveBagStamp(topic, message->time_stamp);
  if (topic == "/diagnostics") {
    audit->ObserveDiagnostics(
      Deserialize<diagnostic_msgs::msg::DiagnosticArray>(message),
      message->time_stamp);
  } else if (topic == "/droneyee207/pose") {
    audit->ObserveRawMocap(Deserialize<geometry_msgs::msg::PoseStamped>(message));
  } else if (topic == "/localization/shadow/mocap/assumed_base_pose") {
    audit->ObserveShadow(
      Deserialize<localization_adapter_interfaces::msg::ShadowPoseCandidate>(message));
  } else if (topic == "/mavros/imu/data_raw") {
    audit->ObserveRawImu(Deserialize<sensor_msgs::msg::Imu>(message));
  } else if (topic == "/fcu/imu/data_raw_aligned") {
    audit->ObserveAlignedImu(Deserialize<sensor_msgs::msg::Imu>(message));
  } else if (topic == "/visual_slam/status") {
    audit->ObserveVisualStatus(
      Deserialize<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(message));
  } else if (topic == "/visual_slam/tracking/odometry") {
    audit->ObserveOdometry(Deserialize<nav_msgs::msg::Odometry>(message));
  } else if (topic == "/mavros/timesync_status") {
    audit->ObserveTimesync(Deserialize<mavros_msgs::msg::TimesyncStatus>(message));
  }
}

void AddFinding(
  detail::AuditData * audit,
  const detail::FindingSeverity severity,
  const std::string & code,
  const std::string & message)
{
  audit->findings.push_back(detail::Finding{severity, code, message});
}

std::string JoinCounts(const std::map<std::string, std::uint64_t> & counts)
{
  std::ostringstream output;
  bool first = true;
  for (const auto & entry : counts) {
    if (!first) {
      output << ';';
    }
    output << entry.first << ':' << entry.second;
    first = false;
  }
  return output.str();
}

std::string JoinByteCounts(const std::map<std::uint8_t, std::uint64_t> & counts)
{
  std::ostringstream output;
  bool first = true;
  for (const auto & entry : counts) {
    if (!first) {
      output << ';';
    }
    output << static_cast<unsigned int>(entry.first) << ':' << entry.second;
    first = false;
  }
  return output.str();
}

std::string OptionalDouble(const std::optional<double> value)
{
  if (!value.has_value()) {
    return "NA";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value.value();
  return output.str();
}

template<typename T>
std::string OptionalInteger(const std::optional<T> value)
{
  return value.has_value() ? std::to_string(value.value()) : "NA";
}

std::string Csv(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '\"';
  return escaped;
}

void FinishReport(std::ofstream * output, const fs::path & path)
{
  output->flush();
  if (!(*output)) {
    throw std::runtime_error("failed while writing " + path.string());
  }
  output->close();
  if (output->fail()) {
    throw std::runtime_error("failed to close " + path.string());
  }
}

void RequireFrames(
  detail::AuditData * audit,
  const std::string & topic,
  const std::map<std::string, std::uint64_t> & observed,
  const std::string & expected,
  const std::string & role)
{
  const auto expected_count = observed.find(expected);
  std::uint64_t matching = expected_count == observed.end() ? 0U : expected_count->second;
  const auto stream = audit->streams.find(topic);
  const std::uint64_t messages = stream == audit->streams.end() ? 0U : stream->second.messages;
  if (observed.size() != 1U || matching != messages) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "FRAME_CONTRACT_MISMATCH",
      topic + " expected " + role + "=" + expected + ", observed " +
      JoinCounts(observed));
  }
}

void CheckStampContract(
  detail::AuditData * audit,
  const std::string & topic,
  const double minimum_rate_hz,
  const double maximum_gap_sec)
{
  const auto stream = audit->streams.find(topic);
  if (stream == audit->streams.end()) {
    return;
  }
  const StampSeriesSummary summary = SummarizeStampSeries(
    stream->second.header_stamps_ns);
  if (summary.zero_or_invalid > 0U || summary.duplicate > 0U ||
    summary.nonmonotonic > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "HEADER_STAMP_CONTRACT_FAILURE",
      topic + " zero_or_invalid=" + std::to_string(summary.zero_or_invalid) +
      " duplicate=" + std::to_string(summary.duplicate) +
      " nonmonotonic=" + std::to_string(summary.nonmonotonic));
    return;
  }
  if (!summary.rate_hz.has_value() || summary.rate_hz.value() < minimum_rate_hz) {
    AddFinding(
      audit, detail::FindingSeverity::kReview, "HEADER_RATE_BELOW_CONTRACT",
      topic + " rate_hz=" + OptionalDouble(summary.rate_hz) +
      " minimum_hz=" + OptionalDouble(minimum_rate_hz));
  }
  if (!summary.maximum_gap_sec.has_value() ||
    summary.maximum_gap_sec.value() > maximum_gap_sec)
  {
    AddFinding(
      audit, detail::FindingSeverity::kReview, "HEADER_GAP_ABOVE_CONTRACT",
      topic + " maximum_gap_sec=" + OptionalDouble(summary.maximum_gap_sec) +
      " limit_sec=" + OptionalDouble(maximum_gap_sec));
  }
}

void CheckDiagnosticCoverage(
  detail::AuditData * audit,
  const std::string & name,
  const double minimum_rate_hz,
  const double maximum_gap_sec,
  const std::pair<std::int64_t, std::int64_t> & evidence_window)
{
  const auto status = audit->diagnostic_statuses.find(name);
  if (status == audit->diagnostic_statuses.end() || status->second.samples == 0U) {
    return;
  }
  const StampSeriesSummary summary = SummarizeStampSeries(
    status->second.header_stamps_ns);
  const StampSeriesSummary bag_summary = SummarizeStampSeries(
    status->second.bag_stamps_ns);
  if (bag_summary.zero_or_invalid > 0U || bag_summary.duplicate > 0U ||
    bag_summary.nonmonotonic > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail,
      "DIAGNOSTIC_BAG_STAMP_CONTRACT_FAILURE",
      name + " zero_or_invalid=" + std::to_string(bag_summary.zero_or_invalid) +
      " duplicate=" + std::to_string(bag_summary.duplicate) +
      " nonmonotonic=" + std::to_string(bag_summary.nonmonotonic));
  }
  const EvidenceWindowCoverageSummary coverage = SummarizeEvidenceWindowCoverage(
    status->second.bag_stamps_ns, evidence_window.first, evidence_window.second);
  if (!coverage.start_gap_sec.has_value() || !coverage.end_gap_sec.has_value() ||
    coverage.start_gap_sec.value() > kMaximumEvidenceEdgeGapSec ||
    coverage.end_gap_sec.value() > kMaximumEvidenceEdgeGapSec)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail,
      "DIAGNOSTIC_EVIDENCE_EDGE_COVERAGE_FAILURE",
      name + " start_gap_sec=" + OptionalDouble(coverage.start_gap_sec) +
      " end_gap_sec=" + OptionalDouble(coverage.end_gap_sec) +
      " limit_sec=" + OptionalDouble(kMaximumEvidenceEdgeGapSec));
  }
  if (summary.zero_or_invalid > 0U || summary.duplicate > 0U ||
    summary.nonmonotonic > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "DIAGNOSTIC_STAMP_CONTRACT_FAILURE",
      name + " zero_or_invalid=" + std::to_string(summary.zero_or_invalid) +
      " duplicate=" + std::to_string(summary.duplicate) +
      " nonmonotonic=" + std::to_string(summary.nonmonotonic));
    return;
  }
  if (!summary.rate_hz.has_value() || summary.rate_hz.value() < minimum_rate_hz ||
    !summary.maximum_gap_sec.has_value() ||
    summary.maximum_gap_sec.value() > maximum_gap_sec)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "DIAGNOSTIC_COVERAGE_FAILURE",
      name + " rate_hz=" + OptionalDouble(summary.rate_hz) +
      " minimum_hz=" + OptionalDouble(minimum_rate_hz) +
      " maximum_gap_sec=" + OptionalDouble(summary.maximum_gap_sec) +
      " limit_sec=" + OptionalDouble(maximum_gap_sec));
  }
}

const std::set<std::string> & EvidenceWindowTopics()
{
  static const std::set<std::string> topics{
    "/droneyee207/pose",
    "/fcu/imu/data_raw_aligned",
    "/mavros/imu/data_raw",
    "/mavros/timesync_status",
    "/tf",
    "/visual_slam/status",
    "/visual_slam/tracking/odometry"};
  return topics;
}

std::pair<std::int64_t, std::int64_t> PinnedEvidenceWindow()
{
  return {
    kExpectedBagStartNs,
    kExpectedBagStartNs + static_cast<std::int64_t>(kExpectedBagDurationNs)};
}

std::string ClassifyEvidencePosition(
  const std::optional<std::int64_t> bag_stamp_ns,
  const std::pair<std::int64_t, std::int64_t> & evidence_window)
{
  if (!bag_stamp_ns.has_value()) {
    return "UNMAPPED";
  }
  if (bag_stamp_ns.value() <= evidence_window.first + kMaximumEvidenceEdgeGapNs) {
    return "START_EDGE";
  }
  if (bag_stamp_ns.value() >= evidence_window.second - kMaximumEvidenceEdgeGapNs) {
    return "END_EDGE";
  }
  return "INTERIOR";
}

std::vector<OrphanShadowRecord> BuildOrphanShadowRecords(
  const detail::AuditData & audit,
  const StampPairingSummary & pairing)
{
  std::map<std::int64_t, std::size_t> remaining;
  for (const std::int64_t stamp_ns : pairing.orphan_shadow_stamps_ns) {
    ++remaining[stamp_ns];
  }
  std::map<std::int64_t, std::size_t> raw_counts;
  std::map<std::int64_t, std::size_t> shadow_counts;
  for (const std::int64_t stamp_ns : audit.raw_mocap_stamps_ns) {
    if (remaining.count(stamp_ns) > 0U) {
      ++raw_counts[stamp_ns];
    }
  }
  for (const std::int64_t stamp_ns : audit.shadow_stamps_ns) {
    if (remaining.count(stamp_ns) > 0U) {
      ++shadow_counts[stamp_ns];
    }
  }
  std::set<std::int64_t> ambiguous_stamps;
  for (const auto & entry : remaining) {
    const auto raw = raw_counts.find(entry.first);
    const auto shadow = shadow_counts.find(entry.first);
    if (raw != raw_counts.end() && raw->second > 0U &&
      shadow != shadow_counts.end() && shadow->second > entry.second)
    {
      ambiguous_stamps.insert(entry.first);
    }
  }

  std::vector<OrphanShadowRecord> records;
  records.reserve(pairing.orphan_shadow_stamps_ns.size());
  const auto stream = audit.streams.find(
    "/localization/shadow/mocap/assumed_base_pose");
  if (stream != audit.streams.end()) {
    const std::size_t samples = std::min(
      stream->second.header_stamps_ns.size(), stream->second.bag_stamps_ns.size());
    const auto evidence_window = PinnedEvidenceWindow();
    for (std::size_t index = 0U; index < samples; ++index) {
      const std::int64_t header_stamp_ns = stream->second.header_stamps_ns[index];
      const auto candidate = remaining.find(header_stamp_ns);
      if (candidate == remaining.end() || candidate->second == 0U) {
        continue;
      }
      if (ambiguous_stamps.count(header_stamp_ns) > 0U) {
        continue;
      }
      const std::int64_t bag_stamp_ns = stream->second.bag_stamps_ns[index];
      records.push_back(
        OrphanShadowRecord{
            header_stamp_ns, bag_stamp_ns,
            ClassifyEvidencePosition(bag_stamp_ns, evidence_window)});
      --candidate->second;
    }
  }

  for (const auto & entry : remaining) {
    const std::string position = ambiguous_stamps.count(entry.first) > 0U ?
      "AMBIGUOUS_DUPLICATE_STAMP" : "UNMAPPED";
    for (std::size_t count = 0U; count < entry.second; ++count) {
      records.push_back(
        OrphanShadowRecord{entry.first, std::nullopt, position});
    }
  }
  std::sort(
    records.begin(), records.end(),
    [](const OrphanShadowRecord & left, const OrphanShadowRecord & right) {
      if (left.header_stamp_ns != right.header_stamp_ns) {
        return left.header_stamp_ns < right.header_stamp_ns;
      }
      if (left.bag_stamp_ns.has_value() != right.bag_stamp_ns.has_value()) {
        return left.bag_stamp_ns.has_value();
      }
      return left.bag_stamp_ns.value_or(0) < right.bag_stamp_ns.value_or(0);
    });
  return records;
}

std::map<std::string, std::uint64_t> CountOrphanPositions(
  const std::vector<OrphanShadowRecord> & records)
{
  std::map<std::string, std::uint64_t> counts;
  for (const auto & record : records) {
    ++counts[record.evidence_position];
  }
  return counts;
}

void CheckBagStampContracts(
  detail::AuditData * audit,
  const std::pair<std::int64_t, std::int64_t> & evidence_window)
{
  for (const auto & expected : kExpectedTopics) {
    const auto stream = audit->streams.find(expected.name);
    if (stream == audit->streams.end()) {
      continue;
    }
    const StampSeriesSummary stamps = SummarizeStampSeries(
      stream->second.bag_stamps_ns);
    if (stamps.zero_or_invalid > 0U || stamps.duplicate > 0U ||
      stamps.nonmonotonic > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "BAG_STAMP_CONTRACT_FAILURE",
        std::string(expected.name) + " zero_or_invalid=" +
        std::to_string(stamps.zero_or_invalid) + " duplicate=" +
        std::to_string(stamps.duplicate) + " nonmonotonic=" +
        std::to_string(stamps.nonmonotonic));
    }

    if (EvidenceWindowTopics().count(expected.name) == 0U) {
      continue;
    }
    const EvidenceWindowCoverageSummary coverage = SummarizeEvidenceWindowCoverage(
      stream->second.bag_stamps_ns, evidence_window.first, evidence_window.second);
    if (!coverage.start_gap_sec.has_value() || !coverage.end_gap_sec.has_value() ||
      coverage.start_gap_sec.value() > kMaximumEvidenceEdgeGapSec ||
      coverage.end_gap_sec.value() > kMaximumEvidenceEdgeGapSec)
    {
      AddFinding(
        audit, detail::FindingSeverity::kFail,
        "TOPIC_EVIDENCE_EDGE_COVERAGE_FAILURE",
        std::string(expected.name) + " start_gap_sec=" +
        OptionalDouble(coverage.start_gap_sec) + " end_gap_sec=" +
        OptionalDouble(coverage.end_gap_sec) + " limit_sec=" +
        OptionalDouble(kMaximumEvidenceEdgeGapSec));
    }
  }
}

void CheckCounterSeries(
  detail::AuditData * audit,
  const std::string & source,
  const std::map<std::string, std::vector<std::uint64_t>> & counters,
  const std::set<std::string> & fail_on_increment,
  const std::set<std::string> & review_on_increment)
{
  for (const auto & entry : counters) {
    const CounterSeriesSummary summary = SummarizeCounterSeries(entry.second);
    const CounterSeriesState state = ClassifyCounterSeries(summary);
    if (state == CounterSeriesState::kReset) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "DIAGNOSTIC_COUNTER_RESET",
        source + " key=" + entry.first + " resets=" +
        std::to_string(summary.resets));
      continue;
    }
    const std::uint64_t first = summary.first.value_or(0U);
    const std::uint64_t delta = summary.delta_without_reset.value_or(0U);
    if (state == CounterSeriesState::kIncreased &&
      fail_on_increment.count(entry.first) > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "DIAGNOSTIC_FAILURE_COUNTER_INCREASED",
        source + " key=" + entry.first + " first_left_censored=" +
        std::to_string(first) + " delta=" + std::to_string(delta));
      continue;
    }
    if (state == CounterSeriesState::kPreexistingNonzero &&
      fail_on_increment.count(entry.first) > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kFail,
        "DIAGNOSTIC_FAILURE_COUNTER_PREEXISTING",
        source + " key=" + entry.first + " first_left_censored=" +
        std::to_string(first) + " delta=0");
      continue;
    }
    if (state == CounterSeriesState::kIncreased &&
      review_on_increment.count(entry.first) > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kReview, "DIAGNOSTIC_WARNING_COUNTER_INCREASED",
        source + " key=" + entry.first + " first_left_censored=" +
        std::to_string(first) + " delta=" + std::to_string(delta));
      continue;
    }
    if (state == CounterSeriesState::kPreexistingNonzero &&
      review_on_increment.count(entry.first) > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kReview,
        "DIAGNOSTIC_WARNING_COUNTER_PREEXISTING",
        source + " key=" + entry.first + " first_left_censored=" +
        std::to_string(first) + " delta=0");
    }
  }
}

std::uint64_t CountWithin(
  const std::vector<std::int64_t> & stamps,
  const std::int64_t first,
  const std::int64_t last)
{
  return static_cast<std::uint64_t>(std::count_if(
           stamps.begin(), stamps.end(),
           [first, last](const std::int64_t stamp) {
             return stamp >= first && stamp <= last;
           }));
}

void FinalizeAudit(detail::AuditData * audit)
{
  for (const auto & expected : kExpectedTopics) {
    const auto stream = audit->streams.find(expected.name);
    const std::uint64_t observed =
      stream == audit->streams.end() ? 0U : stream->second.messages;
    if (observed != expected.messages) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "TOPIC_MESSAGE_COUNT_MISMATCH",
        std::string(expected.name) + " expected=" +
        std::to_string(expected.messages) + " observed=" +
        std::to_string(observed));
    }
  }

  for (const auto & entry : audit->streams) {
    if (entry.second.invalid_payload > 0U) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "INVALID_PAYLOAD",
        entry.first + " invalid_payload=" +
        std::to_string(entry.second.invalid_payload));
    }
  }

  const auto evidence_window = PinnedEvidenceWindow();
  CheckBagStampContracts(audit, evidence_window);

  RequireFrames(
    audit, "/droneyee207/pose",
    audit->streams["/droneyee207/pose"].parent_frames, "world", "parent");
  RequireFrames(
    audit, "/localization/shadow/mocap/assumed_base_pose",
    audit->streams["/localization/shadow/mocap/assumed_base_pose"].parent_frames,
    "mocap_world", "parent");
  RequireFrames(
    audit, "/localization/shadow/mocap/assumed_base_pose",
    audit->streams["/localization/shadow/mocap/assumed_base_pose"].child_frames,
    "base_link", "semantic_child");
  RequireFrames(
    audit, "/mavros/imu/data_raw",
    audit->streams["/mavros/imu/data_raw"].parent_frames, "base_link", "frame");
  RequireFrames(
    audit, "/fcu/imu/data_raw_aligned",
    audit->streams["/fcu/imu/data_raw_aligned"].parent_frames, "fcu_imu", "frame");
  RequireFrames(
    audit, "/visual_slam/status",
    audit->streams["/visual_slam/status"].parent_frames, "map", "parent");
  RequireFrames(
    audit, "/visual_slam/tracking/odometry",
    audit->streams["/visual_slam/tracking/odometry"].parent_frames, "odom", "parent");
  RequireFrames(
    audit, "/visual_slam/tracking/odometry",
    audit->streams["/visual_slam/tracking/odometry"].child_frames,
    "camera_link", "child");

  CheckStampContract(audit, "/droneyee207/pose", 90.0, 0.05);
  CheckStampContract(
    audit, "/localization/shadow/mocap/assumed_base_pose", 90.0, 0.05);
  CheckStampContract(audit, "/mavros/imu/data_raw", 144.5, 3.0 / 170.0);
  CheckStampContract(audit, "/fcu/imu/data_raw_aligned", 144.5, 3.0 / 170.0);
  CheckStampContract(audit, "/visual_slam/status", 60.0, 0.1);
  CheckStampContract(audit, "/visual_slam/tracking/odometry", 60.0, 0.1);
  CheckStampContract(audit, "/mavros/timesync_status", 1.0, 1.0);

  const StampPairingSummary mocap_pairing = PairExactStampMultisets(
    audit->raw_mocap_stamps_ns, audit->shadow_stamps_ns);
  if (mocap_pairing.shadow_without_raw > 0U) {
    const auto orphan_records = BuildOrphanShadowRecords(*audit, mocap_pairing);
    AddFinding(
      audit, detail::FindingSeverity::kFail, "SHADOW_WITHOUT_RAW_STAMP",
      "orphan shadow samples=" + std::to_string(mocap_pairing.shadow_without_raw) +
      " positions=" + JoinCounts(CountOrphanPositions(orphan_records)) +
      " report=mocap_orphan_shadows.csv");
  }
  if (mocap_pairing.raw_without_shadow > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kReview, "RAW_WITHOUT_SHADOW_STAMP",
      "raw samples without recorded shadow=" +
      std::to_string(mocap_pairing.raw_without_shadow));
  }
  if (audit->pose_pairs.value_mismatches > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "IDENTITY_SHADOW_VALUE_MISMATCH",
      "paired poses outside identity-assumption tolerance=" +
      std::to_string(audit->pose_pairs.value_mismatches));
  }
  if (audit->shadow_contract_mismatches > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "SHADOW_MESSAGE_CONTRACT_MISMATCH",
      "messages=" + std::to_string(audit->shadow_contract_mismatches));
  }

  if (audit->imu_pairs.payload_mismatches > 0U ||
    audit->imu_pairs.duplicate_pending_keys > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "IMU_RELAY_PAYLOAD_MISMATCH",
      "payload_mismatches=" + std::to_string(audit->imu_pairs.payload_mismatches) +
      " duplicate_pairing_keys=" +
      std::to_string(audit->imu_pairs.duplicate_pending_keys));
  }
  if (!audit->imu_pairs.pending_raw_by_aligned_stamp.empty() ||
    !audit->imu_pairs.pending_aligned.empty())
  {
    AddFinding(
      audit, detail::FindingSeverity::kReview, "IMU_RECORDING_ORPHANS",
      "raw_without_aligned=" +
      std::to_string(audit->imu_pairs.pending_raw_by_aligned_stamp.size()) +
      " aligned_without_raw=" + std::to_string(audit->imu_pairs.pending_aligned.size()));
  }

  const StampPairingSummary vo_pairing = PairExactStampMultisets(
    audit->odometry_stamps_ns, audit->visual_status_stamps_ns);
  if (vo_pairing.raw_without_shadow > 0U || vo_pairing.shadow_without_raw > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "ODOMETRY_STATUS_STAMP_MISMATCH",
      "odometry_without_status=" + std::to_string(vo_pairing.raw_without_shadow) +
      " status_without_odometry=" + std::to_string(vo_pairing.shadow_without_raw));
  }
  for (const auto & state : audit->vo_states) {
    if (state.first != 1U && state.second > 0U) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "CUVSLAM_VO_STATE_NOT_SUCCESS",
        "vo_state=" + std::to_string(static_cast<unsigned int>(state.first)) +
        " samples=" + std::to_string(state.second));
    }
  }

  const StampSeriesSummary remote_time = SummarizeStampSeries(
    audit->timesync_remote_stamps_ns);
  if (remote_time.zero_or_invalid > 0U || remote_time.duplicate > 0U ||
    remote_time.nonmonotonic > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "TIMESYNC_REMOTE_STAMP_FAILURE",
      "zero_or_invalid=" + std::to_string(remote_time.zero_or_invalid) +
      " duplicate=" + std::to_string(remote_time.duplicate) +
      " nonmonotonic=" + std::to_string(remote_time.nonmonotonic));
  }

  if (audit->duplicate_diagnostic_status_names > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "DUPLICATE_DIAGNOSTIC_STATUS_NAME",
      "duplicates=" + std::to_string(audit->duplicate_diagnostic_status_names));
  }
  const std::set<std::string> required_diagnostics{
    "/mocap_localization_adapter: mocap shadow contract",
    "/aligned_fcu_imu_relay: aligned FCU IMU",
    "/d435i_cuvslam_runtime_health_monitor: calibrated runtime"};
  for (const auto & status : audit->diagnostic_statuses) {
    if (required_diagnostics.count(status.first) > 0U &&
      status.second.duplicate_keys > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "MALFORMED_DIAGNOSTIC_VALUES",
        status.first + " duplicate_keys=" +
        std::to_string(status.second.duplicate_keys));
    }
  }
  for (const auto & name : required_diagnostics) {
    const auto status = audit->diagnostic_statuses.find(name);
    if (status == audit->diagnostic_statuses.end() || status->second.samples == 0U) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "REQUIRED_DIAGNOSTIC_MISSING", name);
    }
  }
  CheckDiagnosticCoverage(
    audit, "/mocap_localization_adapter: mocap shadow contract", 5.0, 0.5,
    evidence_window);
  CheckDiagnosticCoverage(
    audit, "/aligned_fcu_imu_relay: aligned FCU IMU", 0.5, 2.5,
    evidence_window);
  CheckDiagnosticCoverage(
    audit, "/d435i_cuvslam_runtime_health_monitor: calibrated runtime", 0.5, 2.5,
    evidence_window);

  if (audit->mocap_diagnostics.malformed_samples > 0U ||
    audit->mocap_diagnostics.contract_mismatches > 0U ||
    audit->mocap_diagnostics.invariant_violations > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "MOCAP_DIAGNOSTIC_CONTRACT_FAILURE",
      "malformed=" + std::to_string(audit->mocap_diagnostics.malformed_samples) +
      " contract_mismatch=" +
      std::to_string(audit->mocap_diagnostics.contract_mismatches) +
      " invariant_violation=" +
      std::to_string(audit->mocap_diagnostics.invariant_violations));
  }
  if (audit->aligned_imu_diagnostics.malformed_samples > 0U ||
    audit->aligned_imu_diagnostics.contract_mismatches > 0U ||
    audit->aligned_imu_diagnostics.invariant_violations > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "ALIGNED_IMU_DIAGNOSTIC_CONTRACT_FAILURE",
      "malformed=" +
      std::to_string(audit->aligned_imu_diagnostics.malformed_samples) +
      " contract_mismatch=" +
      std::to_string(audit->aligned_imu_diagnostics.contract_mismatches) +
      " invariant_violation=" +
      std::to_string(audit->aligned_imu_diagnostics.invariant_violations));
  }
  if (audit->runtime_diagnostics.malformed_samples > 0U ||
    audit->runtime_diagnostics.contract_mismatches > 0U)
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "RUNTIME_DIAGNOSTIC_CONTRACT_FAILURE",
      "malformed=" + std::to_string(audit->runtime_diagnostics.malformed_samples) +
      " contract_mismatch=" +
      std::to_string(audit->runtime_diagnostics.contract_mismatches));
  }

  const std::set<std::string> known_mocap_states{
    "starting", "healthy", "transient_fault", "recovering", "latched_fault"};
  for (const auto & state : audit->mocap_diagnostics.health_states) {
    if (known_mocap_states.count(state.first) == 0U) {
      AddFinding(
        audit, detail::FindingSeverity::kFail, "UNKNOWN_MOCAP_HEALTH_STATE",
        "state=" + state.first + " samples=" + std::to_string(state.second));
    }
  }
  const auto healthy = audit->mocap_diagnostics.health_states.find("healthy");
  if (healthy == audit->mocap_diagnostics.health_states.end() || healthy->second == 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "MOCAP_NEVER_HEALTHY",
      "no healthy diagnostic sample was recorded");
  }
  const auto latched = audit->mocap_diagnostics.health_states.find("latched_fault");
  if (latched != audit->mocap_diagnostics.health_states.end() && latched->second > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "MOCAP_LATCHED_FAULT_OBSERVED",
      "samples=" + std::to_string(latched->second));
  }
  for (const std::string state : {"transient_fault", "recovering"}) {
    const auto observed = audit->mocap_diagnostics.health_states.find(state);
    if (observed != audit->mocap_diagnostics.health_states.end() &&
      observed->second > 0U)
    {
      AddFinding(
        audit, detail::FindingSeverity::kReview, "MOCAP_NONHEALTHY_INTERVAL",
        "state=" + state + " samples=" + std::to_string(observed->second));
    }
  }
  const auto starting = audit->mocap_diagnostics.health_states.find("starting");
  if (starting != audit->mocap_diagnostics.health_states.end() && starting->second > 0U) {
    AddFinding(
      audit, detail::FindingSeverity::kReview, "MOCAP_STARTING_INTERVAL",
      "samples=" + std::to_string(starting->second));
  }

  CheckCounterSeries(
    audit, "mocap", audit->mocap_diagnostics.counters,
    {"zero_or_invalid_stamp", "duplicate", "nonmonotonic", "frame_mismatch",
      "nonfinite_position", "out_of_bounds_position", "invalid_quaternion",
      "clock_domain_mismatch", "pose_reset_candidate",
      "publisher_authority_violation", "output_publisher_authority_violation"},
    {"rejected", "stamp_gap_violation", "receive_gap_violation"});
  CheckCounterSeries(
    audit, "aligned_imu", audit->aligned_imu_diagnostics.counters,
    {"zero_stamp", "invalid_stamp", "duplicate", "nonmonotonic", "frame_mismatch",
      "nonfinite_measurement", "clock_domain_mismatch", "aligned_out_of_range"},
    {});
  CheckCounterSeries(audit, "runtime", audit->runtime_diagnostics.counters, {}, {});

  const auto forbidden =
    audit->runtime_diagnostics.counters.find("forbidden_camera_imu");
  if (forbidden != audit->runtime_diagnostics.counters.end() &&
    std::any_of(
      forbidden->second.begin(), forbidden->second.end(),
      [](const std::uint64_t value) {return value != 0U;}))
  {
    AddFinding(
      audit, detail::FindingSeverity::kFail, "FORBIDDEN_CAMERA_IMU_OBSERVED",
      "runtime diagnostic counter was nonzero");
  }

  if (audit->mocap_diagnostics.health_observations.size() >= 2U) {
    const std::int64_t first =
      audit->mocap_diagnostics.health_observations.front().header_stamp_ns;
    const std::int64_t last =
      audit->mocap_diagnostics.health_observations.back().header_stamp_ns;
    const std::uint64_t raw_in_window = CountWithin(
      audit->raw_mocap_stamps_ns, first, last);
    const std::uint64_t shadow_in_window = CountWithin(
      audit->shadow_stamps_ns, first, last);
    const auto received = audit->mocap_diagnostics.counters.find("received");
    const auto published =
      audit->mocap_diagnostics.counters.find("published_shadow_candidates");
    const auto received_delta = received == audit->mocap_diagnostics.counters.end() ?
      std::optional<std::uint64_t>{} :
    SummarizeCounterSeries(received->second).delta_without_reset;
    const auto published_delta = published == audit->mocap_diagnostics.counters.end() ?
      std::optional<std::uint64_t>{} :
    SummarizeCounterSeries(published->second).delta_without_reset;
    AddFinding(
      audit, detail::FindingSeverity::kObserved, "MOCAP_COUNTER_WINDOW_CROSSCHECK",
      "raw_recorded=" + std::to_string(raw_in_window) +
      " received_counter_delta=" + OptionalInteger(received_delta) +
      " shadow_recorded=" + std::to_string(shadow_in_window) +
      " published_counter_delta=" + OptionalInteger(published_delta));
  }

  AddFinding(
    audit, detail::FindingSeverity::kObserved, "CROSS_SOURCE_TRAJECTORY_BLOCKED",
    "T[base_link,camera_link], T[mocap_world,odom], and VRPN capture time are not approved; "
    "ATE, RPE, axis, yaw, and latency conclusions are NOT_AUTHORIZED");
}

RuntimeContractVerdict DetermineVerdict(const detail::AuditData & audit)
{
  bool review = false;
  for (const auto & finding : audit.findings) {
    if (finding.severity == detail::FindingSeverity::kFail) {
      return RuntimeContractVerdict::kFail;
    }
    if (finding.severity == detail::FindingSeverity::kReview) {
      review = true;
    }
  }
  return review ? RuntimeContractVerdict::kReviewRequired :
         RuntimeContractVerdict::kPass;
}

std::string ClassifyMissingInterval(
  const detail::AuditData & audit,
  const MissingStampInterval & interval,
  const std::optional<std::int64_t> first_shadow,
  const std::optional<std::int64_t> last_shadow)
{
  if (first_shadow.has_value() && last_shadow.has_value()) {
    if (interval.last_stamp_ns < first_shadow.value()) {
      return "STARTUP_EDGE";
    }
    if (interval.first_stamp_ns > last_shadow.value()) {
      return "TEARDOWN_EDGE";
    }
  }

  const detail::MocapHealthObservation * latest_at_start = nullptr;
  for (const auto & observation : audit.mocap_diagnostics.health_observations) {
    if (observation.header_stamp_ns <= 0 ||
      observation.header_stamp_ns > interval.first_stamp_ns)
    {
      continue;
    }
    if (latest_at_start == nullptr ||
      observation.header_stamp_ns > latest_at_start->header_stamp_ns)
    {
      latest_at_start = &observation;
    }
  }
  if (latest_at_start == nullptr ||
    interval.first_stamp_ns - latest_at_start->header_stamp_ns >
    kMaximumDiagnosticAgeNs)
  {
    return "DIAGNOSTIC_UNKNOWN";
  }

  const auto is_healthy = [](const detail::MocapHealthObservation & observation) {
      return observation.health_state == "healthy" &&
             observation.reason_code == "INPUT_HEALTHY";
    };
  bool observed_healthy = is_healthy(*latest_at_start);
  bool observed_nonhealthy = !observed_healthy;
  const detail::MocapHealthObservation * latest_at_end = latest_at_start;
  for (const auto & observation : audit.mocap_diagnostics.health_observations) {
    if (observation.header_stamp_ns <= interval.first_stamp_ns ||
      observation.header_stamp_ns > interval.last_stamp_ns)
    {
      continue;
    }
    observed_healthy = observed_healthy || is_healthy(observation);
    observed_nonhealthy = observed_nonhealthy || !is_healthy(observation);
    if (observation.header_stamp_ns > latest_at_end->header_stamp_ns) {
      latest_at_end = &observation;
    }
  }
  if (interval.last_stamp_ns - latest_at_end->header_stamp_ns >
    kMaximumDiagnosticAgeNs)
  {
    return "DIAGNOSTIC_UNKNOWN";
  }
  if (observed_healthy && observed_nonhealthy) {
    return "MIXED_HEALTH";
  }
  if (observed_healthy) {
    return "HEALTHY_INTERIOR";
  }
  return "NONHEALTHY_OR_RECOVERING";
}

MocapIntervalDiagnostics SummarizeIntervalDiagnostics(
  const detail::AuditData & audit,
  const MissingStampInterval & interval)
{
  MocapIntervalDiagnostics summary;
  const detail::MocapHealthObservation * carried_in = nullptr;
  for (const auto & observation : audit.mocap_diagnostics.health_observations) {
    if (observation.header_stamp_ns <= interval.first_stamp_ns) {
      if (observation.header_stamp_ns > 0 &&
        (carried_in == nullptr ||
        observation.header_stamp_ns > carried_in->header_stamp_ns))
      {
        carried_in = &observation;
      }
      continue;
    }
    if (observation.header_stamp_ns > interval.last_stamp_ns) {
      continue;
    }
    ++summary.health_states[observation.health_state];
    ++summary.reason_codes[observation.reason_code];
  }
  if (carried_in != nullptr &&
    interval.first_stamp_ns - carried_in->header_stamp_ns <= kMaximumDiagnosticAgeNs)
  {
    ++summary.health_states[carried_in->health_state];
    ++summary.reason_codes[carried_in->reason_code];
  }
  return summary;
}

void WriteTopicStatistics(
  const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "topic,type,serialization,messages,payload_audited,invalid_payload," <<
    "bag_rate_hz,bag_zero,bag_duplicate,bag_nonmonotonic," <<
    "bag_gap_min_sec,bag_gap_p50_sec,bag_gap_p95_sec,bag_gap_p99_sec," <<
    "bag_gap_max_sec,bag_submillisecond_ratio," <<
    "header_rate_hz,header_zero,header_duplicate,header_nonmonotonic," <<
    "header_gap_min_sec,header_gap_p50_sec,header_gap_p95_sec," <<
    "header_gap_p99_sec,header_gap_max_sec,header_submillisecond_ratio,"
    "parent_frames,child_frames\n";
  for (const auto & entry : audit.streams) {
    const StampSeriesSummary bag = SummarizeStampSeries(entry.second.bag_stamps_ns);
    const StampSeriesSummary header = SummarizeStampSeries(entry.second.header_stamps_ns);
    output << Csv(entry.first) << ',' << Csv(entry.second.type) << ',' <<
      Csv(entry.second.serialization_format) << ',' << entry.second.messages << ',' <<
      (entry.second.payload_audited ? "1" : "0") << ',' <<
      (entry.second.payload_audited ?
    std::to_string(entry.second.invalid_payload) : "NA") << ',' <<
      OptionalDouble(bag.rate_hz) << ',' << bag.zero_or_invalid << ',' <<
      bag.duplicate << ',' << bag.nonmonotonic << ',' <<
      OptionalDouble(bag.minimum_gap_sec) << ',' <<
      OptionalDouble(bag.p50_gap_sec) << ',' << OptionalDouble(bag.p95_gap_sec) <<
      ',' << OptionalDouble(bag.p99_gap_sec) << ',' <<
      OptionalDouble(bag.maximum_gap_sec) << ',' <<
      OptionalDouble(bag.submillisecond_positive_gap_ratio) << ',' <<
      OptionalDouble(header.rate_hz) << ',' << header.zero_or_invalid << ',' <<
      header.duplicate << ',' << header.nonmonotonic << ',' <<
      OptionalDouble(header.minimum_gap_sec) << ',' <<
      OptionalDouble(header.p50_gap_sec) << ',' <<
      OptionalDouble(header.p95_gap_sec) << ',' <<
      OptionalDouble(header.p99_gap_sec) << ',' <<
      OptionalDouble(header.maximum_gap_sec) << ',' <<
      OptionalDouble(header.submillisecond_positive_gap_ratio) << ',' <<
      Csv(JoinCounts(entry.second.parent_frames)) << ',' <<
      Csv(JoinCounts(entry.second.child_frames)) << '\n';
  }
  FinishReport(&output, path);
}

void WriteMissingIntervals(
  const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "first_stamp_ns,last_stamp_ns,duration_sec,missing_samples,classification," <<
    "health_states,reason_codes\n";
  const StampPairingSummary pairing = PairExactStampMultisets(
    audit.raw_mocap_stamps_ns, audit.shadow_stamps_ns);
  std::optional<std::int64_t> first_shadow;
  std::optional<std::int64_t> last_shadow;
  for (const std::int64_t stamp_ns : audit.shadow_stamps_ns) {
    if (stamp_ns <= 0) {
      continue;
    }
    first_shadow = first_shadow.has_value() ?
      std::min(first_shadow.value(), stamp_ns) : stamp_ns;
    last_shadow = last_shadow.has_value() ?
      std::max(last_shadow.value(), stamp_ns) : stamp_ns;
  }
  for (const auto & interval : pairing.missing_intervals) {
    const double duration_sec = static_cast<double>(
      interval.last_stamp_ns - interval.first_stamp_ns) / 1000000000.0;
    const auto diagnostics = SummarizeIntervalDiagnostics(audit, interval);
    output << interval.first_stamp_ns << ',' << interval.last_stamp_ns << ',' <<
      std::setprecision(17) << duration_sec << ',' << interval.missing_samples <<
      ',' << ClassifyMissingInterval(
      audit, interval, first_shadow, last_shadow) << ',' <<
      Csv(JoinCounts(diagnostics.health_states)) << ',' <<
      Csv(JoinCounts(diagnostics.reason_codes)) << '\n';
  }
  FinishReport(&output, path);
}

void WriteOrphanShadows(
  const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "header_stamp_ns,bag_stamp_ns,bag_minus_header_sec,evidence_position\n";
  const StampPairingSummary pairing = PairExactStampMultisets(
    audit.raw_mocap_stamps_ns, audit.shadow_stamps_ns);
  for (const auto & record : BuildOrphanShadowRecords(audit, pairing)) {
    std::optional<double> residual_sec;
    if (record.bag_stamp_ns.has_value()) {
      residual_sec = static_cast<double>(
        record.bag_stamp_ns.value() - record.header_stamp_ns) / 1000000000.0;
    }
    output << record.header_stamp_ns << ',' << OptionalInteger(record.bag_stamp_ns) <<
      ',' << OptionalDouble(residual_sec) << ',' << record.evidence_position << '\n';
  }
  FinishReport(&output, path);
}

void WriteCounterGroup(
  std::ofstream * output,
  const std::string & source,
  const std::map<std::string, std::vector<std::uint64_t>> & counters)
{
  for (const auto & entry : counters) {
    const CounterSeriesSummary summary = SummarizeCounterSeries(entry.second);
    *output << source << ',' << entry.first << ',' << summary.observations << ',' <<
      summary.epochs << ',' << summary.resets << ',' <<
      OptionalInteger(summary.first) << ',' << OptionalInteger(summary.last) << ',' <<
      OptionalInteger(summary.delta_without_reset) << ',' <<
      summary.accumulated_increase << '\n';
  }
}

void WriteDiagnosticStatuses(
  const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "name,samples,bag_rate_hz,bag_zero,bag_duplicate,bag_nonmonotonic," <<
    "header_rate_hz,header_zero,header_duplicate,header_nonmonotonic," <<
    "header_gap_p95_sec,header_gap_max_sec,duplicate_keys,levels,messages," <<
    "hardware_ids\n";
  for (const auto & entry : audit.diagnostic_statuses) {
    const StampSeriesSummary bag = SummarizeStampSeries(entry.second.bag_stamps_ns);
    const StampSeriesSummary header = SummarizeStampSeries(
      entry.second.header_stamps_ns);
    output << Csv(entry.first) << ',' << entry.second.samples << ',' <<
      OptionalDouble(bag.rate_hz) << ',' << bag.zero_or_invalid << ',' <<
      bag.duplicate << ',' << bag.nonmonotonic << ',' <<
      OptionalDouble(header.rate_hz) << ',' << header.zero_or_invalid << ',' <<
      header.duplicate << ',' << header.nonmonotonic << ',' <<
      OptionalDouble(header.p95_gap_sec) << ',' <<
      OptionalDouble(header.maximum_gap_sec) << ',' <<
      entry.second.duplicate_keys << ',' <<
      Csv(JoinByteCounts(entry.second.levels)) << ',' <<
      Csv(JoinCounts(entry.second.messages)) << ',' <<
      Csv(JoinCounts(entry.second.hardware_ids)) << '\n';
  }
  FinishReport(&output, path);
}

void WriteDiagnosticCounters(
  const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "source,key,observations,epochs,resets,first_left_censored,last," <<
    "delta_without_reset,accumulated_increase\n";
  WriteCounterGroup(&output, "mocap", audit.mocap_diagnostics.counters);
  WriteCounterGroup(
    &output, "aligned_imu", audit.aligned_imu_diagnostics.counters);
  WriteCounterGroup(&output, "runtime", audit.runtime_diagnostics.counters);
  FinishReport(&output, path);
}

void WriteFindings(const fs::path & path, const detail::AuditData & audit)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  output << "severity,code,detail\n";
  for (const auto & finding : audit.findings) {
    output << detail::ToString(finding.severity) << ',' << Csv(finding.code) << ',' <<
      Csv(finding.detail) << '\n';
  }
  FinishReport(&output, path);
}

void WriteSummary(
  const fs::path & path,
  const fs::path & bag_uri,
  const detail::AuditData & audit,
  const RuntimeContractVerdict verdict)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + path.string());
  }
  const StampPairingSummary mocap = PairExactStampMultisets(
    audit.raw_mocap_stamps_ns, audit.shadow_stamps_ns);
  const auto orphan_positions = CountOrphanPositions(
    BuildOrphanShadowRecords(audit, mocap));
  const StampPairingSummary vo = PairExactStampMultisets(
    audit.odometry_stamps_ns, audit.visual_status_stamps_ns);
  const auto evidence_window = PinnedEvidenceWindow();
  output << "schema_version=" << kAnalysisContractId << '\n';
  output << "analyzer_version=" << kAnalyzerVersion << '\n';
  output << "analyzer_source_revision=" <<
    BAG_CONTRACT_PROBE_SOURCE_REVISION << '\n';
  output << "bag_uri=" << bag_uri.generic_string() << '\n';
  output << "analysis_completion=COMPLETE\n";
  output << "bag_directory_snapshot_unchanged=PASS\n";
  output << "sha256_integrity=NOT_EVALUATED_USE_EXTERNAL_PRECHECK\n";
  output << "expected_metadata_sha256=" << kExpectedMetadataSha256 << '\n';
  output << "expected_rosbag2_0_db3_sha256=" << kExpectedDatabaseSha256 << '\n';
  output << "expected_bag_duration_ns=" << kExpectedBagDurationNs << '\n';
  output << "expected_bag_message_count=" << kExpectedBagMessageCount << '\n';
  output << "pinned_evidence_window_first_bag_stamp_ns=" <<
    evidence_window.first << '\n';
  output << "pinned_evidence_window_last_bag_stamp_ns=" <<
    evidence_window.second << '\n';
  output << "pinned_evidence_window_duration_sec=" << OptionalDouble(
    static_cast<double>(evidence_window.second - evidence_window.first) / 1.0e9) << '\n';
  output << "evidence_edge_gap_limit_sec=" <<
    OptionalDouble(kMaximumEvidenceEdgeGapSec) << '\n';
  output << "runtime_contract=" << ToString(verdict) << '\n';
  output << "cross_source_analysis=NOT_AUTHORIZED\n";
  output << "trajectory_accuracy=NOT_AUTHORIZED\n";
  output << "flight_authorization=DENIED\n";
  output << "messages_read=" << audit.total_messages << '\n';
  output << "raw_mocap_valid_stamps=" << mocap.valid_raw_stamps << '\n';
  output << "shadow_valid_stamps=" << mocap.valid_shadow_stamps << '\n';
  output << "raw_shadow_matched=" << mocap.matched << '\n';
  output << "raw_without_shadow=" << mocap.raw_without_shadow << '\n';
  output << "shadow_without_raw=" << mocap.shadow_without_raw << '\n';
  output << "shadow_orphan_positions=" << JoinCounts(orphan_positions) << '\n';
  output << "raw_shadow_coverage=" << OptionalDouble(mocap.raw_coverage_ratio) << '\n';
  output << "raw_shadow_value_mismatches=" <<
    audit.pose_pairs.value_mismatches << '\n';
  output << "shadow_contract_mismatches=" <<
    audit.shadow_contract_mismatches << '\n';
  output << "imu_paired=" << audit.imu_pairs.paired << '\n';
  output << "imu_payload_mismatches=" << audit.imu_pairs.payload_mismatches << '\n';
  output << "imu_raw_without_aligned=" <<
    audit.imu_pairs.pending_raw_by_aligned_stamp.size() << '\n';
  output << "imu_aligned_without_raw=" << audit.imu_pairs.pending_aligned.size() << '\n';
  output << "imu_expected_offset_ns=" << detail::kExpectedImuOffsetNs << '\n';
  output << "odometry_status_exact_matches=" << vo.matched << '\n';
  output << "odometry_without_status=" << vo.raw_without_shadow << '\n';
  output << "status_without_odometry=" << vo.shadow_without_raw << '\n';
  output << "vo_states=" << JoinByteCounts(audit.vo_states) << '\n';
  output << "mocap_health_states=" <<
    JoinCounts(audit.mocap_diagnostics.health_states) << '\n';
  output << "mocap_reason_codes=" << JoinCounts(audit.mocap_diagnostics.reasons) << '\n';
  output << "timesync_round_trip_ms_min=" <<
    OptionalDouble(audit.timesync_round_trip_ms.minimum) << '\n';
  output << "timesync_round_trip_ms_mean=" <<
    OptionalDouble(audit.timesync_round_trip_ms.Mean()) << '\n';
  output << "timesync_round_trip_ms_max=" <<
    OptionalDouble(audit.timesync_round_trip_ms.maximum) << '\n';
  output << "timesync_observed_offset_ns_min=" <<
    OptionalDouble(audit.timesync_observed_offset_ns.minimum) << '\n';
  output << "timesync_observed_offset_ns_mean=" <<
    OptionalDouble(audit.timesync_observed_offset_ns.Mean()) << '\n';
  output << "timesync_observed_offset_ns_max=" <<
    OptionalDouble(audit.timesync_observed_offset_ns.maximum) << '\n';
  output << "timesync_estimated_offset_ns_min=" <<
    OptionalDouble(audit.timesync_estimated_offset_ns.minimum) << '\n';
  output << "timesync_estimated_offset_ns_mean=" <<
    OptionalDouble(audit.timesync_estimated_offset_ns.Mean()) << '\n';
  output << "timesync_estimated_offset_ns_max=" <<
    OptionalDouble(audit.timesync_estimated_offset_ns.maximum) << '\n';
  output << "timesync_offset_innovation_ns_min=" <<
    OptionalDouble(audit.timesync_offset_innovation_ns.minimum) << '\n';
  output << "timesync_offset_innovation_ns_mean=" <<
    OptionalDouble(audit.timesync_offset_innovation_ns.Mean()) << '\n';
  output << "timesync_offset_innovation_ns_max=" <<
    OptionalDouble(audit.timesync_offset_innovation_ns.maximum) << '\n';
  output << "timesync_estimated_step_ns_min=" <<
    OptionalDouble(audit.timesync_estimated_step_ns.minimum) << '\n';
  output << "timesync_estimated_step_ns_mean=" <<
    OptionalDouble(audit.timesync_estimated_step_ns.Mean()) << '\n';
  output << "timesync_estimated_step_ns_max=" <<
    OptionalDouble(audit.timesync_estimated_step_ns.maximum) << '\n';
  output << "finding_count=" << audit.findings.size() << '\n';
  output << "limitation_1=VRPN stamps are Jetson callback time, not optical capture time\n";
  output << "limitation_2=rosbag does not re-prove live DDS publisher identity or GID\n";
  output << "limitation_3=T[base_link,camera_link] is not approved\n";
  output << "limitation_4=T[mocap_world,odom] is not approved\n";
  output << "limitation_5=direct pose, axis, yaw, ATE, RPE, and latency comparison is forbidden\n";
  FinishReport(&output, path);
}

AnalysisOutcome WriteReports(
  const fs::path & report_directory,
  const fs::path & bag_uri,
  const detail::AuditData & audit,
  const RuntimeContractVerdict verdict)
{
  fs::path staging_directory = report_directory;
  staging_directory += ".incomplete";
  if (PathEntryExists(staging_directory) ||
    !fs::create_directory(staging_directory))
  {
    throw std::runtime_error(
            "cannot create clean staging directory " + staging_directory.string());
  }

  AnalysisOutcome outcome;
  outcome.runtime_contract_verdict = verdict;
  outcome.messages_read = audit.total_messages;
  outcome.finding_count = audit.findings.size();
  outcome.summary_path = report_directory / "analysis_summary.txt";
  outcome.topic_statistics_path = report_directory / "topic_statistics.csv";
  outcome.missing_intervals_path =
    report_directory / "mocap_missing_intervals.csv";
  outcome.orphan_shadows_path =
    report_directory / "mocap_orphan_shadows.csv";
  outcome.diagnostic_statuses_path =
    report_directory / "diagnostic_statuses.csv";
  outcome.diagnostic_counters_path =
    report_directory / "diagnostic_counters.csv";
  outcome.findings_path = report_directory / "findings.csv";
  try {
    WriteTopicStatistics(
      staging_directory / outcome.topic_statistics_path.filename(), audit);
    WriteMissingIntervals(
      staging_directory / outcome.missing_intervals_path.filename(), audit);
    WriteOrphanShadows(
      staging_directory / outcome.orphan_shadows_path.filename(), audit);
    WriteDiagnosticStatuses(
      staging_directory / outcome.diagnostic_statuses_path.filename(), audit);
    WriteDiagnosticCounters(
      staging_directory / outcome.diagnostic_counters_path.filename(), audit);
    WriteFindings(staging_directory / outcome.findings_path.filename(), audit);
    WriteSummary(
      staging_directory / outcome.summary_path.filename(), bag_uri, audit, verdict);
    if (PathEntryExists(report_directory)) {
      throw std::runtime_error("report directory appeared during analysis");
    }
    RenameNoReplace(staging_directory, report_directory);
  } catch (...) {
    std::error_code cleanup_error;
    fs::remove_all(staging_directory, cleanup_error);
    throw;
  }
  return outcome;
}

}  // namespace

AnalysisOutcome AnalyzeBag(const AnalysisRequest & request)
{
  const auto paths = ValidatePaths(request);
  const fs::path & bag_uri = paths.first;
  const fs::path & report_directory = paths.second;
  const BagSnapshot before = SnapshotBagDirectory(bag_uri);
  detail::AuditData audit;
  std::exception_ptr analysis_error;

  try {
    {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_uri.string();
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      rosbag2_cpp::Reader reader;
      reader.open(storage_options, converter_options);
      RegisterAndValidateTopics(reader.get_all_topics_and_types(), &audit);

      while (reader.has_next()) {
        const auto message = reader.read_next();
        try {
          DispatchMessage(message, &audit);
        } catch (const EvidenceContractError &) {
          throw;
        } catch (const std::exception & error) {
          throw std::runtime_error(
                  "failed to analyze topic " + message->topic_name +
                  " at bag stamp " + std::to_string(message->time_stamp) +
                  ": " + error.what());
        }
      }
    }
  } catch (...) {
    analysis_error = std::current_exception();
  }

  const BagSnapshot after = SnapshotBagDirectory(bag_uri);
  if (before != after) {
    throw std::runtime_error("rosbag directory changed during read-only analysis");
  }
  if (analysis_error) {
    std::rethrow_exception(analysis_error);
  }
  FinalizeAudit(&audit);
  const RuntimeContractVerdict verdict = DetermineVerdict(audit);
  return WriteReports(report_directory, bag_uri, audit, verdict);
}

std::string ToString(const RuntimeContractVerdict verdict)
{
  switch (verdict) {
    case RuntimeContractVerdict::kPass:
      return "PASS";
    case RuntimeContractVerdict::kReviewRequired:
      return "REVIEW_REQUIRED";
    case RuntimeContractVerdict::kFail:
      return "FAIL";
  }
  return "FAIL";
}

}  // namespace bag_contract_probe
