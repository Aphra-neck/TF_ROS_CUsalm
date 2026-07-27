# bag_contract_probe

`bag_contract_probe` is a read-only C++17 audit tool for the versioned
`dual_shadow_transport_audit_20260723_v1` evidence contract. It opens a ROS 2
SQLite bag directly with `rosbag2_cpp::Reader`; it never starts a ROS node,
joins DDS, publishes messages, replays data, or writes inside the bag URI.

## Accepted input

`--bag` must point directly to the directory containing `metadata.yaml`, not to
the surrounding evidence directory. The v1 contract requires the exact ten
topics, message types, and message counts recorded by the 2026-07-23 dual-source
soak test. It also pins metadata start time, duration, message count, SQLite
layout, and the single storage filename. The 1,800,000-message ceiling is a
contract scope guard for this pinned bag and bounds retained per-message audit
state; it is not a general process-memory guarantee.

`--output` is required and must resolve outside the bag directory. Reports are
created only after the full bag has been read successfully and the bag's
recursive directory entries, sizes, modification times, and permissions have
been shown to remain unchanged. The output directory and its `.incomplete`
staging sibling must not already exist. The complete report bundle is staged
and then published with Linux `RENAME_NOREPLACE`, with the summary written
last. A destination that appears concurrently causes a fail-closed error
instead of replacement.

The directory snapshot is not a content hash. Run the pinned `sha256sum -c`
check before and after the analyzer. The summary deliberately reports SHA-256
integrity as not evaluated by this executable.

The input must be quiescent: stop the recorder and do not modify, replace, or
copy files into the bag directory while analysis is running. Pre/post hashes
and file snapshots detect endpoint changes, not every possible concurrent
in-place mutation during the read.

## Reports

- `analysis_summary.txt`: stable contract verdict and explicit limitations;
- `topic_statistics.csv`: bag/header rates, gap percentiles, stamps, frames;
- `mocap_missing_intervals.csv`: raw-without-shadow intervals with health states and reasons;
- `mocap_orphan_shadows.csv`: shadow-without-raw header/bag stamps and edge position;
- `diagnostic_statuses.csv`: exact status cadence, levels, messages, hardware IDs;
- `diagnostic_counters.csv`: left-censored first values, deltas, and resets;
- `findings.csv`: `OBSERVED`, `REVIEW`, and `FAIL` findings.

The tool audits exact raw-to-shadow stamps and identity-assumption values, the
FCU IMU relay's fixed `1,737,987 ns` offset and copied payload, cuVSLAM
odometry/status pairing, `vo_state`, timesync continuity, and selected runtime
diagnostic contracts. The mocap, aligned IMU, and calibrated runtime diagnostic
streams are checked for identity, fixed metadata, cadence, health level,
counter consistency, and resets. Safety counters that are already nonzero at
the left-censored recording boundary remain fail-level evidence, but they are
reported once as preexisting rather than miscounted as repeated static-contract
mismatches. Missing shadow intervals are classified as
startup edge, teardown edge, healthy, nonhealthy, mixed-health, or unknown.
Continuous source topics and all three required diagnostic statuses must cover
the pinned recording window to within 2.5 seconds at each edge. Shadow output
and `/tf_static` are intentionally excluded from that edge gate because shadow
warmup/recovery and transient-local snapshots are measured rather than assumed.
The `/tf` and `/tf_static` rows cover only topic/type/count and bag-time
statistics; transform payloads are not deserialized or audited.

It deliberately reports both `cross_source_analysis` and
`trajectory_accuracy` as `NOT_AUTHORIZED`. The current evidence does not
approve `T[base_link,camera_link]`, `T[mocap_world,odom]`, or VRPN optical
capture time. The tool therefore cannot output ATE, RPE, axis/yaw error,
sensor latency, ENU/NED alignment, flight readiness, or PX4 authorization.

## Build and run

```bash
cd /workspaces/isaac_ros-dev
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-select bag_contract_probe \
  --cmake-clean-cache \
  --event-handlers console_direct+

source install/setup.bash
colcon test \
  --packages-select bag_contract_probe \
  --return-code-on-test-failure \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base build/bag_contract_probe \
  --verbose

ros2 run bag_contract_probe dual_shadow_probe \
  --bag /workspaces/isaac_ros-dev/bags/dual_shadow_20260723_030651/rosbag2 \
  --output /workspaces/isaac_ros-dev/bags/dual_shadow_20260723_030651/analysis_v1
```

`ROS_DOMAIN_ID` is irrelevant because the executable does not initialize ROS
or use DDS. Exit code `0` means analysis completed with `PASS` or
`REVIEW_REQUIRED`; `2` means the evidence violated a fail-level contract, and
`1` means the tool could not complete the analysis.

For the pinned evidence, use the literal version-controlled digests below. The
adjacent `SHA256SUMS.txt` is useful as a convenience copy, but is not the trust
anchor for this contract.

```bash
EVIDENCE=/workspaces/isaac_ros-dev/bags/dual_shadow_20260723_030651
BAG="$EVIDENCE/rosbag2"
OUT="$EVIDENCE/analysis_v1"

(
  set -eo pipefail
  verify_evidence()
  {
    printf '%s  %s\n' \
      '938ae8784f3401c057d27d904b21dcd71b9d1e092b45fe970efb07c45f303dd4' \
      "$BAG/metadata.yaml" \
      '37c0cf891789dba0072811d62a3b96b3f0d0a89a9df2c29ac5fcb780c4219cec' \
      "$BAG/rosbag2_0.db3" | sha256sum -c -
  }

  verify_evidence
  if [[ -e "$OUT" || -L "$OUT" || -e "$OUT.incomplete" || -L "$OUT.incomplete" ]]; then
    echo "[STOP] choose a new versioned output directory"
    exit 1
  fi

  set +e
  ros2 run bag_contract_probe dual_shadow_probe \
    --bag "$BAG" \
    --output "$OUT"
  PROBE_RC=$?
  set -e

  verify_evidence
  REPORTS=(
    analysis_summary.txt
    topic_statistics.csv
    mocap_missing_intervals.csv
    mocap_orphan_shadows.csv
    diagnostic_statuses.csv
    diagnostic_counters.csv
    findings.csv
  )
  for REPORT in "${REPORTS[@]}"; do
    if [[ ! -f "$OUT/$REPORT" || -L "$OUT/$REPORT" ]]; then
      echo "[STOP] incomplete report bundle: $REPORT"
      printf 'probe_exit_code=%s\n' "$PROBE_RC"
      if (( PROBE_RC != 0 )); then
        exit "$PROBE_RC"
      fi
      exit 1
    fi
  done
  grep -E \
    'analysis_completion|runtime_contract|trajectory_accuracy|flight_authorization' \
    "$OUT/analysis_summary.txt"
  sed -n '1,160p' "$OUT/findings.csv"
  printf 'probe_exit_code=%s\n' "$PROBE_RC"
  exit "$PROBE_RC"
)
```

Use a new versioned output directory for every rerun. Do not overwrite or
delete a prior report bundle that has already been used as evidence.
