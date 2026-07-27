#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/start_mocap_primary_container.sh"

bounded()
{
  return 1
}

if runtime_health_check; then
  echo "expected runtime_health_check to fail" >&2
  exit 1
fi

if [[ "$runtime_health_failure" != "ROS node graph query timed out" ]]; then
  echo "unexpected health failure reason: ${runtime_health_failure}" >&2
  exit 1
fi

timeout()
{
  if [[ "$3" != "${DIAGNOSTIC_PROBE_TIMEOUT_SEC}s" ]]; then
    echo "unexpected diagnostic timeout: $3" >&2
    return 1
  fi
  printf 'diagnostic-sample\n'
}

output="$(capture_diagnostic localization_output_gateway)"
if [[ "$output" != "diagnostic-sample" ]]; then
  echo "diagnostic output was not returned" >&2
  exit 1
fi

echo "health_failure_reason=PASS"
echo "diagnostic_timeout=PASS"
