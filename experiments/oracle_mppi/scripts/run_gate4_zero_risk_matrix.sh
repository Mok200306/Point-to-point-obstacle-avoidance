#!/usr/bin/env bash
set -Eeuo pipefail

# Run the Gate 4 zero-risk static regression. Reactive and Oracle use the same
# static world, goal, MPPI dynamics, and evidence runner; the only intended
# difference is the Oracle publisher plus PredictionCritic profile.

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
repo_root="$PWD"

runs='3'
start_run='1'
root='experiments/oracle_mppi/gate4/zero_risk'
settle_seconds='5.0'
startup_timeout='110'
contact_timeout='300'

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate4_zero_risk_matrix.sh \
    [--runs N] [--start-run N] [--root PATH]
    [--settle-seconds SEC] [--startup-timeout SEC] [--contact-timeout SEC]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs) runs="$2"; shift 2 ;;
    --start-run) start_run="$2"; shift 2 ;;
    --root) root="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$runs" =~ ^[1-9][0-9]*$ ]] || {
  printf '%s\n' '--runs must be a positive integer' >&2
  exit 2
}
[[ "$start_run" =~ ^[1-9][0-9]*$ ]] || {
  printf '%s\n' '--start-run must be a positive integer' >&2
  exit 2
}
if [[ "$root" == /* ]]; then
  case "$root" in
    "$repo_root"/*) root="${root#"$repo_root/"}" ;;
    *) printf 'Root must be inside repository: %s\n' "$root" >&2; exit 2 ;;
  esac
else
  root="${root#./}"
fi
if [[ "$root" != experiments/oracle_mppi/gate4/* || "$root" == *..* ]]; then
  printf 'Unsafe Gate 4 root: %s\n' "$root" >&2
  exit 2
fi
mkdir -p "$root"

status_file="$root/matrix_status.csv"
printf 'method,run,label,exit_code\n' >"$status_file"
failed=0

run_one() {
  local method="$1" run_number="$2" label exit_code
  label="$root/${method}_run_$(printf '%02d' "$run_number")"
  printf '\n===== %s run %02d/%s =====\n' "$method" "$run_number" "$runs"
  set +e
  if [[ "$method" == reactive ]]; then
    ./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
      --start-x -8.5 --start-y 0.0 --x 8.5 --y 0.0 --yaw 0.0 \
      --profile reactive_mppi_static \
      --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
      --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" --contact-timeout "$contact_timeout" \
      --label "$label"
    exit_code=$?
  else
    ./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
      --start-x -8.5 --start-y 0.0 --x 8.5 --y 0.0 --yaw 0.0 \
      --profile oracle_mppi_prediction \
      --nav2-params experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml \
      --oracle-scenario experiments/oracle_mppi/configs/gate4/s0_zero_risk.yaml \
      --oracle-publisher-config experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml \
      --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" --contact-timeout "$contact_timeout" \
      --label "$label"
    exit_code=$?
  fi
  set -e
  printf '%s,%s,%s,%s\n' "$method" "$run_number" "$label" "$exit_code" >>"$status_file"
  if [[ "$exit_code" -ne 0 ]]; then
    failed=$((failed + 1))
    printf 'FAILED: %s\n' "$label" >&2
  else
    printf 'PASSED: %s\n' "$label"
  fi
}

for method in reactive oracle; do
  for ((run_number = start_run; run_number < start_run + runs; run_number++)); do
    run_one "$method" "$run_number"
  done
done

set +e
python3 experiments/oracle_mppi/scripts/summarize_gate4.py \
  --root "$root" --output "$root/zero_risk_summary.csv"
summary_exit=$?
set -e
if [[ "$summary_exit" -ne 0 ]]; then
  failed=$((failed + 1))
fi

printf '\nGate 4 matrix complete: failed=%s summary_exit=%s status=%s\n' \
  "$failed" "$summary_exit" "$status_file"
exit "$failed"
