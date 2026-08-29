#!/usr/bin/env bash
set -Eeuo pipefail

# Gate 5 paired smoke matrix.  Each Reactive/Oracle run uses the same scenario
# YAML, world, start/goal, difficulty and frozen MPPI snapshots.  The Oracle
# side differs only by starting the Gate 3 publisher and selecting the Gate 4
# PredictionCritic profile.  Evidence directories are never overwritten.

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
repo_root="$PWD"

difficulty='medium'
runs='1'
root='experiments/oracle_mppi/gate5/smoke'
status_output=''
startup_timeout='110'
dynamic_startup_timeout='12'
contact_timeout='420'
settle_seconds='5.0'
goal_timeout_seconds='300'
oracle_publisher_config='experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml'

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate5_smoke_matrix.sh \
    [--difficulty medium] [--runs N] [--root PATH] \
    [--oracle-publisher-config PATH] [--goal-timeout-seconds SEC]

The default smoke matrix runs:
  S1 Reactive, S1 Oracle, S2 Reactive, S2 Oracle (one run each)

Use --runs 5 for the task-book Gate 5 functional batch after smoke passes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --difficulty) difficulty="$2"; shift 2 ;;
    --runs) runs="$2"; shift 2 ;;
    --root) root="$2"; shift 2 ;;
    --status-output) status_output="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --dynamic-startup-timeout) dynamic_startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --goal-timeout-seconds) goal_timeout_seconds="$2"; shift 2 ;;
    --oracle-publisher-config) oracle_publisher_config="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$oracle_publisher_config" in
  /*)
    case "$oracle_publisher_config" in
      "$repo_root"/*) oracle_publisher_config="${oracle_publisher_config#"$repo_root/"}" ;;
      *) printf 'Oracle publisher config must be inside repository: %s\n' \
        "$oracle_publisher_config" >&2; exit 2 ;;
    esac
    ;;
  *) oracle_publisher_config="${oracle_publisher_config#./}" ;;
esac
if [[ "$oracle_publisher_config" != experiments/oracle_mppi/configs/* ||
      ! -f "$repo_root/$oracle_publisher_config" ]]; then
  printf 'Oracle publisher config must be an existing repository config: %s\n' \
    "$oracle_publisher_config" >&2
  exit 2
fi

[[ "$runs" =~ ^[1-9][0-9]*$ ]] || {
  printf '%s\n' '--runs must be a positive integer' >&2
  exit 2
}
case "$root" in
  /*)
    case "$root" in
      "$repo_root"/*) root="${root#"$repo_root/"}" ;;
      *) printf 'Root must be inside repository: %s\n' "$root" >&2; exit 2 ;;
    esac
    ;;
  *) root="${root#./}" ;;
esac
if [[ "$root" != experiments/oracle_mppi/gate5/* || "$root" == *..* ]]; then
  printf 'Unsafe Gate 5 root: %s\n' "$root" >&2
  exit 2
fi
mkdir -p "$root"

if [[ -z "$status_output" ]]; then
  status_output="$root/matrix_status.csv"
fi
case "$status_output" in
  experiments/oracle_mppi/gate5/*) ;;
  *) printf 'Unsafe status output: %s\n' "$status_output" >&2; exit 2 ;;
esac
if [[ -e "$status_output" ]]; then
  printf 'Refusing to overwrite status file: %s\n' "$status_output" >&2
  exit 2
fi
printf 'scenario_id,method,run,label,exit_code,started_at,finished_at\n' \
  >"$status_output"

runner='experiments/oracle_mppi/scripts/run_gate2_scene.sh'
scenario_dir='experiments/oracle_mppi/configs/scenarios'

run_one() {
  local scenario_id="$1" scenario_file="$2" method="$3" run_number="$4"
  local scenario_path="$scenario_dir/$scenario_file"
  local label="$root/$scenario_id/${method}_run_$(printf '%02d' "$run_number")"
  local runner_log="$label.runner.log"
  local started_at finished_at exit_code

  mkdir -p "$(dirname "$runner_log")"
  started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '[Gate 5] %s %s run_%02d/%s\n' \
    "$scenario_id" "$method" "$run_number" "$runs"

  set +e
  if [[ "$method" == reactive ]]; then
    "$repo_root/$runner" \
      --scenario "$repo_root/$scenario_path" \
      --difficulty "$difficulty" \
      --profile reactive_mppi_static \
      --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
      --expected-control-period 0.1 \
      --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" \
      --dynamic-startup-timeout "$dynamic_startup_timeout" \
      --contact-timeout "$contact_timeout" \
      --goal-timeout-seconds "$goal_timeout_seconds" \
      --label "$label" \
      >"$runner_log" 2>&1
    exit_code=$?
  else
    "$repo_root/$runner" \
      --scenario "$repo_root/$scenario_path" \
      --difficulty "$difficulty" \
      --profile oracle_mppi_prediction \
      --nav2-params experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml \
      --expected-control-period 0.1 \
      --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" \
      --dynamic-startup-timeout "$dynamic_startup_timeout" \
      --contact-timeout "$contact_timeout" \
      --goal-timeout-seconds "$goal_timeout_seconds" \
      --oracle-scenario "$repo_root/$scenario_path" \
      --oracle-publisher-config "$oracle_publisher_config" \
      --label "$label" \
      >"$runner_log" 2>&1
    exit_code=$?
  fi
  set -e
  finished_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s,%s,%s,%s,%s,%s,%s\n' \
    "$scenario_id" "$method" "$run_number" "$label" "$exit_code" \
    "$started_at" "$finished_at" >>"$status_output"

  if [[ -d "$repo_root/$label" ]]; then
    mv "$runner_log" "$repo_root/$label/runner.log" 2>/dev/null || true
    if [[ -f "$repo_root/$label/dynamic_groundtruth.csv" ]]; then
      python3 "$repo_root/experiments/oracle_mppi/scripts/plot_gate2_dynamic_run.py" \
        --run "$repo_root/$label" \
        >"$repo_root/$label/plot_gate2.log" 2>&1 || true
      python3 "$repo_root/experiments/oracle_mppi/scripts/analyze_gate5_run.py" \
        --run "$repo_root/$label" \
        --output gate5_timeline.png \
        >"$repo_root/$label/analyze_gate5.log" 2>&1 || true
    fi
  fi
  if [[ "$exit_code" -eq 0 ]]; then
    printf '  result: PASS (runner-level checks)\n'
  else
    printf '  result: FAIL (evidence retained)\n'
  fi
}

matrix_failures=0
for scenario_spec in \
  'S1_crossing:s1_crossing.yaml' \
  'S2_oncoming:s2_oncoming.yaml'; do
  scenario_id="${scenario_spec%%:*}"
  scenario_file="${scenario_spec##*:}"
  for method in reactive oracle; do
    for ((run_number = 1; run_number <= runs; run_number++)); do
      run_one "$scenario_id" "$scenario_file" "$method" "$run_number" || true
      exit_code="$(awk -F',' -v s="$scenario_id" -v m="$method" -v r="$run_number" \
        'NR > 1 && $1 == s && $2 == m && $3 == r {print $5}' "$status_output" | tail -1)"
      if [[ "$exit_code" != 0 ]]; then
        matrix_failures=$((matrix_failures + 1))
      fi
    done
  done
done

summary_output="$root/gate5_smoke_summary.csv"
python3 "$repo_root/experiments/oracle_mppi/scripts/summarize_gate5.py" \
  --root "$root" --output "$summary_output" \
  --status "$status_output" \
  >"$root/summarize.log" 2>&1 || matrix_failures=$((matrix_failures + 1))

printf 'Gate 5 smoke matrix complete: failures=%s status=%s summary=%s\n' \
  "$matrix_failures" "$status_output" "$summary_output"
exit "$matrix_failures"
