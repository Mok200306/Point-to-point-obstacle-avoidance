#!/usr/bin/env bash
set -Eeuo pipefail

# Run the Gate 2 Reactive MPPI medium smoke matrix sequentially. Every run
# receives a unique evidence directory; failures are recorded and do not stop
# later scenarios from running. This script never publishes future obstacle
# information to Nav2.

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
repo_root="$PWD"

difficulty='medium'
run_prefix='medium'
start_run=1
runs_per_scenario=3
status_output='experiments/oracle_mppi/gate2/matrix_status_20260828.csv'
scenario_dir='experiments/oracle_mppi/configs/scenarios'
runner='experiments/oracle_mppi/scripts/run_gate2_scene.sh'

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate2_matrix.sh \
    [--difficulty medium] [--run-prefix medium] [--start-run 1] \
    [--runs-per-scenario 3] [--status-output PATH]

The default writes new results under:
  experiments/oracle_mppi/gate2/S{1..4}_{name}/medium_{01..03}/
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --difficulty) difficulty="$2"; shift 2 ;;
    --run-prefix) run_prefix="$2"; shift 2 ;;
    --start-run) start_run="$2"; shift 2 ;;
    --runs-per-scenario) runs_per_scenario="$2"; shift 2 ;;
    --status-output) status_output="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! [[ "$start_run" =~ ^[0-9]+$ ]] || (( start_run < 1 )); then
  printf '%s must be a positive integer\n' '--start-run' >&2
  exit 2
fi
if ! [[ "$runs_per_scenario" =~ ^[0-9]+$ ]] || (( runs_per_scenario < 1 )); then
  printf '%s must be a positive integer\n' '--runs-per-scenario' >&2
  exit 2
fi

case "$status_output" in
  experiments/oracle_mppi/gate2/*) ;;
  *) printf 'Unsafe status output: %s\n' "$status_output" >&2; exit 2 ;;
esac
mkdir -p "$(dirname "$status_output")"
if [[ -e "$status_output" ]]; then
  printf 'Refusing to overwrite status file: %s\n' "$status_output" >&2
  exit 2
fi
printf 'scenario_id,run,label,exit_code,started_at,finished_at\n' >"$status_output"

scenarios=(
  'S1_crossing:s1_crossing.yaml'
  'S2_oncoming:s2_oncoming.yaml'
  'S3_diagonal:s3_diagonal.yaml'
  'S4_stop_go:s4_stop_go.yaml'
)

matrix_failures=0
for scenario_spec in "${scenarios[@]}"; do
  scenario_id="${scenario_spec%%:*}"
  scenario_file="${scenario_spec##*:}"
  for ((offset = 0; offset < runs_per_scenario; offset++)); do
    run_number=$((start_run + offset))
    run_name="${run_prefix}_$(printf '%02d' "$run_number")"
    label="experiments/oracle_mppi/gate2/${scenario_id}/${run_name}"
    scenario_path="${scenario_dir}/${scenario_file}"
    started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    runner_log="${label}.runner.log"
    mkdir -p "$(dirname "$runner_log")"

    printf '[Gate 2] %s %s/%s\n' "$scenario_id" "$run_name" "$runs_per_scenario"
    set +e
    "$repo_root/$runner" \
      --scenario "$repo_root/$scenario_path" \
      --difficulty "$difficulty" \
      --label "$label" \
      >"$runner_log" 2>&1
    exit_code=$?
    set -e
    finished_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf '%s,%s,%s,%s,%s,%s\n' \
      "$scenario_id" "$run_name" "$label" "$exit_code" \
      "$started_at" "$finished_at" >>"$status_output"
    if [[ -d "${repo_root}/${label}" ]]; then
      if [[ -f "${repo_root}/${label}/dynamic_groundtruth.csv" ]]; then
        python3 "$repo_root/experiments/oracle_mppi/scripts/plot_gate2_dynamic_run.py" \
          --run "${repo_root}/${label}" \
          >"${repo_root}/${label}/plot.log" 2>&1 || true
      fi
      mv "$runner_log" "${repo_root}/${label}/runner.log" 2>/dev/null || true
    fi
    if [[ "$exit_code" -ne 0 ]]; then
      matrix_failures=$((matrix_failures + 1))
      printf '  result: FAIL (evidence retained)\n'
    else
      printf '  result: PASS (runner-level checks)\n'
    fi
  done
done

printf 'Gate 2 matrix complete: %d runner failures; status=%s\n' \
  "$matrix_failures" "$status_output"
exit "$matrix_failures"
