#!/usr/bin/env bash
set -Eeuo pipefail

# Gate 5 single-variable diagnostic sweep.
# Each weight gets a fresh Reactive/Oracle pair. Reactive always uses the
# frozen Gate 1 configuration; Oracle adds only the future publisher and the
# selected PredictionCritic weight. Evidence directories are never overwritten.

cd "$(dirname "$0")/../../.."
repo_root="$PWD"

difficulty='medium'
runs='1'
root='experiments/oracle_mppi/gate5/cost_sweep_20260829_01'
costs='0,10,50'
scenario='experiments/oracle_mppi/configs/scenarios/s2_gate5_conflict.yaml'
startup_timeout='110'
dynamic_startup_timeout='12'
contact_timeout='420'
settle_seconds='5.0'

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate5_cost_sweep.sh \
    [--scenario PATH] [--difficulty medium] [--runs N] [--costs 0,10,50] \
    [--root PATH]

The default runs one paired S2 Gate 5 conflict experiment for each
PredictionCritic cost_weight in 0, 10 and 50. Reactive is repeated for every
weight so every row has explicit paired evidence; its parameters are unchanged.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) scenario="$2"; shift 2 ;;
    --difficulty) difficulty="$2"; shift 2 ;;
    --runs) runs="$2"; shift 2 ;;
    --costs) costs="$2"; shift 2 ;;
    --root) root="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --dynamic-startup-timeout) dynamic_startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$runs" =~ ^[1-9][0-9]*$ ]] || {
  printf '%s\n' '--runs must be a positive integer' >&2
  exit 2
}
if [[ "$root" == /* || "$root" != experiments/oracle_mppi/gate5/* ||
      "$root" == *..* ]]; then
  printf 'Unsafe Gate 5 root: %s\n' "$root" >&2
  exit 2
fi
if [[ -e "$root" ]] && find "$root" -mindepth 1 -print -quit | grep -q .; then
  printf 'Refusing to overwrite non-empty evidence root: %s\n' "$root" >&2
  exit 2
fi

if [[ "$scenario" == /* || "$scenario" == *..* ||
      ! -f "$repo_root/$scenario" ]]; then
  printf 'Scenario must be an existing repository-relative file: %s\n' \
    "$scenario" >&2
  exit 2
fi
scenario_id="$(python3 - "$repo_root/$scenario" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    print(yaml.safe_load(stream)['scenario_id'])
PY
)"

mkdir -p "$root"
status_output="$root/matrix_status.csv"
printf 'scenario_id,cost_weight,method,run,label,exit_code,started_at,finished_at\n' \
  >"$status_output"
printf '%s\n' \
  "scenario_id: $scenario_id" \
  "scenario: $scenario" \
  "difficulty: $difficulty" \
  "cost_weights: [$costs]" \
  "runs_per_method_per_weight: $runs" \
  'reactive_nav2_params: experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml' \
  'oracle_publisher_config: experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml' \
  'oracle_nav2_params:' \
  '  0: experiments/oracle_mppi/configs/nav2_mppi_oracle_cost0_diagnostic.yaml' \
  '  10: experiments/oracle_mppi/configs/nav2_mppi_oracle_cost10_diagnostic.yaml' \
  '  50: experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml' \
  'fairness: same scenario/world/start/goal/difficulty; Oracle adds only future information' \
  >"$root/sweep_manifest.yaml"

oracle_params_for_weight() {
  case "$1" in
    0) printf '%s\n' 'experiments/oracle_mppi/configs/nav2_mppi_oracle_cost0_diagnostic.yaml' ;;
    10) printf '%s\n' 'experiments/oracle_mppi/configs/nav2_mppi_oracle_cost10_diagnostic.yaml' ;;
    50) printf '%s\n' 'experiments/oracle_mppi/configs/nav2_mppi_oracle_params.yaml' ;;
    *) printf 'Unsupported cost_weight: %s\n' "$1" >&2; return 2 ;;
  esac
}

run_one() {
  local weight="$1" method="$2" run_number="$3"
  local weight_dir="cost_$(printf '%03d' "$weight")"
  local label="$root/$scenario_id/$weight_dir/$method"_run_$(printf '%02d' "$run_number")
  local runner_log="$label.runner.log"
  local started_at finished_at exit_code nav2_params

  mkdir -p "$(dirname "$runner_log")"
  started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '[Gate 5 cost sweep] scenario=%s cost=%s method=%s run=%02d/%s\n' \
    "$scenario_id" "$weight" "$method" "$run_number" "$runs"
  if [[ "$method" == reactive ]]; then
    nav2_params='experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml'
  else
    nav2_params="$(oracle_params_for_weight "$weight")"
  fi

  set +e
  if [[ "$method" == reactive ]]; then
    "$repo_root/experiments/oracle_mppi/scripts/run_gate2_scene.sh" \
      --scenario "$repo_root/$scenario" --difficulty "$difficulty" \
      --profile reactive_mppi_static --nav2-params "$nav2_params" \
      --expected-control-period 0.1 --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" \
      --dynamic-startup-timeout "$dynamic_startup_timeout" \
      --contact-timeout "$contact_timeout" --label "$label" \
      >"$runner_log" 2>&1
  else
    "$repo_root/experiments/oracle_mppi/scripts/run_gate2_scene.sh" \
      --scenario "$repo_root/$scenario" --difficulty "$difficulty" \
      --profile oracle_mppi_prediction --nav2-params "$nav2_params" \
      --expected-control-period 0.1 --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" \
      --dynamic-startup-timeout "$dynamic_startup_timeout" \
      --contact-timeout "$contact_timeout" \
      --oracle-scenario "$repo_root/$scenario" \
      --oracle-publisher-config experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml \
      --label "$label" \
      >"$runner_log" 2>&1
  fi
  exit_code=$?
  set -e
  finished_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$scenario_id" "$weight" "$method" "$run_number" "$label" "$exit_code" \
    "$started_at" "$finished_at" >>"$status_output"

  if [[ -d "$repo_root/$label" ]]; then
    mv "$runner_log" "$repo_root/$label/runner.log" 2>/dev/null || true
    if [[ -f "$repo_root/$label/dynamic_groundtruth.csv" ]]; then
      python3 "$repo_root/experiments/oracle_mppi/scripts/plot_gate2_dynamic_run.py" \
        --run "$repo_root/$label" >"$repo_root/$label/plot_gate2.log" 2>&1 || true
      python3 "$repo_root/experiments/oracle_mppi/scripts/analyze_gate5_run.py" \
        --run "$repo_root/$label" --output gate5_timeline.png \
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
for weight in $(printf '%s' "$costs" | tr ',' ' '); do
  [[ "$weight" =~ ^[0-9]+$ ]] || {
    printf 'Invalid cost_weight: %s\n' "$weight" >&2
    exit 2
  }
  oracle_params_for_weight "$weight" >/dev/null
  for method in reactive oracle; do
    for run_number in $(seq 1 "$runs"); do
      run_one "$weight" "$method" "$run_number"
      exit_code="$(awk -F',' -v s="$scenario_id" -v c="$weight" \
        -v m="$method" -v r="$run_number" \
        'NR > 1 && $1 == s && $2 == c && $3 == m && $4 == r {print $6}' \
        "$status_output" | tail -1)"
      if [[ "$exit_code" != 0 ]]; then
        matrix_failures=$((matrix_failures + 1))
      fi
    done
  done
done

summary_output="$root/gate5_smoke_summary.csv"
python3 "$repo_root/experiments/oracle_mppi/scripts/summarize_gate5.py" \
  --root "$root" --output "$summary_output" --status "$status_output" \
  --pairs-output "$root/gate5_paired_summary.csv" \
  >"$root/summarize.log" 2>&1 || {
    matrix_failures=$((matrix_failures + 1))
  }

printf 'Gate 5 cost sweep complete: failures=%s status=%s summary=%s\n' \
  "$matrix_failures" "$status_output" "$summary_output"
exit "$matrix_failures"
