#!/usr/bin/env bash
set -Eeuo pipefail

# Gate 0 matrix: A->B and B->A, three isolated runs per direction.
# Complete run directories are reused after validation. Incomplete directories
# are moved to gate0/runner_audit before a new run is started.

cd "$(dirname "$0")/../../.."
repo_root="$PWD"
profile='adaptive_goal_line_045'
settle_seconds='5.0'
startup_timeout='75'
contact_timeout='420'
rerun_all=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile) profile="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --rerun-all) rerun_all=true; shift ;;
    -h|--help)
      printf '%s\n' \
        'Usage: experiments/oracle_mppi/scripts/run_gate0_matrix.sh' \
        '[--profile NAME] [--settle-seconds SEC] [--rerun-all]'
      exit 0
      ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

runner='experiments/oracle_mppi/scripts/run_gate0_leg.sh'
summary='experiments/oracle_mppi/gate0/baseline_rpp_static.csv'
audit_root='experiments/oracle_mppi/gate0/runner_audit'
mkdir -p "$(dirname "$summary")" "$audit_root"
printf '%s\n' 'case,run,start_x_m,start_y_m,goal_x_m,goal_y_m,goal_yaw_rad,wrapper_exit,nav2_status,succeeded,wall_duration_s,simulation_duration_s,trajectory_length_m,gazebo_trajectory_length_m,final_xy_error_m,gazebo_non_ground_contact,gazebo_contact_pairs,rosbag_present' >"$summary"

is_complete() {
  local label="$1"
  local metrics="$repo_root/$label/metrics.yaml"
  local experiment="$repo_root/$label/experiment.yaml"
  [[ -f "$metrics" &&
     -f "$experiment" &&
     -d "$repo_root/$label/rosbag" &&
     -f "$repo_root/$label/trajectory_comparison.png" &&
     -f "$repo_root/$label/trajectory.csv" &&
     -f "$repo_root/$label/gazebo_trajectory.csv" &&
     -f "$repo_root/$label/nav2_rgbd_params.yaml" &&
     -f "$repo_root/$label/world.sdf" &&
     -f "$repo_root/$label/reproduce_command.sh" ]] || return 1
  grep -q '^nav2_status: 4$' "$metrics" || return 1
  grep -q '^succeeded: true$' "$metrics" || return 1
  grep -q '^gazebo_non_ground_contact: false$' "$metrics" || return 1
  grep -q '^trial_exit_code: 0$' "$experiment" || return 1
}

move_incomplete_to_audit() {
  local label="$1" case_name="$2" run_number="$3"
  local source="$repo_root/$label"
  [[ -d "$source" ]] || return 0
  local target="$repo_root/$audit_root/${case_name}_run_${run_number}_matrix_interrupted_$(date +%Y%m%d_%H%M%S)"
  mv "$source" "$target"
  printf 'Moved incomplete evidence to audit: %s\n' "$target"
}

archive_existing_for_rerun() {
  local label="$1" case_name="$2" run_number="$3"
  local source="$repo_root/$label"
  [[ -d "$source" ]] || return 0
  local target="$repo_root/$audit_root/${case_name}_run_${run_number}_pre_rerun_$(date +%Y%m%d_%H%M%S)_$$"
  mv "$source" "$target"
  printf 'Archived previous evidence for full rerun: %s\n' "$target"
}

append_run() {
  local case_name="$1" run_number="$2" start_x="$3" goal_x="$4" goal_yaw="$5"
  local label="experiments/oracle_mppi/gate0/$case_name/run_$run_number"
  local metrics="$repo_root/$label/metrics.yaml"
  local experiment="$repo_root/$label/experiment.yaml"
  local nav2_status='unknown' succeeded='unknown' wall='null' sim='null'
  local path='null' gazebo_path='null' error='null' contact='unknown' pairs='unknown'
  local wrapper_exit='unknown' bag='false'
  local runner_exit=0

  printf '\n===== %s run_%s =====\n' "$case_name" "$run_number"
  if [[ "$rerun_all" == true ]]; then
    archive_existing_for_rerun "$label" "$case_name" "$run_number"
  fi
  if [[ "$rerun_all" != true ]] && is_complete "$label"; then
    printf 'Reusing complete evidence: %s\n' "$label"
  else
    if [[ "$rerun_all" != true ]]; then
      move_incomplete_to_audit "$label" "$case_name" "$run_number"
    fi
    set +e
    "$runner" \
      --start-x "$start_x" --start-y 0.0 \
      --x "$goal_x" --y 0.0 --yaw "$goal_yaw" \
      --profile "$profile" --settle-seconds "$settle_seconds" \
      --startup-timeout "$startup_timeout" --contact-timeout "$contact_timeout" \
      --label "$label"
    runner_exit=$?
    set -e
  fi

  if [[ -f "$experiment" ]]; then
    wrapper_exit="$(awk -F': ' '$1 == "trial_exit_code" {print $2}' "$experiment" | tail -1)"
  fi
  if [[ -f "$metrics" ]]; then
    nav2_status="$(awk -F': ' '$1 == "nav2_status" {print $2}' "$metrics" | tail -1)"
    succeeded="$(awk -F': ' '$1 == "succeeded" {print $2}' "$metrics" | tail -1)"
    wall="$(awk -F': ' '$1 == "wall_duration_s" {print $2}' "$metrics" | tail -1)"
    sim="$(awk -F': ' '$1 == "simulation_duration_s" {print $2}' "$metrics" | tail -1)"
    path="$(awk -F': ' '$1 == "trajectory_length_m" {print $2}' "$metrics" | tail -1)"
    gazebo_path="$(awk -F': ' '$1 == "gazebo_trajectory_length_m" {print $2}' "$metrics" | tail -1)"
    error="$(awk -F': ' '$1 == "final_xy_error_m" {print $2}' "$metrics" | tail -1)"
    contact="$(awk -F': ' '$1 == "gazebo_non_ground_contact" {print $2}' "$metrics" | tail -1)"
    pairs="$(awk -F': ' '$1 == "gazebo_contact_pairs" {sub(/^"/, "", $2); sub(/"$/, "", $2); print $2}' "$metrics" | tail -1)"
  fi
  [[ -d "$repo_root/$label/rosbag" ]] && bag='true'

  printf '%s,%s,%s,0.0,%s,0.0,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"%s",%s\n' \
    "$case_name" "$run_number" "$start_x" "$goal_x" "$goal_yaw" \
    "$runner_exit" "$nav2_status" "$succeeded" "$wall" "$sim" "$path" \
    "$gazebo_path" "$error" "$contact" "$pairs" "$bag" >>"$summary"
  printf 'recorded: wrapper=%s nav2=%s succeeded=%s contact=%s bag=%s\n' \
    "$runner_exit" "$nav2_status" "$succeeded" "$contact" "$bag"
}

for run_number in 01 02 03; do
  append_run case_A_to_B "$run_number" -8.5 8.5 0.0
done
for run_number in 01 02 03; do
  append_run case_B_to_A "$run_number" 8.5 -8.5 3.141592653589793
done

successes="$(awk -F, 'NR > 1 && $8 == 0 && $9 == 4 && $10 == "true" && $16 == "false" && $18 == "true" {count++} END {print count + 0}' "$summary")"
total="$(awk 'END {print NR - 1}' "$summary")"
failures=$((total - successes))
printf '\nGate 0 matrix complete: successes=%s total=%s failures=%s\n' "$successes" "$total" "$failures"
printf 'summary=%s\n' "$summary"
if [[ "$failures" -ne 0 ]]; then
  exit 5
fi
