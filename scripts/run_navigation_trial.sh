#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

goal_x=''
goal_y='0.0'
goal_yaw='0.0'
label='navigation_trial'
output_dir='/workspaces/rtabmap_tb3_nav/results'

usage() {
  cat <<'EOF'
Usage:
  scripts/run_navigation_trial.sh --x X [--y Y] [--yaw RAD] [--label NAME]

The running demo must already be publishing /odom, /map and Nav2's
navigate_to_pose action. Artifacts are written to results/ inside the project:
trajectory.png, trajectory.csv and metrics.yaml.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --x) goal_x="$2"; shift 2 ;;
    --y) goal_y="$2"; shift 2 ;;
    --yaw) goal_yaw="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$goal_x" ]]; then
  usage >&2
  exit 2
fi

command_text="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav navigation_trial.py --x ${goal_x} --y ${goal_y} --yaw ${goal_yaw} --label ${label} --output-dir ${output_dir}"
printf -v quoted '%q' "$command_text"

if docker info >/dev/null 2>&1; then
  docker compose exec -T ros2 bash -lc "$command_text"
else
  sg docker -c "docker compose exec -T ros2 bash -lc $quoted"
fi

printf 'Artifacts: results/%s/{trajectory.png,trajectory.csv,metrics.yaml}\n' "$label"
