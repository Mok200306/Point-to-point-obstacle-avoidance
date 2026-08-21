#!/usr/bin/env bash
set -euo pipefail

# Run two sequential NavigateToPose goals in one live RGB-D session while
# recording contacts and the red/blue multi-goal trajectory evidence.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

goal_a_x=''
goal_a_y=''
goal_a_yaw='0.0'
goal_b_x=''
goal_b_y=''
goal_b_yaw='0.0'
label='多目标导航实验'
profile='adaptive_goal_line_045'
world_file='src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world'
contacts_topic='/gazebo/indoor_obstacle_course_large/physics/contacts'
contact_timeout='600'
startup_timeout='45'
settle_seconds='5'
contact_pid=''
contact_log=''

usage() {
  cat <<'EOF'
Usage:
  scripts/multi_goal_regression.sh \
    --goal-a-x X --goal-a-y Y --goal-b-x X --goal-b-y Y \
    [--goal-a-yaw RAD] [--goal-b-yaw RAD] [--label NAME]
    [--profile NAME] [--world-file PATH] [--contacts-topic TOPIC]
    [--settle-seconds SECONDS]

The current container must already be running with demo.launch.py. The two
goals are sent sequentially without resetting Gazebo or the RTAB-Map database.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --goal-a-x) goal_a_x="$2"; shift 2 ;;
    --goal-a-y) goal_a_y="$2"; shift 2 ;;
    --goal-a-yaw) goal_a_yaw="$2"; shift 2 ;;
    --goal-b-x) goal_b_x="$2"; shift 2 ;;
    --goal-b-y) goal_b_y="$2"; shift 2 ;;
    --goal-b-yaw) goal_b_yaw="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --profile) profile="$2"; shift 2 ;;
    --world-file) world_file="$2"; shift 2 ;;
    --contacts-topic) contacts_topic="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$goal_a_x" || -z "$goal_a_y" || -z "$goal_b_x" || -z "$goal_b_y" ]]; then
  usage >&2
  exit 2
fi

label="${label#results/}"

compose_exec() {
  local command_text="$1"
  local quoted
  local escaped="${command_text//\'/\'\\\'\'}"
  printf -v quoted "'%s'" "$escaped"
  if docker info >/dev/null 2>&1; then
    docker compose exec -T ros2 bash -lc "$command_text"
  else
    sg docker -c "docker compose exec -T ros2 bash -lc $quoted"
  fi
}

compose_ps() {
  if docker info >/dev/null 2>&1; then
    docker compose ps --status running --services | grep -qx ros2
  else
    sg docker -c 'docker compose ps --status running --services' | grep -qx ros2
  fi
}

stop_contact_listener() {
  if [[ -n "$contact_pid" ]]; then
    kill "$contact_pid" 2>/dev/null || true
    wait "$contact_pid" 2>/dev/null || true
    contact_pid=''
  fi
  local remote_pids
  remote_pids="$(compose_exec 'pgrep -x gz || true' 2>/dev/null || true)"
  if [[ -n "$remote_pids" ]]; then
    compose_exec "kill ${remote_pids}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_contact_listener
  if [[ -n "$contact_log" ]]; then
    rm -f -- "$contact_log"
  fi
}

trap cleanup EXIT INT TERM

compose_ps || {
  printf 'The ros2 container is not running. Start the demo first.\n' >&2
  exit 3
}

wait_for_nav2() {
  local attempt
  for ((attempt = 1; attempt <= startup_timeout; attempt++)); do
    local active_count
    active_count="$(compose_exec 'source /opt/ros/humble/setup.bash; {
      ros2 lifecycle get /controller_server 2>/dev/null;
      ros2 lifecycle get /planner_server 2>/dev/null;
    }' 2>/dev/null | grep -c '^active \[3\]$' || true)"
    if [[ "$active_count" -eq 2 ]]; then
      return 0
    fi
    sleep 1
  done
  printf 'Nav2 controller/planner did not become active within %ss.\n' "$startup_timeout" >&2
  return 1
}

artifact_dir="results/${label}"
mkdir -p "$artifact_dir"
cp src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml \
  "${artifact_dir}/导航参数.yaml"
cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml \
  "${artifact_dir}/碰撞监视参数.yaml"
cp "$world_file" "${artifact_dir}/世界文件.sdf"

contact_label="${label//\//_}"
contact_log="$(mktemp "/tmp/rtabmap-multi-${contact_label}.XXXXXX")"
contact_command="timeout ${contact_timeout}s gz topic -e ${contacts_topic} -u"
compose_exec "$contact_command" >"$contact_log" 2>/dev/null &
contact_pid=$!

sleep 2
wait_for_nav2
set +e
compose_exec "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav multi_goal_trial.py --goal-a-x ${goal_a_x} --goal-a-y ${goal_a_y} --goal-a-yaw ${goal_a_yaw} --goal-b-x ${goal_b_x} --goal-b-y ${goal_b_y} --goal-b-yaw ${goal_b_yaw} --settle-seconds ${settle_seconds} --label '${label}' --profile ${profile} --output-dir /workspaces/rtabmap_tb3_nav/results --world-file /workspaces/rtabmap_tb3_nav/${world_file}"
trial_exit=$?
set -e

stop_contact_listener

contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" |
  grep waffle | grep -v ground_plane | sort -u || true)"
contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"
contact_pairs_one_line="$(printf '%s' "$contact_pairs" | tr '\n' ';' | sed 's/;$//')"

if [[ -f "${artifact_dir}/metrics.yaml" ]]; then
  {
    printf 'git_commit: %s\n' "$(git rev-parse HEAD 2>/dev/null || printf unknown)"
    printf 'gazebo_contact_messages: %s\n' "$contact_count"
    if [[ -n "$contact_pairs" ]]; then
      printf 'gazebo_non_ground_contact: true\n'
      printf 'gazebo_contact_pairs: "%s"\n' "$contact_pairs_one_line"
    else
      printf 'gazebo_non_ground_contact: false\n'
      printf 'gazebo_contact_pairs: "(none)"\n'
    fi
  } >>"${artifact_dir}/metrics.yaml"
fi

printf 'label=%s\n' "$label"
printf 'trial_exit=%s\n' "$trial_exit"
printf 'profile=%s\n' "$profile"
printf 'contact_messages=%s\n' "$contact_count"
printf 'non_ground_contact_pairs:\n'
if [[ -n "$contact_pairs" ]]; then
  printf '%s\n' "$contact_pairs"
else
  printf '(none)\n'
fi

if [[ "$trial_exit" -ne 0 || -n "$contact_pairs" ]]; then
  exit 5
fi
