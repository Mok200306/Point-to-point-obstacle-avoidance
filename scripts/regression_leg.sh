#!/usr/bin/env bash
set -euo pipefail

# Run one navigation leg while recording Gazebo contacts. Start the simulation
# separately and place the robot at the matching endpoint before each leg.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

goal_x=''
goal_y='0.0'
goal_yaw='0.0'
label='navigation-leg'
contact_timeout='300'
startup_timeout='45'
contact_pid=''
contact_log=''

usage() {
  cat <<'EOF'
Usage:
  scripts/regression_leg.sh --x X [--y Y] [--yaw RAD] [--label NAME]

The current container must already be running with demo.launch.py. The script
returns non-zero when Nav2 does not finish with status 4 or when the leg has a
non-ground Gazebo contact involving waffle and a room obstacle.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --x) goal_x="$2"; shift 2 ;;
    --y) goal_y="$2"; shift 2 ;;
    --yaw) goal_yaw="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$goal_x" ]]; then
  usage >&2
  exit 2
fi

compose_exec() {
  local command_text="$1"
  local quoted
  printf -v quoted '%q' "$command_text"
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

  # `sg docker` can leave the inner docker exec alive after its local wrapper
  # receives SIGTERM. The `gz` executable is specific to this contact stream;
  # do not kill gzserver or gzclient.
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

contact_log="$(mktemp "/tmp/rtabmap-${label}.XXXXXX")"
contact_command="timeout ${contact_timeout}s gz topic -e /gazebo/indoor_obstacle_course_large/physics/contacts -u"
compose_exec "$contact_command" >"$contact_log" 2>/dev/null &
contact_pid=$!

sleep 2
wait_for_nav2
set +e
compose_exec "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x ${goal_x} --y ${goal_y} --yaw ${goal_yaw}"
goal_exit=$?
set -e

stop_contact_listener

contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" |
  grep waffle | grep -v ground_plane | sort -u || true)"
contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"

printf 'label=%s\n' "$label"
printf 'goal=(%s, %s, yaw=%s)\n' "$goal_x" "$goal_y" "$goal_yaw"
printf 'send_goal_exit=%s\n' "$goal_exit"
printf 'contact_messages=%s\n' "$contact_count"
printf 'non_ground_contact_pairs:\n'
if [[ -n "$contact_pairs" ]]; then
  printf '%s\n' "$contact_pairs"
else
  printf '(none)\n'
fi

if [[ "$goal_exit" -ne 0 || -n "$contact_pairs" ]]; then
  exit 5
fi
