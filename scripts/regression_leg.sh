#!/usr/bin/env bash
set -euo pipefail

# Run one navigation leg while recording Gazebo contacts. Start the simulation
# separately and place the robot at the matching endpoint before each leg.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

goal_x=''
goal_y='0.0'
goal_yaw='0.0'
label='navigation-leg'
profile='unspecified'
contact_timeout='300'
startup_timeout='45'
settle_seconds='0'
contact_pid=''
contact_log=''

usage() {
  cat <<'EOF'
Usage:
  scripts/regression_leg.sh --x X [--y Y] [--yaw RAD] [--label NAME]
    [--profile NAME] [--settle-seconds SECONDS]

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
    --profile) profile="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$goal_x" ]]; then
  usage >&2
  exit 2
fi

# Accept both foo/bar and results/foo/bar without creating results/results/...
label="${label#results/}"

compose_exec() {
  local command_text="$1"
  local quoted
  # `sg docker -c` is evaluated by /bin/sh on this host. Bash's `%q` emits
  # `$'...'`, which /bin/sh does not parse and which silently breaks the ROS
  # setup command. Use portable single-quote escaping instead.
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

snapshot_trial() {
  local artifact_dir="results/${label}"
  local commit
  local planner
  commit="$(git rev-parse HEAD 2>/dev/null || printf 'unknown')"
  planner="$(awk '
    /^    GridBased:/ { in_grid=1; next }
    in_grid && /^    [A-Za-z_][A-Za-z0-9_]*:/ { exit }
    in_grid && /plugin:/ {
      sub(/^[^:]*: */, "")
      print
      exit
    }
  ' src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)"
  planner="${planner:-unknown}"
  mkdir -p "${artifact_dir}"
  cp src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml \
    "${artifact_dir}/nav2_rgbd_params.yaml"
  cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml \
    "${artifact_dir}/collision_monitor_rgbd_params.yaml"
  cp src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world \
    "${artifact_dir}/world.sdf"
  {
    printf 'label: %s\n' "$label"
    printf 'profile: %s\n' "$profile"
    printf 'git_commit: %s\n' "$commit"
    printf 'world: indoor_obstacle_course_large\n'
    printf 'goal_frame: map\n'
    printf 'goal_x_m: %s\n' "$goal_x"
    printf 'goal_y_m: %s\n' "$goal_y"
    printf 'goal_yaw_rad: %s\n' "$goal_yaw"
    printf 'startup_settle_s: %s\n' "$settle_seconds"
    printf 'navigation_wall_time_excludes_startup_settle: true\n'
    printf 'planner: %s\n' "$planner"
    printf 'controller: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController\n'
    printf 'gazebo_view: SDF collision geometry plus /gazebo/model_states ground truth\n'
    printf 'rviz_view: /map plus /global_costmap/costmap and map-frame trajectory\n'
  } >"${artifact_dir}/experiment.yaml"
}

contact_label="${label//\//_}"
contact_log="$(mktemp "/tmp/rtabmap-${contact_label}.XXXXXX")"
contact_command="timeout ${contact_timeout}s gz topic -e /gazebo/indoor_obstacle_course_large/physics/contacts -u"
compose_exec "$contact_command" >"$contact_log" 2>/dev/null &
contact_pid=$!

sleep 2
wait_for_nav2
set +e
compose_exec "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav navigation_trial.py --x ${goal_x} --y ${goal_y} --yaw ${goal_yaw} --settle-seconds ${settle_seconds} --label ${label} --output-dir /workspaces/rtabmap_tb3_nav/results --world-file /workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world"
goal_exit=$?
set -e

stop_contact_listener

contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" |
  grep waffle | grep -v ground_plane | sort -u || true)"
contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"
contact_pairs_one_line="$(printf '%s' "$contact_pairs" | tr '\n' ';' | sed 's/;$//')"

snapshot_trial
if [[ -f "results/${label}/metrics.yaml" ]]; then
  {
    printf 'gazebo_contact_messages: %s\n' "$contact_count"
    if [[ -n "$contact_pairs" ]]; then
      printf 'gazebo_non_ground_contact: true\n'
      printf 'gazebo_contact_pairs: "%s"\n' "$contact_pairs_one_line"
    else
      printf 'gazebo_non_ground_contact: false\n'
      printf 'gazebo_contact_pairs: "(none)"\n'
    fi
  } >>"results/${label}/metrics.yaml"
fi

printf 'label=%s\n' "$label"
printf 'goal=(%s, %s, yaw=%s)\n' "$goal_x" "$goal_y" "$goal_yaw"
printf 'navigation_trial_exit=%s\n' "$goal_exit"
printf 'profile=%s\n' "$profile"
printf 'startup_settle_s=%s\n' "$settle_seconds"
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
