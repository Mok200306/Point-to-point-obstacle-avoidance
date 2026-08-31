#!/usr/bin/env bash
set -euo pipefail

# Run an arbitrary sequence of NavigateToPose goals in one live RGB-D session
# while recording contacts and the color-coded multi-waypoint evidence.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

goal_specs=()
start_name='M'
start_x='-8.5'
start_y='0.0'
label='五点闭环导航实验'
profile='adaptive_goal_line_045'
world_file='src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world'
contacts_topic=''
contact_timeout='1200'
startup_timeout='45'
settle_seconds='5'
dynamic_obstacle_model=''
contact_pid=''
contact_log=''
contacts_archived='false'
artifact_dir=''
contact_count='0'
contact_pairs=''
contact_pairs_one_line=''
contact_pairs_yaml=''

usage() {
  cat <<'EOF'
Usage:
  scripts/multi_waypoint_regression.sh \
    --goal NAME:X:Y:YAW [--goal NAME:X:Y:YAW ...] \
    [--start-name NAME] [--start-x X] [--start-y Y] [--label NAME]
    [--profile NAME] [--world-file PATH] [--contacts-topic TOPIC]
    [--contact-timeout SECONDS] [--startup-timeout SECONDS]
    [--settle-seconds SECONDS] [--dynamic-obstacle-model MODEL]

The current container must already be running with demo.launch.py. Goals are
sent sequentially without resetting Gazebo or the RTAB-Map database.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --goal) goal_specs+=("$2"); shift 2 ;;
    --start-name) start_name="$2"; shift 2 ;;
    --start-x) start_x="$2"; shift 2 ;;
    --start-y) start_y="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --profile) profile="$2"; shift 2 ;;
    --world-file) world_file="$2"; shift 2 ;;
    --contacts-topic) contacts_topic="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --dynamic-obstacle-model) dynamic_obstacle_model="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "${#goal_specs[@]}" -eq 0 ]]; then
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

if [[ ! -f "$world_file" ]]; then
  printf 'World file does not exist: %s\n' "$world_file" >&2
  exit 2
fi

if [[ -z "$contacts_topic" ]]; then
  world_name="$(python3 - "$world_file" <<'PY'
import sys
import xml.etree.ElementTree as element_tree

root = element_tree.parse(sys.argv[1]).getroot()
world = root.find('world')
if world is None or not world.get('name'):
    raise SystemExit('world SDF has no named <world> element')
print(world.get('name'))
PY
)"
  contacts_topic="/gazebo/${world_name}/physics/contacts"
fi

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
  archive_contacts || true
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
    # A lifecycle query can wait indefinitely when the service is not ready
    # (or when a stale ROS 2 CLI daemon still holds an old graph). Keep each
    # probe bounded so a regression run cannot hang before its first goal.
    active_count="$(compose_exec 'source /opt/ros/humble/setup.bash;
      for node_name in /controller_server /planner_server; do
        timeout 4s ros2 lifecycle get "${node_name}" 2>/dev/null || true;
      done' 2>/dev/null | grep -c '^active \[3\]$' || true)"
    if [[ "$active_count" -eq 2 ]]; then
      return 0
    fi
    sleep 1
  done
  printf 'Nav2 controller/planner did not become active within %ss.\n' \
    "$startup_timeout" >&2
  return 1
}

artifact_dir="results/${label}"
mkdir -p "$artifact_dir"
cp src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml \
  "${artifact_dir}/导航参数_源码.yaml"
cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml \
  "${artifact_dir}/碰撞监视参数_源码.yaml"
cp "$world_file" "${artifact_dir}/世界文件.sdf"

if [[ ! -f "${artifact_dir}/导航参数.yaml" ]]; then
  cp src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml \
    "${artifact_dir}/导航参数.yaml"
fi
if [[ ! -f "${artifact_dir}/碰撞监视参数.yaml" ]]; then
  cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml \
    "${artifact_dir}/碰撞监视参数.yaml"
fi

snapshot_runtime_params() {
  local node
  local file_name
  local dump
  while IFS='|' read -r node file_name; do
    if dump="$(compose_exec "source /opt/ros/humble/setup.bash && timeout 10s ros2 param dump ${node}" 2>/dev/null)"; then
      printf '%s\n' "$dump" >"${artifact_dir}/${file_name}"
    fi
  done <<'EOF'
/planner_server|运行时参数_planner_server.yaml
/controller_server|运行时参数_controller_server.yaml
/velocity_smoother|运行时参数_velocity_smoother.yaml
/global_costmap/global_costmap|运行时参数_global_costmap.yaml
/local_costmap/local_costmap|运行时参数_local_costmap.yaml
/collision_monitor|运行时参数_collision_monitor.yaml
EOF
}

archive_contacts() {
  if [[ "$contacts_archived" == 'true' || -z "$contact_log" || \
        ! -f "$contact_log" || -z "$artifact_dir" ]]; then
    return 0
  fi
  contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' \
    "$contact_log" | grep waffle | grep -v ground_plane | sort -u || true)"
  contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"
  contact_pairs_one_line="$(printf '%s' "$contact_pairs" | tr '\n' ';' | sed 's/;$//')"
  contact_pairs_yaml="$(printf '%s' "$contact_pairs_one_line" | sed "s/'/''/g")"
  gzip -c "$contact_log" >"${artifact_dir}/gazebo_contacts_raw.log.gz"
  {
    printf 'topic: "%s"\n' "$contacts_topic"
    printf 'messages: %s\n' "$contact_count"
    if [[ -n "$contact_pairs" ]]; then
      printf 'non_ground_contact: true\n'
      printf "pairs: '%s'\n" "$contact_pairs_yaml"
    else
      printf 'non_ground_contact: false\n'
      printf 'pairs: "(none)"\n'
    fi
  } >"${artifact_dir}/gazebo_contacts_summary.yaml"
  contacts_archived='true'
}

contact_label="${label//\//_}"
wait_for_nav2
snapshot_runtime_params

# Start the Gazebo contacts subscriber only after Nav2 has completed its
# lifecycle transition.  `gz topic -e` can be CPU-intensive on this host; if
# it is started during bringup it may delay lifecycle service responses and
# create a false "Nav2 not active" failure before the first goal is sent.
contact_log="$(mktemp "/tmp/rtabmap-waypoints-${contact_label}.XXXXXX")"
contact_command="timeout ${contact_timeout}s gz topic -e ${contacts_topic} -u"
compose_exec "$contact_command" >"$contact_log" 2>/dev/null &
contact_pid=$!
sleep 2

goal_arguments=''
for goal_spec in "${goal_specs[@]}"; do
  printf -v quoted_goal '%q' "$goal_spec"
  goal_arguments+=" --goal ${quoted_goal}"
done
printf -v quoted_start_name '%q' "$start_name"
printf -v quoted_label '%q' "$label"
printf -v quoted_profile '%q' "$profile"
printf -v quoted_world_file '%q' "/workspaces/rtabmap_tb3_nav/${world_file}"
printf -v quoted_dynamic_obstacle_model '%q' "$dynamic_obstacle_model"
dynamic_obstacle_argument=''
if [[ -n "$dynamic_obstacle_model" ]]; then
  dynamic_obstacle_argument=" --dynamic-obstacle-model ${quoted_dynamic_obstacle_model}"
fi

trial_command="source /opt/ros/humble/setup.bash && \
source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
ros2 run rtabmap_tb3_nav multi_waypoint_trial.py${goal_arguments} \
--start-name ${quoted_start_name} --start-x ${start_x} --start-y ${start_y} \
--settle-seconds ${settle_seconds} --label ${quoted_label} \
--profile ${quoted_profile} --output-dir /workspaces/rtabmap_tb3_nav/results \
--world-file ${quoted_world_file}${dynamic_obstacle_argument}"

set +e
compose_exec "$trial_command"
trial_exit=$?
set -e

stop_contact_listener
archive_contacts

if [[ -f "${artifact_dir}/metrics.yaml" ]]; then
  {
    printf 'git_commit: %s\n' "$(git rev-parse HEAD 2>/dev/null || printf unknown)"
    source_dirty='false'
    if ! git diff --quiet --ignore-submodules -- . ':(exclude)results'; then
      source_dirty='true'
    elif [[ -n "$(git ls-files --others --exclude-standard -- . ':(exclude)results')" ]]; then
      source_dirty='true'
    fi
    if [[ "$source_dirty" == 'false' ]]; then
      printf 'git_dirty: false\n'
    else
      printf 'git_dirty: true\n'
    fi
    printf 'runtime_parameter_snapshot: true\n'
    printf 'dynamic_obstacle_model: %s\n' "${dynamic_obstacle_model:-null}"
    if [[ -n "$dynamic_obstacle_model" ]]; then
      printf 'dynamic_obstacle_driver: dynamic_obstacle_driver.py\n'
    else
      printf 'dynamic_obstacle_driver: null\n'
    fi
    printf 'gazebo_contacts_topic: "%s"\n' "$contacts_topic"
    printf 'gazebo_contact_messages: %s\n' "$contact_count"
    printf 'gazebo_contacts_raw: gazebo_contacts_raw.log.gz\n'
    printf 'gazebo_contacts_summary: gazebo_contacts_summary.yaml\n'
    if [[ -n "$contact_pairs" ]]; then
      printf 'gazebo_non_ground_contact: true\n'
      printf "gazebo_contact_pairs: '%s'\n" "$contact_pairs_yaml"
    else
      printf 'gazebo_non_ground_contact: false\n'
      printf 'gazebo_contact_pairs: "(none)"\n'
    fi
  } >>"${artifact_dir}/metrics.yaml"
fi

printf 'label=%s\n' "$label"
printf 'trial_exit=%s\n' "$trial_exit"
printf 'profile=%s\n' "$profile"
printf 'world_file=%s\n' "$world_file"
printf 'contacts_topic=%s\n' "$contacts_topic"
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
