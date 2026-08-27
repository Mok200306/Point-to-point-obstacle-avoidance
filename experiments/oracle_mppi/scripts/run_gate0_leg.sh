#!/usr/bin/env bash
set -Eeuo pipefail

# Isolated Gate 0 runner. It keeps the existing RPP/navigation implementation
# unchanged and adds evidence collection around one static navigation leg.

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
repo_root="$PWD"

start_x='-8.5'
start_y='0.0'
goal_x=''
goal_y='0.0'
goal_yaw='0.0'
label=''
profile='adaptive_goal_line_045'
world_file='src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world'
settle_seconds='5.0'
startup_timeout='75'
contact_timeout='420'

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate0_leg.sh \
    --start-x X --start-y Y --x X --y Y [--yaw RAD] \
    --label experiments/oracle_mppi/gate0/case_A_to_B/run_01 \
    [--profile NAME] [--settle-seconds SEC]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-x) start_x="$2"; shift 2 ;;
    --start-y) start_y="$2"; shift 2 ;;
    --x) goal_x="$2"; shift 2 ;;
    --y) goal_y="$2"; shift 2 ;;
    --yaw) goal_yaw="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --profile) profile="$2"; shift 2 ;;
    --world-file) world_file="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$goal_x" || -z "$label" ]]; then
  usage >&2
  exit 2
fi
if [[ "$label" != experiments/oracle_mppi/gate0/* || "$label" == *..* ]]; then
  printf 'Unsafe Gate 0 label: %s\n' "$label" >&2
  exit 2
fi

if [[ "$world_file" == /* ]]; then
  case "$world_file" in
    "$repo_root"/*)
      world_host="$world_file"
      world_file="${world_file#"$repo_root/"}"
      ;;
    *) printf 'World must be inside repository: %s\n' "$world_file" >&2; exit 2 ;;
  esac
else
  world_file="${world_file#./}"
  world_host="$repo_root/$world_file"
fi
[[ -f "$world_host" ]] || { printf 'World not found: %s\n' "$world_host" >&2; exit 2; }

artifact_dir="$repo_root/$label"
if [[ -e "$artifact_dir" ]] && find "$artifact_dir" -mindepth 1 -print -quit | grep -q .; then
  printf 'Refusing to overwrite non-empty evidence directory: %s\n' "$artifact_dir" >&2
  exit 2
fi
mkdir -p "$artifact_dir"

if docker info >/dev/null 2>&1; then
  direct_docker=true
else
  direct_docker=false
fi

compose_exec() {
  local command_text="$1"
  if [[ "$direct_docker" == true ]]; then
    docker compose exec -T ros2 bash -lc "$command_text"
    return
  fi
  local escaped quoted
  escaped="${command_text//\'/\'\\\'\'}"
  printf -v quoted "'%s'" "$escaped"
  sg docker -c "docker compose exec -T ros2 bash -lc $quoted"
}

compose_up() {
  if [[ "$direct_docker" == true ]]; then
    docker compose up -d >/dev/null
  else
    sg docker -c 'docker compose up -d >/dev/null'
  fi
}

compose_down() {
  if [[ "$direct_docker" == true ]]; then
    docker compose down >/dev/null 2>&1 || true
  else
    sg docker -c 'docker compose down >/dev/null 2>&1 || true'
  fi
}

compose_running() {
  if [[ "$direct_docker" == true ]]; then
    docker compose ps --status running --services | grep -qx ros2
  else
    sg docker -c 'docker compose ps --status running --services' | grep -qx ros2
  fi
}

world_name="$(python3 - "$world_host" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
world = root.find('world')
if world is None or not world.get('name'):
    raise SystemExit('world has no name')
print(world.get('name'))
PY
)"
contacts_topic="/gazebo/${world_name}/physics/contacts"
world_container="/workspaces/rtabmap_tb3_nav/$world_file"
commit="$(git rev-parse HEAD)"

cp src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml "$artifact_dir/nav2_rgbd_params.yaml"
cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml "$artifact_dir/collision_monitor_rgbd_params.yaml"
cp src/rtabmap_tb3_nav/launch/demo.launch.py "$artifact_dir/demo.launch.py"
cp src/rtabmap_tb3_nav/src/goal_line_smac_planner.cpp "$artifact_dir/goal_line_smac_planner.cpp"
cp "$world_host" "$artifact_dir/world.sdf"

printf '%s\n'   "world_file=$world_file"   "world_name=$world_name"   "start_x=$start_x"   "start_y=$start_y"   "goal_x=$goal_x"   "goal_y=$goal_y"   "goal_yaw=$goal_yaw"   "profile=$profile"   "online=true"   "localization=false"   "reset_db=true"   "use_sim_time=true"   "rviz=false"   "gazebo_gui=false"   "rtabmap_viz=false"   >"$artifact_dir/launch_arguments.txt"

cat >"$artifact_dir/reproduce_command.sh" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
cd "$repo_root"
./experiments/oracle_mppi/scripts/run_gate0_leg.sh \\
  --start-x "$start_x" --start-y "$start_y" \\
  --x "$goal_x" --y "$goal_y" --yaw "$goal_yaw" \\
  --profile "$profile" --settle-seconds "$settle_seconds" \\
  --startup-timeout "$startup_timeout" --contact-timeout "$contact_timeout" \\
  --label "$label"
EOF
chmod +x "$artifact_dir/reproduce_command.sh"

{
  printf 'label: %s\n' "$label"
  printf 'git_commit: %s\n' "$commit"
  printf 'world_file: %s\n' "$world_file"
  printf 'world_name: %s\n' "$world_name"
  printf 'contacts_topic: %s\n' "$contacts_topic"
  printf 'profile: %s\n' "$profile"
  printf 'start_x_m: %s\n' "$start_x"
  printf 'start_y_m: %s\n' "$start_y"
  printf 'goal_x_m: %s\n' "$goal_x"
  printf 'goal_y_m: %s\n' "$goal_y"
  printf 'goal_yaw_rad: %s\n' "$goal_yaw"
  printf 'online: true\nlocalization: false\nreset_db: true\nuse_sim_time: true\n'
  printf 'seed: not explicitly set; static world and fixed launch parameters\n'
  printf 'settle_seconds: %s\n' "$settle_seconds"
  printf 'evidence_time_basis: sim message timestamps; wall time only for process duration\n'
} >"$artifact_dir/experiment.yaml"

launch_pid=''
contact_pid=''
bag_pid=''
tmp_label="oracle_gate0_$(basename "$label")_$$"

stop_contact() {
  if [[ -n "$contact_pid" ]]; then
    kill -TERM "$contact_pid" 2>/dev/null || true
    wait "$contact_pid" 2>/dev/null || true
    contact_pid=''
  fi
  compose_exec 'pgrep -x gz | xargs -r kill -TERM' >/dev/null 2>&1 || true
}

stop_bag() {
  if [[ -n "$bag_pid" ]]; then
    compose_exec 'pkill -INT -f "[r]os2 bag record" || true' >/dev/null 2>&1 || true
    kill -TERM "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
    bag_pid=''
  fi
}

stop_launch() {
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for _ in {1..24}; do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    launch_pid=''
  fi
}

cleanup() {
  stop_bag
  stop_contact
  stop_launch
  compose_down
}
trap cleanup EXIT INT TERM

compose_down
compose_up
if ! compose_running; then
  printf 'ros2 container did not start\n' >&2
  exit 3
fi

launch_command="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav demo.launch.py world_file:=$world_container x_pose:=$start_x y_pose:=$start_y online:=true localization:=false reset_db:=true use_sim_time:=true rviz:=false gazebo_gui:=false rtabmap_viz:=false navigation_profile:=$profile"
printf 'Starting isolated launch for %s\n' "$label"
(compose_exec "$launch_command" >"$artifact_dir/launch.log" 2>&1) &
launch_pid=$!

wait_for_nav2() {
  local states count
  for ((attempt = 1; attempt <= startup_timeout; attempt++)); do
    states="$(compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /collision_monitor; do timeout 4s ros2 lifecycle get "$n" 2>/dev/null || true; done' 2>/dev/null || true)"
    count="$(printf '%s\n' "$states" | grep -c '^active \[3\]$' || true)"
    [[ "$count" -eq 3 ]] && return 0
    kill -0 "$launch_pid" 2>/dev/null || return 1
    sleep 1
  done
  return 1
}

if ! wait_for_nav2; then
  printf 'Nav2 startup failed; evidence kept at %s\n' "$artifact_dir" >&2
  printf 'startup_failure: true\n' >>"$artifact_dir/experiment.yaml"
  compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /collision_monitor; do echo "[$n]"; timeout 4s ros2 lifecycle get "$n" 2>&1 || true; done' >"$artifact_dir/startup_readiness.txt" 2>&1 || true
  exit 4
fi

compose_exec 'source /opt/ros/humble/setup.bash; ros2 topic list | sort; echo "--- lifecycle ---"; for n in /controller_server /planner_server /bt_navigator /collision_monitor; do echo "[$n]"; timeout 4s ros2 lifecycle get "$n" 2>&1 || true; done' >"$artifact_dir/runtime_topics_and_lifecycle.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /global_costmap/global_costmap /local_costmap/local_costmap /velocity_smoother /collision_monitor; do echo "===== $n ====="; timeout 12s ros2 param dump "$n" 2>&1 || true; done' >"$artifact_dir/runtime_parameters.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /clock --once' >"$artifact_dir/clock_probe.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /gazebo/model_states --once' >"$artifact_dir/model_states_probe.txt" 2>&1 || true

contact_command="timeout ${contact_timeout}s gz topic -e ${contacts_topic} -u"
(compose_exec "$contact_command" >"$artifact_dir/gazebo_contacts.log" 2>&1) &
contact_pid=$!
sleep 2

bag_dir="/workspaces/rtabmap_tb3_nav/${label}/rosbag"
bag_command="source /opt/ros/humble/setup.bash; ros2 bag record --use-sim-time --compression-mode file --compression-format zstd -o ${bag_dir} /clock /tf /tf_static /odom /cmd_vel /cmd_vel_safe /map /nav_map /global_costmap/costmap /local_costmap/costmap /camera/obstacles /gazebo/model_states"
(compose_exec "$bag_command" >"$artifact_dir/rosbag_record.log" 2>&1) &
bag_pid=$!
sleep 2

trial_command="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav navigation_trial.py --x ${goal_x} --y ${goal_y} --yaw ${goal_yaw} --settle-seconds ${settle_seconds} --label ${tmp_label} --output-dir /workspaces/rtabmap_tb3_nav/results --world-file ${world_container}"
printf 'Running navigation and evidence capture...\n'
set +e
compose_exec "$trial_command" >"$artifact_dir/navigation.log" 2>&1
trial_exit=$?
set -e

stop_bag
stop_contact
stop_launch

if [[ -d "results/${tmp_label}" ]]; then
  if find "results/${tmp_label}" -mindepth 1 -print -quit | grep -q .; then
    mv "results/${tmp_label}"/* "$artifact_dir/"
    rmdir "results/${tmp_label}"
  fi
fi

if [[ -f "$artifact_dir/metrics.yaml" ]]; then
  sed -i "1s|^label:.*|label: $label|" "$artifact_dir/metrics.yaml"
fi

contact_log="$artifact_dir/gazebo_contacts.log"
contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" | grep waffle | grep -v ground_plane | sort -u || true)"
contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"
contact_pairs_one_line="$(printf '%s' "$contact_pairs" | tr '\n' ';' | sed 's/;$//')"
{
  printf 'trial_exit_code: %s\n' "$trial_exit"
  printf 'gazebo_contact_messages: %s\n' "$contact_count"
  if [[ -n "$contact_pairs" ]]; then
    printf 'gazebo_non_ground_contact: true\n'
    printf 'gazebo_contact_pairs: "%s"\n' "$contact_pairs_one_line"
  else
    printf 'gazebo_non_ground_contact: false\n'
    printf 'gazebo_contact_pairs: "(none)"\n'
  fi
} >>"$artifact_dir/experiment.yaml"

# Preserve the full transport stream in a compact, recoverable form. The
# summary above is computed from the uncompressed stream before compression.
gzip -f "$contact_log"

if [[ -f "$artifact_dir/metrics.yaml" ]]; then
  {
    printf 'git_commit: %s\n' "$commit"
    printf 'profile: %s\n' "$profile"
    printf 'world_file: %s\n' "$world_file"
    printf 'gazebo_contacts_topic: "%s"\n' "$contacts_topic"
    printf 'gazebo_contact_messages: %s\n' "$contact_count"
    printf 'wrapper_trial_exit: %s\n' "$trial_exit"
    if [[ -n "$contact_pairs" ]]; then
      printf 'gazebo_non_ground_contact: true\n'
      printf 'gazebo_contact_pairs: "%s"\n' "$contact_pairs_one_line"
    else
      printf 'gazebo_non_ground_contact: false\n'
      printf 'gazebo_contact_pairs: "(none)"\n'
    fi
  } >>"$artifact_dir/metrics.yaml"
fi

if [[ -d "$artifact_dir/rosbag" ]]; then
  compose_exec "source /opt/ros/humble/setup.bash; ros2 bag info /workspaces/rtabmap_tb3_nav/${label}/rosbag" >"$artifact_dir/rosbag_info.txt" 2>&1 || true
fi

# Keep the complete Gazebo transport stream without leaving six very large
# uncompressed text files in the repository. The gzip file is the raw stream,
# and the summary fields above are computed before compression.
if [[ -f "$artifact_dir/gazebo_contacts.log" ]]; then
  gzip -f "$artifact_dir/gazebo_contacts.log"
fi

printf 'label=%s\ntrial_exit=%s\ncontacts_topic=%s\ncontact_messages=%s\nartifact=%s\n' "$label" "$trial_exit" "$contacts_topic" "$contact_count" "$artifact_dir"
if [[ "$trial_exit" -ne 0 || -n "$contact_pairs" ]]; then
  exit 5
fi
