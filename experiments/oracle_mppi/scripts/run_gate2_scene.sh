#!/usr/bin/env bash
set -Eeuo pipefail

# Run one dynamic-scene experiment.  With no Oracle arguments this preserves
# the original Gate 2 Reactive-only behavior.  Gate 5 supplies an Oracle
# scenario and publisher config, in which case the same world/controller is
# used and the real future-occupancy publisher is started as an additional
# process.  The runner never overwrites a non-empty evidence directory.

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
repo_root="$PWD"

scenario='experiments/oracle_mppi/configs/scenarios/s1_crossing.yaml'
difficulty='medium'
label=''
profile='reactive_mppi_static'
nav2_params='experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml'
world_file=''
obstacle_model='experiments/oracle_mppi/worlds/oracle_dynamic_obstacle.sdf'
settle_seconds='5.0'
startup_timeout='90'
dynamic_startup_timeout='12'
contact_timeout='420'
expected_control_period='0.1'
oracle_scenario=''
oracle_publisher_config=''

usage() {
  cat <<'EOF'
Usage:
  experiments/oracle_mppi/scripts/run_gate2_scene.sh \
    --scenario experiments/oracle_mppi/configs/scenarios/s1_crossing.yaml \
    --difficulty medium \
    --label experiments/oracle_mppi/gate2/S1_crossing/run_01 \
    [--profile NAME] [--nav2-params PATH] [--world-file PATH] \
    [--obstacle-model PATH] [--expected-control-period SEC] \
    [--settle-seconds SEC] [--startup-timeout SEC] \
    [--dynamic-startup-timeout SEC] [--contact-timeout SEC] \
    [--oracle-scenario PATH] [--oracle-publisher-config PATH]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) scenario="$2"; shift 2 ;;
    --difficulty) difficulty="$2"; shift 2 ;;
    --label) label="$2"; shift 2 ;;
    --profile) profile="$2"; shift 2 ;;
    --nav2-params) nav2_params="$2"; shift 2 ;;
    --world-file) world_file="$2"; shift 2 ;;
    --obstacle-model) obstacle_model="$2"; shift 2 ;;
    --expected-control-period) expected_control_period="$2"; shift 2 ;;
    --settle-seconds) settle_seconds="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --dynamic-startup-timeout) dynamic_startup_timeout="$2"; shift 2 ;;
    --contact-timeout) contact_timeout="$2"; shift 2 ;;
    --oracle-scenario) oracle_scenario="$2"; shift 2 ;;
    --oracle-publisher-config) oracle_publisher_config="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$label" ]]; then
  usage >&2
  exit 2
fi
if [[ ("$label" != experiments/oracle_mppi/gate2/* &&
       "$label" != experiments/oracle_mppi/gate5/*) || "$label" == *..* ]]; then
  printf 'Unsafe dynamic-scene label: %s\n' "$label" >&2
  exit 2
fi
if [[ -n "$oracle_scenario" && -z "$oracle_publisher_config" ]] ||
   [[ -z "$oracle_scenario" && -n "$oracle_publisher_config" ]]; then
  printf '%s\n' '--oracle-scenario and --oracle-publisher-config must be supplied together' >&2
  exit 2
fi

# Gazebo is reset by every run, so two dynamic-scene runners cannot safely
# share the compose project at the same time.  Serialize all such runs before
# the first compose_down/compose_up call.  This also protects the evidence
# labels from accidental duplicate invocations by a batch driver.
runner_lock_path="/tmp/rtabmap_tb3_dynamic_scene_runner.lock"
exec 9>"$runner_lock_path"
if ! flock -n 9; then
  printf 'Another dynamic-scene runner is active; refusing concurrent run\n' >&2
  exit 3
fi

resolve_repo_file() {
  local value="$1" kind="$2" normalized host
  if [[ "$value" == /* ]]; then
    case "$value" in
      "$repo_root"/*) host="$value"; normalized="${value#"$repo_root/"}" ;;
      *) printf '%s must be inside repository: %s\n' "$kind" "$value" >&2; exit 2 ;;
    esac
  else
    normalized="${value#./}"
    host="$repo_root/$normalized"
  fi
  [[ -f "$host" ]] || { printf '%s not found: %s\n' "$kind" "$host" >&2; exit 2; }
  printf '%s\n%s\n' "$normalized" "$host"
}

mapfile -t scenario_file < <(resolve_repo_file "$scenario" 'Scenario')
scenario="${scenario_file[0]}"
scenario_host="${scenario_file[1]}"
mapfile -t nav2_file < <(resolve_repo_file "$nav2_params" 'Nav2 params')
nav2_params="${nav2_file[0]}"
nav2_params_host="${nav2_file[1]}"
mapfile -t model_file < <(resolve_repo_file "$obstacle_model" 'Obstacle model')
obstacle_model="${model_file[0]}"
obstacle_model_host="${model_file[1]}"

oracle_enabled=false
if [[ -n "$oracle_scenario" ]]; then
  mapfile -t oracle_scenario_file < <(resolve_repo_file "$oracle_scenario" 'Oracle scenario')
  oracle_scenario="${oracle_scenario_file[0]}"
  oracle_scenario_host="${oracle_scenario_file[1]}"
  mapfile -t oracle_config_file < <(resolve_repo_file "$oracle_publisher_config" 'Oracle publisher config')
  oracle_publisher_config="${oracle_config_file[0]}"
  oracle_publisher_config_host="${oracle_config_file[1]}"
  oracle_enabled=true
fi

if [[ -z "$world_file" ]]; then
  world_file="$(python3 - "$scenario_host" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    data = yaml.safe_load(stream)
print(data['world_file'])
PY
)"
fi
mapfile -t world_file_resolved < <(resolve_repo_file "$world_file" 'World')
world_file="${world_file_resolved[0]}"
world_host="${world_file_resolved[1]}"

mapfile -t scene_values < <(python3 - "$scenario_host" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    data = yaml.safe_load(stream)
robot = data['robot']
spawn = data['spawn']
print(data['scenario_id'])
print(data['obstacle_name'])
print(float(spawn['x']))
print(float(spawn['y']))
print(float(spawn.get('yaw', 0.0)))
print(float(robot['start_x']))
print(float(robot['start_y']))
print(float(robot['goal_x']))
print(float(robot['goal_y']))
print(float(robot.get('goal_yaw', 0.0)))
print(float(data.get('update_period_s', 0.05)))
PY
)
scenario_id="${scene_values[0]}"
obstacle_name="${scene_values[1]}"
spawn_x="${scene_values[2]}"
spawn_y="${scene_values[3]}"
spawn_yaw="${scene_values[4]}"
start_x="${scene_values[5]}"
start_y="${scene_values[6]}"
goal_x="${scene_values[7]}"
goal_y="${scene_values[8]}"
goal_yaw="${scene_values[9]}"
scenario_update_period="${scene_values[10]}"

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
scenario_container="/workspaces/rtabmap_tb3_nav/$scenario"
nav2_params_container="/workspaces/rtabmap_tb3_nav/$nav2_params"
model_container="/workspaces/rtabmap_tb3_nav/$obstacle_model"
commit="$(git rev-parse HEAD)"
nav2_params_snapshot="$(basename "$nav2_params")"
oracle_scenario_container="/workspaces/rtabmap_tb3_nav/$oracle_scenario"
oracle_publisher_config_container="/workspaces/rtabmap_tb3_nav/$oracle_publisher_config"
prediction_cost_weight="$(python3 - "$nav2_params_host" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    data = yaml.safe_load(stream) or {}
value = (
    data.get('controller_server', {})
        .get('ros__parameters', {})
        .get('FollowPath', {})
        .get('PredictionCritic', {})
        .get('cost_weight')
)
print('' if value is None else value)
PY
)"

cp "$scenario_host" "$artifact_dir/scenario.yaml"
cp "$nav2_params_host" "$artifact_dir/$nav2_params_snapshot"
cp src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml \
  "$artifact_dir/collision_monitor_rgbd_params.yaml"
cp src/rtabmap_tb3_nav/launch/demo.launch.py "$artifact_dir/demo.launch.py"
cp src/rtabmap_tb3_nav/src/goal_line_smac_planner.cpp \
  "$artifact_dir/goal_line_smac_planner.cpp"
cp "$world_host" "$artifact_dir/world.sdf"
cp "$obstacle_model_host" "$artifact_dir/oracle_dynamic_obstacle.sdf"
if [[ "$oracle_enabled" == true ]]; then
  cp "$oracle_scenario_host" "$artifact_dir/oracle_scenario.yaml"
  cp "$oracle_publisher_config_host" "$artifact_dir/oracle_publisher_gate3.yaml"
fi

printf '%s\n' \
  "scenario_id=$scenario_id" \
  "scenario=$scenario" \
  "difficulty=$difficulty" \
  "world_file=$world_file" \
  "world_name=$world_name" \
  "obstacle_name=$obstacle_name" \
  "start_x=$start_x" "start_y=$start_y" \
  "goal_x=$goal_x" "goal_y=$goal_y" "goal_yaw=$goal_yaw" \
  "profile=$profile" "nav2_params=$nav2_params" \
  "prediction_cost_weight=$prediction_cost_weight" \
  "expected_control_period=$expected_control_period" \
  "oracle_enabled=$oracle_enabled" \
  "scenario_update_period=$scenario_update_period" \
  "online=true" "localization=false" "reset_db=true" \
  "use_sim_time=true" "rviz=false" "gazebo_gui=false" "rtabmap_viz=false" \
  >"$artifact_dir/launch_arguments.txt"

if [[ "$oracle_enabled" == true ]]; then
  cat >"$artifact_dir/reproduce_command.sh" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
cd "$repo_root"
./experiments/oracle_mppi/scripts/run_gate2_scene.sh \\
  --scenario "$scenario" --difficulty "$difficulty" \\
  --profile "$profile" --nav2-params "$nav2_params" \\
  --expected-control-period "$expected_control_period" \\
  --settle-seconds "$settle_seconds" --startup-timeout "$startup_timeout" \\
  --contact-timeout "$contact_timeout" \\
  --label "$label" \\
  --oracle-scenario "$oracle_scenario" \\
  --oracle-publisher-config "$oracle_publisher_config"
EOF
else
  cat >"$artifact_dir/reproduce_command.sh" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
cd "$repo_root"
./experiments/oracle_mppi/scripts/run_gate2_scene.sh \\
  --scenario "$scenario" --difficulty "$difficulty" \\
  --profile "$profile" --nav2-params "$nav2_params" \\
  --expected-control-period "$expected_control_period" \\
  --settle-seconds "$settle_seconds" --startup-timeout "$startup_timeout" \\
  --contact-timeout "$contact_timeout" \\
  --label "$label"
EOF
fi
chmod +x "$artifact_dir/reproduce_command.sh"

{
  printf 'label: %s\n' "$label"
  printf 'scenario_id: %s\n' "$scenario_id"
  printf 'scenario: %s\n' "$scenario"
  printf 'difficulty: %s\n' "$difficulty"
  printf 'git_commit: %s\n' "$commit"
  printf 'world_file: %s\nworld_name: %s\n' "$world_file" "$world_name"
  printf 'contacts_topic: %s\n' "$contacts_topic"
  printf 'obstacle_name: %s\n' "$obstacle_name"
  printf 'start_x_m: %s\nstart_y_m: %s\n' "$start_x" "$start_y"
  printf 'goal_x_m: %s\ngoal_y_m: %s\ngoal_yaw_rad: %s\n' "$goal_x" "$goal_y" "$goal_yaw"
  printf 'profile: %s\nnav2_params: %s\n' "$profile" "$nav2_params"
  printf 'prediction_cost_weight: %s\n' "$prediction_cost_weight"
  printf 'online: true\nlocalization: false\nreset_db: true\nuse_sim_time: true\n'
  printf 'seed: deterministic waypoint schedule; no randomization\n'
  printf 'difficulty_time_scale: %s\nstart_delay_s: %s\n' 'from scenario profile' 'from scenario profile'
  printf 'scenario_update_period_s: %s\n' "$scenario_update_period"
  printf 'expected_control_period_s: %s\n' "$expected_control_period"
  printf 'oracle_enabled: %s\n' "$oracle_enabled"
  printf 'dynamic_startup_timeout_s: %s\n' "$dynamic_startup_timeout"
  printf 'evidence_time_basis: sim message timestamps; wall time only for process duration\n'
} >"$artifact_dir/experiment.yaml"
if [[ "$oracle_enabled" == true ]]; then
  printf 'oracle_scenario: %s\noracle_publisher_config: %s\n' \
    "$oracle_scenario" "$oracle_publisher_config" >>"$artifact_dir/experiment.yaml"
fi

launch_pid=''
dynamic_pid=''
oracle_pid=''
contact_pid=''
bag_pid=''
control_pid=''
tmp_label="oracle_gate2_${scenario_id}_$(basename "$label")_$$"
startup_recovery_attempted=false

stop_dynamic() {
  if [[ -n "$dynamic_pid" ]]; then
    compose_exec 'pkill -TERM -f "[d]ynamic_obstacle_controller.py" || true' >/dev/null 2>&1 || true
    kill -TERM "$dynamic_pid" 2>/dev/null || true
    wait "$dynamic_pid" 2>/dev/null || true
    dynamic_pid=''
  fi
}

stop_oracle() {
  if [[ -n "$oracle_pid" ]]; then
    compose_exec 'pkill -TERM -f "[o]racle_prediction_publisher" || true' \
      >/dev/null 2>&1 || true
    kill -TERM "$oracle_pid" 2>/dev/null || true
    wait "$oracle_pid" 2>/dev/null || true
    oracle_pid=''
  fi
}

stop_control() {
  if [[ -n "$control_pid" ]]; then
    compose_exec 'pkill -INT -f "[r]ecord_cmd_vel.py" || true' >/dev/null 2>&1 || true
    kill -TERM "$control_pid" 2>/dev/null || true
    wait "$control_pid" 2>/dev/null || true
    control_pid=''
  fi
}

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
  stop_dynamic
  stop_oracle
  stop_control
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

launch_command="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav demo.launch.py world_file:=$world_container x_pose:=$start_x y_pose:=$start_y online:=true localization:=false reset_db:=true use_sim_time:=true rviz:=false gazebo_gui:=false rtabmap_viz:=false navigation_profile:=$profile nav2_params:=$nav2_params_container"
printf 'Starting Gate 2 launch for %s (%s)\n' "$label" "$scenario_id"
(compose_exec "$launch_command" >"$artifact_dir/launch.log" 2>&1) &
launch_pid=$!

wait_for_nav2() {
  local states count collision_state
  for ((attempt = 1; attempt <= startup_timeout; attempt++)); do
    states="$(compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /collision_monitor; do timeout 4s ros2 lifecycle get "$n" 2>/dev/null || true; done' 2>/dev/null || true)"
    count="$(printf '%s\n' "$states" | grep -c '^active \[3\]$' || true)"
    [[ "$count" -eq 3 ]] && return 0

    # On a cold Gazebo/DDS start the independently launched collision monitor
    # can remain inactive after its lifecycle manager times out.  Retry that
    # one lifecycle transition once, while still requiring all three nodes to
    # be active before a goal is sent.  This is orchestration recovery only:
    # it does not bypass the safety node or change any navigation parameter.
    if [[ "$startup_recovery_attempted" == false ]] && \
       [[ "$count" -ge 2 ]] && \
       printf '%s\n' "$states" | grep -q '^inactive \[2\]$' && \
       (( attempt >= 15 )); then
      startup_recovery_attempted=true
      {
        printf 'attempt: %s\n' "$attempt"
        printf 'states_before_retry:\n%s\n' "$states"
        printf 'transition_log:\n'
      } >>"$artifact_dir/startup_recovery.txt"

      collision_state="$(compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 lifecycle get /collision_monitor 2>/dev/null || true' 2>/dev/null || true)"
      if printf '%s\n' "$collision_state" | grep -q '^unconfigured \[1\]$'; then
        compose_exec 'source /opt/ros/humble/setup.bash; timeout 20s ros2 lifecycle set /collision_monitor configure' \
          >>"$artifact_dir/startup_recovery.txt" 2>&1 || true
      fi
      compose_exec 'source /opt/ros/humble/setup.bash; timeout 20s ros2 lifecycle set /collision_monitor activate' \
        >>"$artifact_dir/startup_recovery.txt" 2>&1 || true
      printf 'states_after_retry:\n' >>"$artifact_dir/startup_recovery.txt"
      compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 lifecycle get /collision_monitor 2>&1 || true' \
        >>"$artifact_dir/startup_recovery.txt" 2>&1 || true
    fi

    kill -0 "$launch_pid" 2>/dev/null || return 1
    sleep 1
  done
  return 1
}

if ! wait_for_nav2; then
  printf 'Nav2 startup failed; evidence kept at %s\n' "$artifact_dir" >&2
  printf 'startup_failure: true\n' >>"$artifact_dir/experiment.yaml"
  printf 'startup_recovery_attempted: %s\n' "$startup_recovery_attempted" >>"$artifact_dir/experiment.yaml"
  compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /bt_navigator /collision_monitor; do echo "[$n]"; timeout 4s ros2 lifecycle get "$n" 2>&1 || true; done' \
    >"$artifact_dir/startup_readiness.txt" 2>&1 || true
  exit 4
fi

printf 'startup_recovery_attempted: %s\n' "$startup_recovery_attempted" >>"$artifact_dir/experiment.yaml"

compose_exec 'source /opt/ros/humble/setup.bash; ros2 topic list | sort; echo "--- lifecycle ---"; for n in /controller_server /planner_server /bt_navigator /collision_monitor; do echo "[$n]"; timeout 4s ros2 lifecycle get "$n" 2>&1 || true; done' \
  >"$artifact_dir/runtime_topics_and_lifecycle.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; for n in /controller_server /planner_server /global_costmap/global_costmap /local_costmap/local_costmap /velocity_smoother /collision_monitor; do echo "===== $n ====="; timeout 12s ros2 param dump "$n" 2>&1 || true; done' \
  >"$artifact_dir/runtime_parameters.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /clock --once' \
  >"$artifact_dir/clock_probe.txt" 2>&1 || true
compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /gazebo/model_states --once' \
  >"$artifact_dir/model_states_probe.txt" 2>&1 || true

spawn_command="source /opt/ros/humble/setup.bash; timeout 45s ros2 run gazebo_ros spawn_entity.py -entity ${obstacle_name} -file ${model_container} -x ${spawn_x} -y ${spawn_y} -z 0.0"
printf 'Spawning collidable dynamic obstacle: %s\n' "$obstacle_name"
spawn_recovery_attempted=false
spawn_attempts=1
set +e
compose_exec "$spawn_command" >"$artifact_dir/spawn_obstacle.log" 2>&1
spawn_exit=$?
set -e

# Gazebo Classic may expose /spawn_entity before the factory callback is
# ready.  A cold start can therefore time out the client even though the
# model was created, or fail before creation.  Check model_states first and
# retry once only when the model is absent.  This is launch orchestration
# recovery; the model remains a real collidable object and no navigation
# parameter or scenario trajectory is changed.
if [[ "$spawn_exit" -ne 0 ]]; then
  spawn_recovery_attempted=true
  {
    printf '\n--- spawn recovery ---\n'
    printf 'initial_spawn_exit_code: %s\n' "$spawn_exit"
    printf 'model_state_probe:\n'
  } >>"$artifact_dir/spawn_obstacle.log"
  model_state_probe="$(compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /gazebo/model_states --once 2>/dev/null || true' 2>/dev/null || true)"
  printf '%s\n' "$model_state_probe" >>"$artifact_dir/spawn_obstacle.log"
  if printf '%s\n' "$model_state_probe" | grep -qF -- "- $obstacle_name"; then
    spawn_exit=0
    printf 'model already present after client timeout; accepting spawn\n' \
      >>"$artifact_dir/spawn_obstacle.log"
  else
    sleep 2
    spawn_attempts=2
    printf 'retrying spawn_entity once\n' >>"$artifact_dir/spawn_obstacle.log"
    set +e
    compose_exec "$spawn_command" >>"$artifact_dir/spawn_obstacle.log" 2>&1
    spawn_exit=$?
    set -e
  fi
fi
printf 'spawn_attempts: %s\nspawn_recovery_attempted: %s\nspawn_exit_code: %s\n' \
  "$spawn_attempts" "$spawn_recovery_attempted" "$spawn_exit" \
  >>"$artifact_dir/experiment.yaml"
if [[ "$spawn_exit" -ne 0 ]]; then
  printf 'Dynamic obstacle spawn failed; evidence kept at %s\n' "$artifact_dir" >&2
  exit 4
fi

groundtruth_container="/workspaces/rtabmap_tb3_nav/${label}/dynamic_groundtruth.csv"
dynamic_summary_container="/workspaces/rtabmap_tb3_nav/${label}/dynamic_summary.yaml"
# For a paired Oracle run, choose one absolute Gazebo-clock reference before
# either schedule process starts.  Both the dynamic object driver and the
# publisher then evaluate the same YAML schedule at the same simulation time.
oracle_reference_sim_time=''
if [[ "$oracle_enabled" == true ]]; then
  clock_sample="$(compose_exec 'source /opt/ros/humble/setup.bash; timeout 8s ros2 topic echo /clock --once' 2>/dev/null || true)"
  oracle_clock_sec="$(printf '%s\n' "$clock_sample" | awk '$1 == "sec:" {print $2}' | head -1)"
  oracle_clock_nanosec="$(printf '%s\n' "$clock_sample" | awk '$1 == "nanosec:" {print $2}' | head -1)"
  if [[ -z "$oracle_clock_sec" || -z "$oracle_clock_nanosec" ]]; then
    printf 'Unable to determine shared Oracle simulation-time reference\n' >&2
    printf 'oracle_startup_failure: true\n' >>"$artifact_dir/experiment.yaml"
    exit 6
  fi
  oracle_reference_sim_time="$(awk -v sec="$oracle_clock_sec" -v nanosec="$oracle_clock_nanosec" \
    'BEGIN {printf "%.9f", sec + nanosec / 1000000000.0}')"
fi
# The controller declares use_sim_time with a true default.  Do not append
# ROS 2 CLI arguments here: this executable uses argparse and would reject
# them before rclpy can initialize.  Keeping the default in the node also
# makes this command independent of the shell's ROS argument quoting.
dynamic_command="source /opt/ros/humble/setup.bash; python3 /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/scripts/dynamic_obstacle_controller.py --scenario ${scenario_container} --difficulty ${difficulty} --output ${groundtruth_container} --summary ${dynamic_summary_container} --robot-name waffle"
if [[ "$oracle_enabled" == true ]]; then
  dynamic_command="${dynamic_command} --scenario-start-sim-time ${oracle_reference_sim_time}"
fi
(compose_exec "$dynamic_command" >"$artifact_dir/dynamic_controller.log" 2>&1) &
dynamic_pid=$!
dynamic_ready=false
for ((attempt = 1; attempt <= dynamic_startup_timeout; attempt++)); do
  if ! kill -0 "$dynamic_pid" 2>/dev/null; then
    break
  fi
  if [[ -f "$artifact_dir/dynamic_groundtruth.csv" ]] && \
     [[ "$(wc -l <"$artifact_dir/dynamic_groundtruth.csv")" -gt 1 ]]; then
    dynamic_ready=true
    break
  fi
  sleep 1
done
if [[ "$dynamic_ready" != true ]]; then
  printf 'Dynamic controller failed startup; evidence kept at %s\n' "$artifact_dir" >&2
  printf 'dynamic_controller_startup_failure: true\n' >>"$artifact_dir/experiment.yaml"
  if [[ -f "$artifact_dir/dynamic_groundtruth.csv" ]]; then
    printf 'dynamic_groundtruth_rows_at_startup_failure: %s\n' \
      "$(wc -l <"$artifact_dir/dynamic_groundtruth.csv")" \
      >>"$artifact_dir/experiment.yaml"
  fi
  exit 4
fi
printf 'dynamic_controller_startup_failure: false\n' >>"$artifact_dir/experiment.yaml"

oracle_validation_exit=0
if [[ "$oracle_enabled" == true ]]; then
  # The dynamic controller was started with this same absolute Gazebo-clock
  # reference, so publisher and physics object remain phase aligned.
  oracle_command="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run oracle_prediction_publisher oracle_prediction_publisher --ros-args --params-file $oracle_publisher_config_container -p scenario_file:=$oracle_scenario_container -p nav2_params_file:=$nav2_params_container -p scenario_start_sim_time:=$oracle_reference_sim_time"
  printf 'Starting Oracle publisher with shared reference t0=%s\n' \
    "$oracle_reference_sim_time"
  (compose_exec "$oracle_command" >"$artifact_dir/oracle_publisher.log" 2>&1) &
  oracle_pid=$!
  sleep 2
  if ! kill -0 "$oracle_pid" 2>/dev/null; then
    printf 'Oracle publisher exited during startup; evidence kept at %s\n' \
      "$artifact_dir" >&2
    printf 'oracle_startup_failure: true\n' >>"$artifact_dir/experiment.yaml"
    exit 6
  fi
  compose_exec 'source /opt/ros/humble/setup.bash; ros2 topic info /oracle/predicted_occupancy --verbose' \
    >"$artifact_dir/oracle_topic_info.txt" 2>&1 || true
  set +e
  compose_exec "source /opt/ros/humble/setup.bash; source /workspaces/rtabmap_tb3_nav/install/setup.bash; timeout 20s python3 /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/scripts/validate_gate3_ros_message.py --topic /oracle/predicted_occupancy --expected-frame odom --expected-source oracle --expected-resolution 0.05 --expected-width 120 --expected-height 100 --expected-dt 0.10 --expected-steps 31" \
    >"$artifact_dir/oracle_message_validation.txt" 2>&1
  oracle_validation_exit=$?
  set -e
  printf 'oracle_message_validation_exit: %s\n' "$oracle_validation_exit" \
    >>"$artifact_dir/experiment.yaml"
  if [[ "$oracle_validation_exit" -ne 0 ]]; then
    printf 'Oracle message validation failed; evidence kept at %s\n' \
      "$artifact_dir" >&2
    exit 6
  fi
  printf 'oracle_reference_sim_time_s: %s\n' "$oracle_reference_sim_time" \
    >>"$artifact_dir/experiment.yaml"
fi

contact_command="timeout ${contact_timeout}s gz topic -e ${contacts_topic} -u"
(compose_exec "$contact_command" >"$artifact_dir/gazebo_contacts.log" 2>&1) &
contact_pid=$!
sleep 2

bag_dir="/workspaces/rtabmap_tb3_nav/${label}/rosbag"
bag_command="source /opt/ros/humble/setup.bash; ros2 bag record --use-sim-time --compression-mode file --compression-format zstd -o ${bag_dir} /clock /tf /tf_static /odom /cmd_vel /cmd_vel_safe /map /nav_map /global_costmap/costmap /local_costmap/costmap /camera/obstacles /gazebo/model_states"
(compose_exec "$bag_command" >"$artifact_dir/rosbag_record.log" 2>&1) &
bag_pid=$!
sleep 2

control_csv="/workspaces/rtabmap_tb3_nav/${label}/cmd_vel.csv"
control_command="source /opt/ros/humble/setup.bash; python3 /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/scripts/record_cmd_vel.py --topic /cmd_vel --output ${control_csv}"
(compose_exec "$control_command" >"$artifact_dir/control_record.log" 2>&1) &
control_pid=$!
sleep 1

trial_command="source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav navigation_trial.py --x ${goal_x} --y ${goal_y} --yaw ${goal_yaw} --settle-seconds ${settle_seconds} --label ${tmp_label} --output-dir /workspaces/rtabmap_tb3_nav/results --world-file ${world_container}"
printf 'Running Reactive MPPI navigation and dynamic evidence capture...\n'
set +e
compose_exec "$trial_command" >"$artifact_dir/navigation.log" 2>&1
trial_exit=$?
set -e

stop_dynamic
stop_oracle
stop_control
stop_bag
stop_contact
stop_launch

if [[ -d "results/${tmp_label}" ]]; then
  if find "results/${tmp_label}" -mindepth 1 -print -quit | grep -q .; then
    mv "results/${tmp_label}"/* "$artifact_dir/"
    rmdir "results/${tmp_label}"
  fi
fi

contact_log="$artifact_dir/gazebo_contacts.log"
contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" | grep waffle | grep -v ground_plane | sort -u || true)"
# Require both model names in the same collision pair. A broad OR match
# would incorrectly classify oracle_dynamic_obstacle-vs-static-barrier
# contacts as robot-vs-dynamic-obstacle contacts.
dynamic_contact_pairs="$(grep -oE 'collision1: "[^"]+" collision2: "[^"]+"' "$contact_log" | awk -v obstacle="$obstacle_name" 'index($0, "waffle") && index($0, obstacle)' | sort -u || true)"
contact_count="$(grep -o 'contact {' "$contact_log" | wc -l | tr -d ' ')"
contact_pairs_one_line="$(printf '%s' "$contact_pairs" | tr '\n' ';' | sed 's/;$//')"
dynamic_pairs_one_line="$(printf '%s' "$dynamic_contact_pairs" | tr '\n' ';' | sed 's/;$//')"
dynamic_summary="${artifact_dir}/dynamic_summary.yaml"
dynamic_min_clearance='null'
dynamic_tracking_error='null'
dynamic_service_updates='null'
dynamic_service_failures='null'
dynamic_rows='0'
if [[ -f "$dynamic_summary" ]]; then
  dynamic_min_clearance="$(awk -F': ' '$1 == "minimum_robot_obstacle_clearance_m" {print $2}' "$dynamic_summary" | tail -1)"
  dynamic_tracking_error="$(awk -F': ' '$1 == "maximum_script_to_gazebo_position_error_m" {print $2}' "$dynamic_summary" | tail -1)"
  dynamic_service_updates="$(awk -F': ' '$1 == "service_updates" {print $2}' "$dynamic_summary" | tail -1)"
  dynamic_service_failures="$(awk -F': ' '$1 == "service_failures" {print $2}' "$dynamic_summary" | tail -1)"
  dynamic_rows="$(awk -F': ' '$1 == "groundtruth_rows_with_clearance" {print $2}' "$dynamic_summary" | tail -1)"
fi
{
  printf 'nav2_params: %s\n' "$nav2_params"
  printf 'cmd_vel_topic: /cmd_vel\ncmd_vel_csv: cmd_vel.csv\n'
  printf 'trial_exit_code: %s\nspawn_exit_code: %s\n' "$trial_exit" "$spawn_exit"
  printf 'oracle_message_validation_exit: %s\n' "$oracle_validation_exit"
  printf 'gazebo_contact_messages: %s\n' "$contact_count"
  printf 'dynamic_groundtruth_rows: %s\n' "$dynamic_rows"
  printf 'dynamic_service_updates: %s\ndynamic_service_failures: %s\n' \
    "$dynamic_service_updates" "$dynamic_service_failures"
  printf 'minimum_robot_obstacle_clearance_m: %s\n' "$dynamic_min_clearance"
  printf 'maximum_script_to_gazebo_position_error_m: %s\n' "$dynamic_tracking_error"
  if [[ -n "$contact_pairs" ]]; then
    printf 'gazebo_non_ground_contact: true\n'
    printf "gazebo_contact_pairs: '%s'\n" "$contact_pairs_one_line"
  else
    printf 'gazebo_non_ground_contact: false\n'
    printf 'gazebo_contact_pairs: "(none)"\n'
  fi
  if [[ -n "$dynamic_contact_pairs" ]]; then
    printf 'gazebo_robot_dynamic_contact: true\n'
    printf "gazebo_robot_dynamic_contact_pairs: '%s'\n" "$dynamic_pairs_one_line"
  else
    printf 'gazebo_robot_dynamic_contact: false\n'
    printf 'gazebo_robot_dynamic_contact_pairs: "(none)"\n'
  fi
} >>"$artifact_dir/experiment.yaml"

if [[ -f "$artifact_dir/metrics.yaml" ]]; then
  sed -i "1s|^label:.*|label: $label|" "$artifact_dir/metrics.yaml"
  {
    printf 'git_commit: %s\n' "$commit"
    printf 'scenario_id: %s\n' "$scenario_id"
    printf 'difficulty: %s\n' "$difficulty"
    printf 'scenario: %s\n' "$scenario"
    printf 'obstacle_name: %s\n' "$obstacle_name"
    printf 'prediction_cost_weight: %s\n' "$prediction_cost_weight"
    printf 'gazebo_contacts_topic: "%s"\n' "$contacts_topic"
    printf 'gazebo_contact_messages: %s\n' "$contact_count"
    printf 'wrapper_trial_exit: %s\n' "$trial_exit"
    printf 'spawn_exit_code: %s\n' "$spawn_exit"
    printf 'oracle_message_validation_exit: %s\n' "$oracle_validation_exit"
    printf 'dynamic_groundtruth_rows: %s\n' "$dynamic_rows"
    printf 'dynamic_service_failures: %s\n' "$dynamic_service_failures"
    printf 'minimum_robot_obstacle_clearance_m: %s\n' "$dynamic_min_clearance"
    if [[ -n "$contact_pairs" ]]; then
      printf 'gazebo_non_ground_contact: true\n'
      printf "gazebo_contact_pairs: '%s'\n" "$contact_pairs_one_line"
    else
      printf 'gazebo_non_ground_contact: false\n'
      printf 'gazebo_contact_pairs: "(none)"\n'
    fi
    if [[ -n "$dynamic_contact_pairs" ]]; then
      printf 'gazebo_robot_dynamic_contact: true\n'
      printf "gazebo_robot_dynamic_contact_pairs: '%s'\n" "$dynamic_pairs_one_line"
    else
      printf 'gazebo_robot_dynamic_contact: false\n'
      printf 'gazebo_robot_dynamic_contact_pairs: "(none)"\n'
    fi
    if [[ "$oracle_enabled" == true ]]; then
      printf 'oracle_active_log_lines: %s\n' \
        "$(grep -c 'PredictionCritic status=active' "$artifact_dir/launch.log" 2>/dev/null || true)"
      printf 'oracle_stale_log_lines: %s\n' \
        "$(grep -c 'PredictionCritic status=stale' "$artifact_dir/launch.log" 2>/dev/null || true)"
      printf 'oracle_publisher_ready_log_lines: %s\n' \
        "$(grep -c 'Gate 3 Oracle ready' "$artifact_dir/oracle_publisher.log" 2>/dev/null || true)"
      printf 'oracle_first_message_log_lines: %s\n' \
        "$(grep -c 'Published first Oracle grid' "$artifact_dir/oracle_publisher.log" 2>/dev/null || true)"
    fi
  } >>"$artifact_dir/metrics.yaml"
fi

gzip -f "$contact_log"

if [[ -d "$artifact_dir/rosbag" ]]; then
  compose_exec "source /opt/ros/humble/setup.bash; ros2 bag info /workspaces/rtabmap_tb3_nav/${label}/rosbag" \
    >"$artifact_dir/rosbag_info.txt" 2>&1 || true
fi

groundtruth_ok=false
if [[ -f "$artifact_dir/dynamic_groundtruth.csv" ]] && \
   [[ "$(wc -l <"$artifact_dir/dynamic_groundtruth.csv")" -gt 5 ]]; then
  groundtruth_ok=true
fi
printf 'scenario_id=%s\ntrial_exit=%s\nspawn_exit=%s\ndynamic_groundtruth=%s\ncontacts_topic=%s\ncontact_messages=%s\nartifact=%s\n' \
  "$scenario_id" "$trial_exit" "$spawn_exit" "$groundtruth_ok" "$contacts_topic" "$contact_count" "$artifact_dir"

if [[ "$trial_exit" -ne 0 || "$spawn_exit" -ne 0 || \
      "$oracle_validation_exit" -ne 0 || "$groundtruth_ok" != true || \
      -n "$contact_pairs" || "$dynamic_service_failures" != '0' ]]; then
  exit 5
fi
