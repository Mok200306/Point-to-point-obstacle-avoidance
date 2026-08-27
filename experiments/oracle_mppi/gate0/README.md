# Gate 0：RPP 静态基线冻结

## 目的

把当前可运行的 RGB-D 在线建图 + Nav2 + RPP 系统冻结成可复核的静态回归基线，为之后 MPPI 和 Oracle 对照提供参照。这里的“静态”指 world 中没有会主动运动的动态障碍；机器人仍然通过模拟 RGB-D 进行 RTAB-Map 在线建图和实时导航。

## 固定条件

| 项目 | Gate 0 值 |
|---|---|
| Git 分支 | `exp/oracle-mppi-2026-08-27` |
| world | `src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world` |
| launch | `src/rtabmap_tb3_nav/launch/demo.launch.py` |
| Nav2 参数 | `src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml` |
| collision monitor | `src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml` |
| profile | `adaptive_goal_line_045` |
| mapping | `online:=true`, `localization:=false`, `reset_db:=true` |
| clock | `use_sim_time:=true` |
| robot | TurtleBot3 Waffle，差速底盘，无 `/scan` |
| RGB-D | `/camera/image_raw`、`/camera/depth/image_raw`、`/camera/camera_info` |
| footprint | `[[0.30,0.24],[0.30,-0.24],[-0.30,-0.24],[-0.30,0.24]]`，padding 0.03 m |
| inflation | local/global radius 0.45 m，默认 cost scaling 3.0；profile 将 soft scaling 设为 4.5 |
| controller | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| planner | `rtabmap_tb3_nav/GoalLineSmacPlanner`；实时按本次 start/goal 计算目标线，关闭场景坐标 side bias |
| 目标 A→B | A=`(-8.5,0.0)`，B=`(8.5,0.0)`，yaw=0 |
| 目标 B→A | B=`(8.5,0.0)`，A=`(-8.5,0.0)`，yaw=π |
| 重复次数 | 每个方向 3 次 |

这里不使用历史 `fast_goalline_*` 固定世界坐标走廊 profile。这样 Gate 0 对后续目标和场景的解释边界更清楚。

## 启动 Gazebo、RViz 和在线建图

先启动容器并构建：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
```

每次要改变起点时，必须重新启动 launch，并把 `x_pose/y_pose` 与该用例起点一致：

```bash
# A → B
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav demo.launch.py world_file:=/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world x_pose:=-8.5 y_pose:=0.0 online:=true localization:=false reset_db:=true use_sim_time:=true rviz:=true gazebo_gui:=true navigation_profile:=adaptive_goal_line_045"'

# B → A（先停止上一个 launch，再执行）
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav demo.launch.py world_file:=/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world x_pose:=8.5 y_pose:=0.0 online:=true localization:=false reset_db:=true use_sim_time:=true rviz:=true gazebo_gui:=true navigation_profile:=adaptive_goal_line_045"'
```

`demo.launch.py` 默认同时启动 Gazebo GUI、RViz2、RTAB-Map 在线节点、`/nav_map` padder、Nav2、collision monitor 和目标线可视化。若只做无窗口 smoke test，可将 `rviz:=false gazebo_gui:=false`，但正式 Gate 证据仍需保存关键日志和轨迹。

## 单次运行

在另一个终端，从已经运行的 demo 中执行一条 leg。命令会保存 `metrics.yaml`、地图坐标轨迹、Gazebo ground-truth 轨迹、两栏 `trajectory_comparison.png`、参数和 world 快照，并从 Gazebo contacts 中过滤地面接触：

```bash
cd /home/w417/RTAB-Map
./experiments/oracle_mppi/scripts/run_gate0_leg.sh \
  --start-x -8.5 --start-y 0.0 \
  --x 8.5 --y 0.0 --yaw 0.0 \
  --profile adaptive_goal_line_045 \
  --label experiments/oracle_mppi/gate0/case_A_to_B/run_01
```

反向用例：

```bash
./experiments/oracle_mppi/scripts/run_gate0_leg.sh \
  --start-x 8.5 --start-y 0.0 \
  --x -8.5 --y 0.0 --yaw 3.141592653589793 \
  --profile adaptive_goal_line_045 \
  --label experiments/oracle_mppi/gate0/case_B_to_A/run_01
```

runner 会先 `docker compose down/up`，再以指定 `x_pose/y_pose` 启动新仿真，因而每一次 run 都重新开始世界与 RTAB-Map 数据库。`trajectory_trial` 的临时目录会在结束后自动搬入该 run；如果导航失败，失败目录和日志仍会保留。

批量执行 Gate 0：

```bash
./experiments/oracle_mppi/scripts/run_gate0_matrix.sh
```

矩阵脚本会按 A→B 的 run_01～03、B→A 的 run_01～03 依次执行，并生成 `baseline_rpp_static.csv`。不要在矩阵执行期间手动启动另一套 Gazebo 或 rosbag。

如果已经存在一组旧的正式目录，而本次修改了启动协议或其他会影响实验的代码，
必须用完整复测选项。它会把旧目录移动到 `runner_audit/`，再使用当前提交重新
生成相同的六个正式目录，不会覆盖旧证据：

```bash
./experiments/oracle_mppi/scripts/run_gate0_matrix.sh \
  --profile adaptive_goal_line_045 \
  --settle-seconds 5.0 \
  --startup-timeout 90 \
  --contact-timeout 420 \
  --rerun-all
```

本次基线曾发现冷启动时 `collision_monitor` 生命周期管理器与点云源创建存在竞态。
当前 launch 已将 monitor 创建后的 manager 延迟到 5 秒，并将 runner 的发送 Goal
前置条件扩大为 `/controller_server`、`/planner_server`、`/collision_monitor` 三者
全部处于 `active [3]`。因此修正后的正式矩阵不能混用修正前的运行目录。

## 必须保存的最小证据

每一个 run 至少包含：

- `metrics.yaml`：Nav2 status、成功标记、wall/sim time、轨迹长度、末端误差、地图/代价地图是否收到、contacts 摘要；
- `trajectory.csv`：Nav2/map-frame 采样轨迹；
- `gazebo_trajectory.csv`：Gazebo 真值轨迹；
- `trajectory.png` 与 `trajectory_comparison.png`；
- `nav2_rgbd_params.yaml`、`collision_monitor_rgbd_params.yaml`、`world.sdf`；
- `experiment.yaml`：commit、world、profile、Goal、seed、时间口径和 contacts 汇总；
- `reproduce_command.sh`：该 run 的不可覆盖复现命令；
- `launch.log`、`navigation.log`、`runtime_topics_and_lifecycle.txt`、`runtime_parameters.txt`；
- `rosbag/`、`rosbag_info.txt`、`rosbag_record.log`；
- `gazebo_contacts.log.gz`：过滤前的完整 Gazebo contacts 原始流压缩文件；
- 关键 launch 日志和轻量 rosbag，至少包含 `/clock`、`/tf`、`/tf_static`、`/odom`、`/cmd_vel`、`/cmd_vel_safe`、`/map`、`/nav_map`、两个 costmap、`/camera/obstacles` 和 `/gazebo/model_states`。

## 通过标准

Gate 0 只有在以下条件全部满足时才可标记 PASS：

1. A→B 3/3 成功，B→A 3/3 成功；
2. 每次 `nav2_status=4`，且 `succeeded=true`；
3. 每次 Gazebo 过滤地面后的 `gazebo_non_ground_contact=false`、`gazebo_contact_pairs=(none)`；
4. 每次有参数、轨迹、图和日志/rosbag，且使用仿真时钟；
5. 至少从一个新终端按本文命令完成一次端到端复现；
6. 汇总表和 Gate 报告明确记录失败、缺项或时间口径，不以单次成功代替 3/3 验收。

通过后才允许打 tag，例如：

```bash
git tag -a oracle-g0-pass -m 'Gate 0 static RPP baseline passed'
```
