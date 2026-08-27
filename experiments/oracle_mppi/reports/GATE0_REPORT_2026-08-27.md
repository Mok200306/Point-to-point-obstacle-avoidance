# Gate 0 正式报告：RPP 静态基线冻结

日期：2026-08-27
实验分支：`exp/oracle-mppi-2026-08-27`
正式提交：`84bff33699f770d7e8a8b785100ecf10e806f671`
任务书：`/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`

## 1. 结论

Gate 0 **PASS**。

在同一份修正后提交、同一静态世界和同一 `adaptive_goal_line_045` profile
下完成 A→B、B→A 各 3 次，共 6 次正式实验：

| 硬验收项 | 结果 |
|---|---|
| A→B | 3/3 成功 |
| B→A | 3/3 成功 |
| 总成功率 | 6/6 = 100% |
| Nav2 action status | 6 次均为 `4`（SUCCEEDED） |
| Gazebo 非地面物理接触 | 6 次均为 `false` |
| rosbag | 6 次均存在并包含核心话题 |
| RGB-D 在线建图 | 6 次均收到 `/map` 和 global costmap |
| `/use_sim_time` | 已启用 |

当前 RPP 系统可以作为后续 MPPI / Oracle 对照实验的静态 Reactive 基线。
Gate 0 不代表 MPPI、Oracle 未来信息或 Transformer 已完成。

## 2. 固定实验条件

```text
world: src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
profile: adaptive_goal_line_045
mapping: online=true, localization=false, reset_db=true
controller: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
planner: rtabmap_tb3_nav/GoalLineSmacPlanner + SmacPlanner2D
robot: TurtleBot3 Waffle，差速底盘，无真实 /scan
sensor: Gazebo 模拟 RGB-D
inflation: 0.45 m
goal A: (-8.5, 0.0), yaw=0
goal B: (8.5, 0.0), yaw=pi
```

链路为：

```text
模拟 RGB-D → RTAB-Map online SLAM → /map、map→odom
          → map_padder → /nav_map → global/local costmap
          → GoalLineSmacPlanner → RPP → /cmd_vel → velocity_smoother
          → collision_monitor → /cmd_vel_safe → TurtleBot3 Waffle
```

本阶段没有动态障碍、Oracle 未来占据、MPPI PredictionCritic 或训练模型。

## 3. 首轮失败与修正

修正前首轮矩阵为 5/6，唯一失败是 `A→B/run_03`：机器人轨迹约 `0.01 m`，
Nav2 返回 `status=6`，但 Gazebo 过滤地面后没有碰撞。关键日志为：

```text
collision_monitor.rclcpp: failed to send response to /collision_monitor/change_state
controller_server: Failed to make progress
behavior_server: backup failed
```

底盘只订阅 `/cmd_vel_safe`。collision monitor 未完成生命周期激活时，安全输出
没有有效发布，机器人无法移动；这不是规划器撞障碍或路线选择错误。

采用的最小修正如下：

1. `demo.launch.py`：monitor 进程创建后等待 5 秒，再启动独立 lifecycle manager；
   manager 又等待 5 秒后激活。
2. `run_gate0_leg.sh`：发送 Goal 前同时等待 `/controller_server`、
   `/planner_server`、`/collision_monitor` 为 `active [3]`。
3. `run_gate0_matrix.sh`：增加 `--rerun-all`，完整复测时先把旧目录移动到
   `runner_audit`，并严格检查结果目录的必需证据。

修正前的目录没有删除，均在
`experiments/oracle_mppi/gate0/runner_audit/` 中保留。修正后的 smoke 也成功，
并在 rosbag 中记录到 1917 条 `/cmd_vel_safe` 消息。

## 4. 六次正式结果

`wall_duration_s` 是宿主机墙钟时间，`simulation_duration_s` 是 Gazebo `/clock`
仿真时间；导航分析优先使用仿真时间。

| 用例 | 状态 | 墙钟时间 (s) | 仿真时间 (s) | Gazebo 轨迹 (m) | 末端 map 误差 (m) | 非地面 contacts |
|---|---:|---:|---:|---:|---:|---|
| A→B run_01 | 4 | 88.271 | 71.2 | 18.036 | 0.382 | false |
| A→B run_02 | 4 | 88.593 | 71.1 | 17.880 | 0.169 | false |
| A→B run_03 | 4 | 95.232 | 77.0 | 18.345 | 0.207 | false |
| B→A run_01 | 4 | 93.978 | 76.0 | 17.380 | 0.200 | false |
| B→A run_02 | 4 | 91.821 | 73.2 | 17.305 | 0.263 | false |
| B→A run_03 | 4 | 92.830 | 74.4 | 17.559 | 0.149 | false |

| 方向 | 墙钟平均 (s) | 仿真平均 (s) | Gazebo 轨迹平均 (m) | 末端误差平均 (m) |
|---|---:|---:|---:|---:|
| A→B | 90.698 | 73.100 | 18.087 | 0.253 |
| B→A | 92.877 | 74.533 | 17.415 | 0.204 |
| 全部 6 次 | 91.788 | 73.817 | 17.751 | 0.228 |

机器可读汇总：[baseline_rpp_static.csv](../gate0/baseline_rpp_static.csv)。

## 5. 误差和物理证据口径

`final_xy_error_m` 是取证节点在 action 完成后最后一次 map-frame 采样与命令目标
的距离，当前为 `0.149–0.382 m`。在线 RTAB-Map 的 `map→odom` 可能在末段更新，
且轨迹采样与 action 完成异步，因此它不直接等同于 Nav2 内部 goal-checker 误差。
任务书 Gate 0 没有额外的末端误差硬阈值，硬验收使用 Nav2 `status=4`、成功标志、
无非地面 contacts 和完整证据。后续若需要严格终点精度，应单独冻结新的指标口径。

`minimum_approx_clearance_m` 是近似离线几何值，不是物理碰撞判定；物理判定以
Gazebo contacts 过滤结果为准。

## 6. 轨迹图索引

每个 `trajectory_comparison.png` 均为左右双栏图：左侧是 Gazebo 世界俯视与真实
轨迹，右侧是 `/map`、global costmap 与 map-frame 轨迹。

| 用例 | 双栏图 |
|---|---|
| A→B run_01 | [图](../gate0/case_A_to_B/run_01/trajectory_comparison.png) |
| A→B run_02 | [图](../gate0/case_A_to_B/run_02/trajectory_comparison.png) |
| A→B run_03 | [图](../gate0/case_A_to_B/run_03/trajectory_comparison.png) |
| B→A run_01 | [图](../gate0/case_B_to_A/run_01/trajectory_comparison.png) |
| B→A run_02 | [图](../gate0/case_B_to_A/run_02/trajectory_comparison.png) |
| B→A run_03 | [图](../gate0/case_B_to_A/run_03/trajectory_comparison.png) |

正式 run 目录还包括参数、world、launch/navigation/runtime 日志、两种轨迹 CSV、
`rosbag_info.txt`、rosbag 和 `gazebo_contacts.log.gz`。原始大流和 verbose 日志
保留在本机；因部分文件超过 GitHub 单文件限制，版本库保存摘要、CSV、图、快照
和 `rosbag_info.txt`。

## 7. 复现命令

```bash
cd /home/w417/RTAB-Map
git switch exp/oracle-mppi-2026-08-27
./scripts/stop.sh
./scripts/start.sh
docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'
```

完整矩阵（会将旧 run 归档，不覆盖证据）：

```bash
./experiments/oracle_mppi/scripts/run_gate0_matrix.sh \
  --profile adaptive_goal_line_045 \
  --settle-seconds 5.0 --startup-timeout 90 --contact-timeout 420 \
  --rerun-all
```

单次复现使用 `run_gate0_leg.sh`，并给 `--label` 一个新的目录；矩阵执行期间不要
同时手动启动另一套 Gazebo。

## 8. 下一步

Gate 0 通过后，按任务书进入 Gate 1：新增独立 `nav2_mppi_oracle_params.yaml`，
先验证静态 Reactive MPPI；保留当前 RPP profile 不变，保存相同证据，且 MPPI
稳定前不实现动态 Oracle 或 Transformer。
