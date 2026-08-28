# RTAB-Map TurtleBot3 RGB-D + Nav2

更新时间：2026-08-28
主机：Ubuntu 20.04 + RTX 4090
容器：Ubuntu 22.04 + ROS 2 Humble

本项目在 Docker 中运行 TurtleBot3 Waffle、Gazebo、RTAB-Map、Nav2 和模拟 RGB-D 相机，
实现无真实 LiDAR 的室内在线建图、点到点导航和障碍物绕行。

## 当前结论

目前已在两个静态跨场景仿真世界中完成：

- RGB-D 在线 RTAB-Map 建图；
- Nav2 全局/局部实时规划；
- GoalLineSmacPlanner 当前目标线软偏好；
- RPP 局部路径跟踪；
- collision_monitor 减速/停止保护；
- 多目标逐段导航、双视角轨迹记录和 Gazebo contacts 验收。

准确边界是：当前系统已在设计好的静态 Gazebo 场景中完成验证，不代表任意陌生环境、
动态障碍、任意窄通道或真实 D435i 已达到 100% 成功率。

## 文档和交接入口

- [项目交接资料（2026-08-24）](文档/09_项目交接_2026-08-24/README.md)：下一位 worker 的首要入口；
- [完整技术路线与阶段结果示范报告](文档/09_项目交接_2026-08-24/完整技术路线与阶段结果示范报告_2026-08-24.md)：从算法链路、阶段演进到轨迹证据和复现命令的详细技术报告；
- [阶段 2 长篇总览](文档/08_下一阶段实验归档_2026-08-22/00_阶段总览/阶段2跨场景验证总览与当前状态_2026-08-24.md)：场景 01/02 演进和边界；
- [当前项目进度](文档/00_项目总览/PROJECT_PROGRESS.md)；
- [全部实验索引](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)；
- [结果归档](results/README.md)；
- [文档目录](文档/README.md)。

### Oracle 预测式导航实验

已在独立分支 `exp/oracle-g3-publisher-2026-08-28` 继续执行任务书规定的逐 Gate
实验：Gate 0 静态 RPP 基线、Gate 1 Reactive MPPI 静态基线、Gate 2 动态场景与
碰撞真值链路均已完成，Gate 3 Oracle 未来时空占据接口也已通过硬验收。Gate 1 使用独立的
10 Hz `reactive_mppi_static` profile，在 `indoor_obstacle_course_large.world`
中执行 A→B、B→A 各 3 次；当前正式结果为 6/6 成功、无非地面 Gazebo contacts，
控制周期 P95 约为 0.1003 s。该 MPPI 基线仍有 3/6 次触发 Nav2 progress recovery，
效率明显低于 RPP，不能把它描述为性能提升。Gate 2 证明四类动态场景、真实碰撞、
Gazebo contacts、连续 clearance 和证据采集可复现，但 Reactive MPPI 在 S2/S4
仍有动态碰撞。Gate 3 仅证明 Oracle 未来信息消息接口的 frame、时间、栅格几何和
独立启停正确，尚未接入 PredictionCritic。完整协议、证据索引、参数和下一步
Gate 4 入口见 [Oracle 实验目录](experiments/oracle_mppi/README.md)、
[Gate 0 报告](experiments/oracle_mppi/reports/GATE0_REPORT_2026-08-27.md)与
[Gate 1 报告](experiments/oracle_mppi/reports/GATE1_REPORT_2026-08-28.md)、
[Gate 2 报告](experiments/oracle_mppi/reports/GATE2_REPORT_2026-08-28.md)和
[Gate 3 报告](experiments/oracle_mppi/reports/GATE3_REPORT_2026-08-28.md)。

## 当前 profile

### 通用跨场景基线

`adaptive_goal_line_045` 是当前通用基线：全局规划器为
`rtabmap_tb3_nav/GoalLineSmacPlanner`，底层为 SmacPlanner2D。每次规划调用根据当前
起点、当前目标和实时 costmap 计算目标线软偏好；关闭 large 场景固定世界走廊和目标
调度。全局/局部 inflation 为 `0.45 m`，目标速度约 `0.28 m/s`。

### 场景 02 最终 profile

`adaptive_goal_line_050_recovery_v13_line_tiebreaker` 只用于
`indoor_obstacle_course_cross_scene_02.world` 的最终验收：

- inflation `0.50 m`；
- `line_bias_max_cost=18.0`、`line_bias_distance_scale=2.8`、指数 `2.0`；
- `cost_travel_multiplier=6.0`；
- 关闭 `goal_progress_bias`、`unknown_bias` 和 `side_bias`；
- 约 6 秒周期重规划；
- RPP 速度 `0.28 m/s`，常规 `allow_reversing=false`；
- 保留 collision monitor 和恢复行为。

v13 不是固定坐标路线，不包含 `(6,-6)` 等中间 waypoint。

## 场景 01 与场景 02结果

| 场景 | 任务 | profile | 结果 |
| --- | --- | --- | --- |
| cross_scene_01 | 三组五点闭环，每组 3 次 | `adaptive_goal_line_045` | 9/9 run，45/45 段，0/9 非地面 contacts |
| cross_scene_02 | M→N × 3 | `adaptive_goal_line_045` | 3/3，0/3 非地面 contacts |
| cross_scene_02 | M→N→X→Y→M × 3 | v5 | 3/3，12/12 段，0/3 contacts |
| cross_scene_02 | M→N→X→Y→M × 3 | v13 | 3/3，12/12 段，0/3 contacts，平均 509.826 s |

正式场景 02结果位于
`results/05_跨场景验证/场景02/01_正式验收/`；历史失败排查位于
`results/05_跨场景验证/场景02/02_历史参考/`。

## 最短启动流程

### 1. 构建

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
```

### 2. 启动默认场景的 Gazebo + RViz2

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/launch_demo.sh \
  gazebo_gui:=true rviz:=true rtabmap_viz:=false \
  online:=true localization:=false reset_db:=true \
  navigation_profile:=adaptive_goal_line_045'
```

### 3. 发送单段目标

另开终端：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

## 复现场景 02 v13

终端 1：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/launch_demo.sh \
  gazebo_gui:=true rviz:=true rtabmap_viz:=false \
  online:=true localization:=false reset_db:=true \
  world_file:=/workspaces/rtabmap_tb3_nav/src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_02.world \
  navigation_profile:=adaptive_goal_line_050_recovery_v13_line_tiebreaker \
  x_pose:=-8.5 y_pose:=0.0'
```

终端 2 使用新的 label：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/multi_waypoint_regression.sh \
  --start-name M --start-x -8.5 --start-y 0.0 \
  --goal N:8.5:0.0:0.0 --goal X:-3.0:-4.0:0.0 \
  --goal Y:5.0:5.0:0.0 --goal M:-8.5:0.0:0.0 \
  --profile adaptive_goal_line_050_recovery_v13_line_tiebreaker \
  --world-file src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_02.world \
  --label 05_跨场景验证/场景02/03_新实验/复现_v13_$(date +%Y%m%d_%H%M%S) \
  --contact-timeout 1200 --settle-seconds 5'
```

确认：

```bash
rg -n 'overall_succeeded|nav2_status|gazebo_non_ground_contact|gazebo_contact_pairs' \
  results/05_跨场景验证/场景02/03_新实验/复现_v13_*/metrics.yaml
find results/05_跨场景验证/场景02 -name trajectory_comparison.png -print | sort
```

新 label 会创建新目录，不会覆盖已有轨迹图。

## 技术链路

```text
RGB-D -> RTAB-Map online SLAM -> /map、map->odom
      -> /nav_map + global/local costmap
      -> GoalLineSmacPlanner -> RPP -> collision_monitor
      -> TurtleBot3 Waffle
```

目标线由当前 start/goal 动态计算，黑线只是在轨迹图中提供参考。SDF 由 Gazebo 用于
物理场景，Nav2 不读取 SDF 作为固定路线。

## 结束仿真

在启动终端按 `Ctrl+C`，然后：

```bash
sg docker -c './scripts/stop.sh'
```

不要在仿真运行时同时启动真实 D435i launch；两者默认会使用相同 ROS 名称和 topic。
