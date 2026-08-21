# 2026-08-21 固定世界坐标连续走廊版参数冻结

## 冻结对象

冻结 profile：`fast_north_045_v3`。它包含 TurtleBot3 Waffle 仿真、RGB-D 在线建图、
GoalLineSmacPlanner、RPP、velocity smoother 和 collision monitor 完整链路。

- 代码提交：`6202912`
- 场景：`indoor_obstacle_course_large.world`
- 起点：`map=(-8.5, 0.0, yaw=0)`
- 终点：`map=(8.5, 0.0, yaw=0)`
- 模式：`online=true localization=false reset_db=true`
- 验收标准：Nav2 status `4`，Gazebo 无非地面 contacts

## 三次正式结果

| 运行 | wall [s] | map path [m] | Gazebo path [m] | error [m] | min approx clearance [m] | contacts |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| run 01 | 90.238 | 17.220 | 17.568 | 0.261 | 0.009 | none |
| run 02 | 91.352 | 17.196 | 17.392 | 0.344 | 0.004 | none |
| run 03 | 91.659 | 17.106 | 17.517 | 0.321 | -0.035 | none |
| 平均 | **91.083** | **17.174** | **17.492** | **0.309** | -0.007 | **0/3** |

近似 clearance 只用于诊断；物理碰撞以过滤 `ground_plane` 后的 Gazebo contacts 为准。
三次正式结果的统计和参数仍保留在本文；原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md) 清理，可从提交 `6202912` 恢复。

## 规划与控制参数

| 类别 | 冻结值 |
| --- | --- |
| global planner | `rtabmap_tb3_nav/GoalLineSmacPlanner` |
| planner base | `nav2_smac_planner/SmacPlanner2D` |
| `allow_unknown` | `true` |
| Smac tolerance / max planning time | `0.25 m` / `1.0 s` |
| `cost_travel_multiplier` | `6.0` |
| line bias | enabled, max `60`, scale `2.0 m`, exponent `2.0` |
| Smac smoother | `max_iterations=1000`, `w_data=0.20`, `w_smooth=0.40`, refinement=true |
| RPP desired speed | `0.26 m/s` |
| RPP lookahead | `0.75 m`, dynamic `0.56--1.15 m`, time `1.5 s` |
| RPP curvature regulation | enabled, min radius `0.75 m`, min speed `0.08 m/s` |
| RPP cost regulation | enabled, distance `0.55 m`, gain `1.0`, inflation factor `3.0` |
| RPP collision check | enabled, carrot horizon `1.0 s` |
| controller frequency | `15 Hz` |
| progress checker | radius `0.20 m`, allowance `20 s` |
| goal checker | XY `0.12 m`, yaw `0.15 rad` |
| velocity smoother max | `[0.26, 0.0, 0.85]` |
| velocity smoother accel/decel | `[0.9, 0.0, 2.0]` / `[-1.1, 0.0, -2.0]` |

## 固定连续走廊参数

这些参数是当前大场景 benchmark 的有限先验，不应直接当作通用未知地图参数：

| 参数 | 值 | 作用 |
| --- | ---: | --- |
| `side_bias_enabled` | `true` | 启用有限范围路线 hint |
| `side_bias_preferred_y_sign` | `1` | 偏好世界坐标 `+Y` 北侧 |
| `side_bias_world_x_min/max` | `-7.2 / 3.45 m` | 连续覆盖西侧和中央障碍窗口 |
| `side_bias_max_cost` | `45` | 反向侧软代价上限 |
| `side_bias_distance_scale` | `0.9 m` | 反向侧代价尺度 |
| `side_bias_apply_to_unknown` | `true` | 仅在有限窗口处理未知栅格 |
| `side_bias_unknown_base_cost` | `1` | 未知栅格软代价基线 |
| `side_bias_target_world_y_enabled` | `true` | 使用固定世界坐标目标带 |
| `side_bias_reference_world_y` | `0.0 m` | 北/南侧参考线 |
| `side_bias_target_world_y` | `0.75 m` | 连续北侧绕行目标带中心 |
| `side_bias_target_offset` | `0.75 m` | 非固定模式下的目标偏移，v3 固定模式不漂移 |
| `side_bias_target_max_cost` | `100` | 目标带偏离代价上限 |
| `side_bias_target_distance_scale` | `0.60 m` | 目标带代价尺度 |
| `side_bias_target_exponent` | `2.0` | 二次代价曲线 |

## 安全与地图参数

| 类别 | 冻结值 |
| --- | --- |
| footprint | `[[0.30,0.24],[0.30,-0.24],[-0.30,-0.24],[-0.30,0.24]]` |
| footprint padding | `0.03 m` |
| local costmap | `6 x 5 m`, resolution `0.05 m`, update `10 Hz` |
| global costmap | `24 x 17 m`, origin `(-12,-8.5)`, resolution `0.05 m` |
| local/global inflation | `radius=0.45 m`, `cost_scaling_factor=3.0` |
| obstacle cloud | `/camera/obstacles`, max range `3.8 m`, height `0.08--1.5 m` |
| collision cloud | `/camera/cloud`, source timeout `0.5 s` |
| PolygonStop | 前向约 `0.38 m`，横向约 `±0.29 m` |
| PolygonSlow | 前向约 `1.05 m`，横向约 `±0.32 m`，ratio `0.75` |
| Gazebo command path | `/cmd_vel -> /cmd_vel_safe -> turtlebot3_diff_drive` |
| LiDAR | 不使用 `/scan` |

## 启动与回归

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'"
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_north_045_v3'
```

第二个终端发送目标：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

正式记录：

```bash
./scripts/regression_leg.sh \
  --x 8.5 --y 0.0 --yaw 0.0 \
  --label optimization_2026-08-21/fast_north_045_v3_A_to_B_new \
  --profile fast_north_045_v3
```

## 历史 profile

```text
fast_north_045_v3        当前冻结、三次正式成功
fast_north_045_v2        上一轮固定西侧窗口、三次正式成功
fast_north_045_v1        更早的动态目标带探索版
frozen_goal_line_045_v1  旧目标线 0.45 baseline，速度约 0.22 m/s
```

参数 profile 可以恢复启动时的参数覆盖；若需代码级精确复现旧 v2，请使用实验目录
`experiment.yaml` 中记录的提交建立 worktree。当前分支的 v3 只修改 launch profile，
没有改动 `GoalLineSmacPlanner` 的 C++ 算法。

## v2 与 v3 的选择

v3 是当前速度和路线一致性冻结值；v2 的末端采样误差更小，但路线方差和平均耗时更大。
如果下一轮重点是目标附近停稳精度，应从 v3 单独调 goal checker/进场控制，并保留
这两个 profile 作为对照，不要覆盖现有结果目录。
