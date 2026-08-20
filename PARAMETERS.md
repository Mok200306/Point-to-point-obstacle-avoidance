# 仿真导航参数手册

本文对应 `demo.launch.py` 的在线 RGB-D 模式。参数位于
`src/rtabmap_tb3_nav/config/`，用于 Ubuntu 22.04 + ROS 2 Humble 仿真基线，不应
未经真实 D435i、底盘和 TF 验证就直接用于真实机器人。

## 1. 当前规划链路

```text
RGB-D -> /camera/cloud -> /camera/obstacles
                         |
                         v
RTAB-Map /map -> fixed-map padder -> /nav_map
                 Nav2 global costmap (StaticLayer + live obstacle layer)
                         |
                         v
             SmacPlanner2D + path smoother
                         |
                         v
              Regulated Pure Pursuit
                         |
                         v
              velocity_smoother
                         |
                         v
              collision_monitor
                         |
                         v
                    /cmd_vel_safe
```

这是全局规划加局部控制的结构，但两者职责不同：Smac 选择绕行拓扑和全局路径，RPP
沿路径前视跟踪并根据曲率、代价和碰撞预测调速，collision monitor 只负责最后的硬
安全过滤。它不是第三个路径规划器。

在线模式不直接把不断增长的 `/map` 交给 `StaticLayer`，而是由 `map_padder.py` 将其
复制到固定尺寸的 `/nav_map`；`StaticLayer` 仍然保留，用来记住已经观测到的障碍。
实时障碍同时由 `/camera/obstacles` 进入 global/local costmap。这样避免 Humble 因
RTAB-Map 地图尺寸变化反复 resize。定位模式则把 `StaticLayer.map_topic` 改为保存的
`/map`。

## 2. 当前基线参数

| 类别 | 参数 | 当前值 | 作用 |
| --- | --- | ---: | --- |
| 车体 | `footprint` | `0.60 x 0.48 m` | costmap 的硬碰撞几何 |
| 车体 | `footprint_padding` | `0.03 m` | 额外安全余量 |
| 速度 | `FollowPath.desired_linear_vel` | `0.26 m/s` | v3 RPP 直线段目标速度 |
| 速度 | `max_velocity[0]` | `0.26 m/s` | velocity smoother 上限 |
| 速度 | `max_velocity[2]` | `0.85 rad/s` | 角速度上限 |
| 加速度 | `max_accel` / `max_decel` | `0.9` / `-1.1` | 线速度变化限制 |
| RPP 前视 | `lookahead_dist` | `0.75 m` | 默认前视距离 |
| RPP 前视 | `min/max_lookahead_dist` | `0.56 / 1.15 m` | 前视范围 |
| RPP 转向 | `use_rotate_to_heading` | `true`, threshold `1.20 rad` | 普通弯道连续跟踪，终点大角度误差才原地对齐 |
| RPP 调速 | `regulated_linear_scaling_min_radius` | `0.75 m` | 曲率变大时减速 |
| RPP 调速 | `cost_scaling_dist` | `0.55 m` | 靠近障碍代价区时调速 |
| 控制频率 | `controller_frequency` | `15 Hz` | RPP 控制循环 |
| 全局规划 | `GridBased.plugin` | `rtabmap_tb3_nav/GoalLineSmacPlanner` | 带目标线/有限走廊软偏好的 Smac A* |
| 全局规划 | `cost_travel_multiplier` | `6.0` | 放大路径经过的 costmap 代价 |
| 全局规划 | `max_planning_time` | `1.0 s` | 单次规划时间上限 |
| 障碍代价 | `inflation_radius` | `0.45 m` | 当前速度优先冻结值；障碍物外侧软代价梯度半径 |
| 障碍代价 | `cost_scaling_factor` | `3.0` | 梯度衰减；越小，远处代价越明显 |
| 局部地图 | `width x height` | `6 x 5 m` | 机器人周围实时窗口 |
| 地图频率 | local update/publish | `10 / 5 Hz` | 障碍地图更新和发布频率 |
| RGB-D | `camera_cloud.max_depth` | `3.5 m` | 低延迟安全点云范围 |
| RGB-D | costmap obstacle range | `3.8 m` | costmap 使用的最远障碍距离 |
| 安全层 | `PolygonSlow` | `1.05 m`, `0.75` | 前方减速区和速度比例 |
| 安全层 | `PolygonStop` | `0.38 m` | 前方硬停止区 |
| 目标 | `xy_goal_tolerance` | `0.12 m` | 到达 XY 容差 |
| 目标 | `yaw_goal_tolerance` | `0.15 rad` | 到达方向容差 |

## 3. 路径为什么会贴边

你提出的判断方向是对的，但“贴边”不代表障碍物边缘本身是最高分。Nav2 的代价
大致可以这样理解：

```text
实体障碍 + footprint        -> lethal / 不可碰撞
inflation gradient           -> 离障碍越近，栅格 cost 越高
Smac path cost               -> 路径长度 + costmap cost 的累计
RPP                         -> 跟随全局路径，并按曲率/代价降低速度
```

旧版 NavFn + DWB 更容易出现“最短可行路径贴着膨胀层走”：全局路径只要没有穿过
致命栅格就可能沿边缘，DWB 又会较强地追随这条路径。现在改成 Smac 并设置
`cost_travel_multiplier: 6.0`，让一条稍长但低代价的路线更有机会胜过短的高代价
路线；`inflation_radius=0.55 m` 为车体硬边界外保留更长的代价梯度。RPP 使用更长
前视优先沿平滑路径产生弧形转向；仅在终点或大于 `1.20 rad` 的姿态误差时原地对齐，
保证 B→A 这类反向目标也能完成。

当前实现没有额外手写一条“当前位置到目标的直线最高分”路径。原因是这条直线在
未知环境中可能穿过尚未观测的障碍，且一旦强行作为目标会和 costmap 的碰撞约束冲突。
现有安全等价物是：Smac 的 A* 启发式保持目标方向偏好，costmap 累计代价负责绕开
障碍，smoother 和 RPP 负责把折线变成可跟踪的连续转向。

## 4. 如何调离障碍物

当前默认 profile 是 `fast_north_045_v3`；`fast_north_045_v2` 仍可作为旧路线对照。
每次只改一组参数，完成 A -> B 和 B -> A 后再比较轨迹图、耗时和 contacts。

### 4.1 想让全局路线更偏向开阔区域

优先调整：

1. 将 Smac `cost_travel_multiplier` 从 `6.0` 试到 `7.0`，观察是否绕行更早、更远。
2. 若仍贴边，将 `cost_scaling_factor` 从 `3.0` 降到 `2.5`；相同 `inflation_radius`
   下会让较远位置也保留更明显的代价梯度。
3. 若物理通道足够宽，可将 `inflation_radius` 从 `0.55` 试到 `0.60`；若窄口不可行，
   退回 `0.50`，不要先缩小 footprint。
   这会改变代价区宽度，不会改变真实车体尺寸；通道是否可行必须用 footprint
   和 Gazebo contacts 验证。

不要把 `cost_travel_multiplier` 无限增大，也不要把 inflation 直接设为零。否则
规划器可能为了“离障碍远”选择过长路线，或者让地图显示看起来更空但安全余量消失。

### 4.2 想让 RPP 更早、更平滑地转弯

- 增大 `lookahead_dist` 或 `min_lookahead_dist`：更早朝前方路径转向，过大会切弯；
- 增大 `rotate_to_heading_min_angle`：减少普通障碍弯道原地停转，但不能关闭终点姿态对齐；
- 增大 `lookahead_time`：速度变化时前视更稳定；
- 降低 `regulated_linear_scaling_min_radius`：较缓的弯也会更早减速；
- 保持 `use_collision_detection: true`，不要用关闭碰撞检查换取流畅度。

当前 v3 配置的前视值是：

```yaml
lookahead_dist: 0.75
min_lookahead_dist: 0.56
max_lookahead_dist: 1.15
```

如果出现切角、车身靠近障碍或目标附近绕过目标，再退回当前值。

### 4.3 看起来“堵住”或停车

先区分实际来源：

- `/cmd_vel` 有速度，`/cmd_vel_safe` 变成零：collision monitor 触发 stop；
- `/cmd_vel` 本身为零：RPP 正在转向、路径无效、规划失败或生命周期异常；
- 日志是 `Robot to slowdown`：正常安全减速，不等于 action 失败；
- 日志是 `PolygonStop` 持续触发：检查点云、TF、障碍距离和车体 footprint；
- 日志有 `StaticLayer: Resizing`：运行的还是旧在线配置，需要停干净并重建工作区；
- 日志有 `no valid path found` 或 `Failed to make progress`：该次导航应判为失败。

不要先缩小 `footprint` 来强行通过。`inflation_radius` 不是“车体到障碍物的固定
距离”：它只控制硬 footprint 外侧的软代价带；当前 `0.55 m` 才能在约 `0.27 m`
内切半径之外提供约 `0.28 m` 的有效梯度。真实车体和 padding 仍是硬约束。

## 5. 修改、构建和重新启动

修改 YAML、launch、行为树或 RViz 后：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

`demo.launch.py` 会在启动时生成 Nav2 临时参数文件，因此必须重启 launch 才能让
参数生效。修改 `Dockerfile` 或 `scripts/patch_turtlebot3_rgbd.sh` 才需要重建镜像：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/build.sh'
sg docker -c './scripts/start.sh'
```

启动后检查实际参数，而不是只看源码：

```bash
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 param get /controller_server FollowPath.plugin && ros2 param get /planner_server GridBased.plugin && ros2 param get /bt_navigator default_nav_to_pose_bt_xml && ros2 param get /global_costmap/global_costmap plugins'"
```

在线模式预期全局插件为：

```text
['static_layer', 'obstacle_layer', 'inflation_layer']
```

在线模式的 `static_layer.map_topic` 应为 `/nav_map`；定位模式才使用 `/map`。

## 6. 轨迹和实验记录

使用 `run_navigation_trial.sh` 可以同时记录耗时、轨迹 CSV、轨迹 PNG 和最终地图：

```bash
sg docker -c './scripts/run_navigation_trial.sh --x 8.5 --y 0.0 --yaw 0.0 --label smac_rpp_055_A_to_B'
sg docker -c './scripts/run_navigation_trial.sh --x -8.5 --y 0.0 --yaw 3.14159265 --label smac_rpp_055_B_to_A'
```

每次结果在 `results/<label>/`。论文统计建议至少包含：成功率、墙钟/仿真耗时、末端
XY 误差、轨迹长度、最小障碍距离、重规划次数和 Gazebo 非地面 contacts。旧版
`NavFn + DWB` 结果只能作为历史对照，不能和当前 Smac + RPP 结果混写成同一基线。
