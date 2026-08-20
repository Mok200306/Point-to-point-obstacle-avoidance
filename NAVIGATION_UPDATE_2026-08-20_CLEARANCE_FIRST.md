# 2026-08-20 Clearance-First 导航更新

## 本轮结论

本轮针对“机器人贴着障碍物边缘走、遇到障碍才停下来转向、路径重规划看起来很慢”
完成了配置调整和双向回归。当前推荐配置是：

```text
SmacPlanner2D + path smoother
Regulated Pure Pursuit
fixed /nav_map + StaticLayer
RGB-D obstacle layer
collision_monitor
```

A -> B 和 B -> A 都返回 Nav2 action `status=4`，Gazebo contacts 过滤地面后都没有
发现 `waffle` 与墙、barrier、crate 或 pillar 的接触对。

## 为什么会贴边

`inflation_radius` 不是“车体离障碍物必须保持的固定距离”，而是障碍物周围的软代价
梯度。Waffle footprint 加 `0.03 m` padding 后，内切半径约为 `0.27 m`。当
`inflation_radius=0.30 m` 时，硬 footprint 外只剩大约 `0.03 m` 的有效梯度，Smac
看到的“贴边”和“开阔”差别很小，因此短路线可能沿膨胀边缘通过。

当前调整没有缩小硬 footprint，也没有关闭碰撞检查，而是：

- global/local `inflation_radius: 0.55 m`；
- global/local `cost_scaling_factor: 3.0`；
- Smac `cost_travel_multiplier: 6.0`；
- RPP lookahead `0.52--1.10 m`；
- RPP 保留目标姿态对齐，但只有角度误差达到 `1.20 rad` 才优先原地转向；
- controller frequency `15 Hz`；
- 路径有效期从 `10 s` 延长到 `20 s`，只有路径过期、失效或目标改变时才重新规划。

这使全局规划更重视低代价开阔区域，局部控制器更早沿平滑路径产生弧形转向，同时
保留终点姿态和反向目标的收敛能力。

## 当前链路

```text
Gazebo RGB-D
  -> RTAB-Map /map + map->odom
  -> map_padder.py -> fixed /nav_map
  -> global costmap: StaticLayer + live RGB-D obstacle layer + inflation
  -> SmacPlanner2D + smoother
  -> Regulated Pure Pursuit
  -> velocity_smoother
  -> collision_monitor
  -> /cmd_vel_safe -> Gazebo
```

在线模式不是直接把不断增长的 `/map` 交给 StaticLayer。`map_padder.py` 把地图复制
到固定的 `480 x 340`、`0.05 m` 栅格，覆盖 `24 x 17 m`，从而避免 `/map` 尺寸变化
反复重建全局 costmap。在线模式实际仍加载 `static_layer`，其 `map_topic` 是
`/nav_map`；定位模式才使用保存地图的 `/map`。

## 回归结果

| 方向 | Nav2 status | 墙钟/仿真时间 | trial 最后采样 XY 误差 | contacts | 结果 |
| --- | ---: | ---: | ---: | ---: | --- |
| A -> B | `4` | `114.52 s` | `0.152 m` | `280327`，障碍对 `(none)` | 通过 |
| B -> A | `4` | `110.82 s` | `0.208 m` | `285301`，障碍对 `(none)` | 通过 |

轨迹和指标：

- `results/smac_rpp_055_final_A_to_B/`
- `results/smac_rpp_055_final_B_to_A/`

这里的 `trial 最后采样 XY 误差` 是 `navigation_trial.py` 收到 action result 前后
保存的最后一条 TF/odom 轨迹采样，不是 Nav2 内部 goal checker 的判定值。Nav2 已返回
`status=4`，因此不能把这两个采样值写成“导航没有达到目标”；它们应作为记录频率、
TF 时间对齐和末端停稳误差的诊断指标。后续论文实验应同时报告 action status、目标
容差和独立的末端停稳采样。

## 已验证的失败修复

曾试过关闭 `use_rotate_to_heading`。该配置在 A -> B 前半程可以连续移动，但接近
目标时在线地图更新会触发路径重新计算，最终姿态无法稳定收敛，曾出现 `status=6`
并在目标附近反复反馈 `0.39--1.52 m`。因此没有保留这个改动。

当前配置恢复 `use_rotate_to_heading: true`，并将 `rotate_to_heading_min_angle` 提高
到 `1.20 rad`：普通障碍弯道由 RPP 前视和路径曲率连续跟踪，只有终点或大角度反向
目标才允许原地对齐。A -> B / B -> A 最终候选回归均通过。

## 运行

修改 YAML、launch、行为树或 RViz 后：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

发送 A -> B：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

发送 B -> A：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x -8.5 --y 0.0 --yaw 3.14159265"'
```

实际加载参数检查：

```bash
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash; ros2 param get /planner_server GridBased.cost_travel_multiplier; ros2 param get /controller_server FollowPath.lookahead_dist; ros2 param get /global_costmap/global_costmap plugins; ros2 param get /global_costmap/global_costmap static_layer.map_topic'"
```

## 当前边界

本轮证明了当前大场景中 RGB-D 在线建图、点到点导航、开阔方向绕行和无实体障碍接触
的闭环。它不证明任意未知房间、遮挡严重的环境、窄于车体安全宽度的通道或真实 D435i
已经达到工程级可靠性。真实阶段仍需固定安装 D435i、测量相机 TF、接入真实底盘里程计，
并重新做速度、footprint、点云时延和碰撞回归。
