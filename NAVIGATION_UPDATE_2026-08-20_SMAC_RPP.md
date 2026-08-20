# 2026-08-20 导航规划更新记录

> 历史阶段记录：本文记录 `0.30 m + Smac/RPP` 候选配置。当前推荐配置和最终双向
> 回归以 [NAVIGATION_UPDATE_2026-08-20_CLEARANCE_FIRST.md](NAVIGATION_UPDATE_2026-08-20_CLEARANCE_FIRST.md)
> 为准，当前源码已经使用 `inflation_radius=0.55 m`、`cost_travel_multiplier=6.0`
> 和 RPP 目标姿态兼容配置。

## 本次问题

机器人在障碍物前经常停顿，绕行路线贴着障碍物膨胀层，明明存在更开阔的方向却没有
提前转弯。用户把 `inflation_radius` 调整为 `0.30 m` 后速度有所改善，但路径选择
问题仍然存在。

## 原因定位

当前任务本来就是全局规划加局部控制，并不是只有一层规划：

```text
RTAB-Map / RGB-D costmap -> 全局规划器 -> 局部控制器 -> 安全过滤 -> Gazebo
```

旧版 `NavFn + DWB` 的主要问题有两个：

1. NavFn 更偏向寻找较短的可行栅格路径。当障碍边缘尚未达到 lethal 时，最短路径可能
   沿 inflation 边缘通过；它不会天然把“离障碍更远”作为独立的强偏好。
2. DWB 需要在每个周期采样速度轨迹并评价多个 critic。全局路径改变时，候选轨迹和
   costmap 评价一起改变，RGB-D 新障碍刚进入地图时可能出现短暂停顿，然后沿着新路径
   边缘跟随。

此外，之前在线模式只是把 `StaticLayer.enabled` 设为 false。Humble 实测仍然出现：

```text
StaticLayer: Resizing static layer ...
```

RTAB-Map 的 `/map` 增长会触发全局网格反复 resize，造成额外规划延迟。它不是障碍物
本身造成的唯一原因，但会放大“停下来思考”的现象。

## 本轮实现

当前链路已经调整为：

```text
Gazebo RGB-D
  -> RTAB-Map /map + map->odom
  -> online fixed rolling global costmap + RGB-D obstacle layer
  -> SmacPlanner2D cost-aware A* + path smoother
  -> stable replanning behavior tree
  -> Regulated Pure Pursuit (RPP)
  -> velocity_smoother
  -> collision_monitor
  -> /cmd_vel_safe -> Gazebo
```

具体变化：

- 在线模式从临时 global costmap 插件列表移除 `StaticLayer`，不再让增长中的 `/map`
  触发全局网格 resize；`localization:=true` 时仍使用保存地图的 StaticLayer。
- 全局规划器替换为 `nav2_smac_planner/SmacPlanner2D`，设置
  `cost_travel_multiplier: 3.0`，让经过高代价区域的累计代价更高。
- 启用 Smac 的平滑器，使全局路径为 RPP 提供连续方向变化。
- 局部控制器替换为
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`。
  RPP 使用 `0.40--0.85 m` 自适应前视，并按曲率、cost 和前向碰撞预测调速，目标是
  在障碍角点前开始弧形转向。
- 单目标和多目标 action 使用独立的稳定重规划行为树：每 0.5 秒检查路径，路径有效时约 10 秒才重算，
  目标改变或路径失效时立即重算；控制器每个周期仍做碰撞检查。
- `controller_server.failure_tolerance` 提高到 `1.0 s`，允许 RPP 在 RGB-D 障碍层更新
  和重新规划的短暂交接期保持停止，而不是立即 abort；`collision_monitor` 仍是独立
  的硬安全层。
- 保留用户当前 `inflation_radius: 0.30`，同时将 global/local
  `cost_scaling_factor` 设为 `2.0`，让 0.30 m 代价带更平滑。车体 footprint
  `0.60 x 0.48 m` 和 padding `0.03 m` 没有缩小。

## 这不是怎样实现的

没有把“当前位置到目标的直线”硬编码成最高分路径。未知环境中这条直线可能穿过尚未
看到的障碍，且会绕过 costmap 的碰撞约束。当前实现的等价目标分工是：

- Smac 的启发式保持目标方向偏好；
- costmap 的 inflation cost 和 Smac 累计代价惩罚贴障碍路径；
- smoother 和 RPP 把绕行路径变成提前、连续、可跟踪的转向；
- collision monitor 只在真实近碰撞风险时减速或停止。

## 可调参数入口

主文件：`src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml`

| 目的 | 参数 |
| --- | --- |
| 让全局路线更远离障碍 | `GridBased.cost_travel_multiplier`、`cost_scaling_factor`、`inflation_radius` |
| 让 RPP 更早转弯 | `lookahead_dist`、`min_lookahead_dist`、`max_lookahead_dist` |
| 调整直线速度 | `FollowPath.desired_linear_vel`、`velocity_smoother.max_velocity[0]` |
| 调整硬安全边界 | `footprint`、`footprint_padding`、`collision_monitor` 的 `PolygonStop` |
| 调整代价地图实时性 | local/global `update_frequency`、点云范围和 `observation_persistence` |

推荐调参顺序：先确认点云和 TF 正常，再调 `cost_travel_multiplier`，然后调 RPP 前视，
最后才考虑 inflation 或 footprint。每一组改动都必须重新跑 A→B、B→A，并检查
Gazebo contacts；不能只根据 RViz 中紫色区域宽窄判断安全性。

## 运行和验证

修改 YAML、launch 或行为树后：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

启动后确认实际加载的是当前链路：

```bash
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 param get /controller_server FollowPath.plugin && ros2 param get /planner_server GridBased.plugin && ros2 param get /bt_navigator default_nav_to_pose_bt_xml && ros2 param get /global_costmap/global_costmap plugins'"
```

在线模式应显示：

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
nav2_smac_planner/SmacPlanner2D
['obstacle_layer', 'inflation_layer']
```

发送目标：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

完整实验用：

```bash
sg docker -c './scripts/run_navigation_trial.sh --x 8.5 --y 0.0 --yaw 0.0 --label smac_rpp_030_A_to_B'
sg docker -c './scripts/run_navigation_trial.sh --x -8.5 --y 0.0 --yaw 3.14159265 --label smac_rpp_030_B_to_A'
```

## 证据边界

此前的 `NavFn + DWB` A→B `182.87 s`、B→A `161.07 s` 以及对应 contacts 是历史对照，
不能直接写成当前 Smac + RPP 的结果。本次配置需要用 `smac_rpp_*` 标签重新记录
耗时、轨迹、末端误差、最小障碍距离和 contacts。

即使新配置双向成功，也只能说明当前大场景中的仿真 RGB-D 在线建图、点到点导航和绕行
闭环通过；不能推导出任意未知房间、窄通道或真实 D435i 已达到工程级零碰撞。
