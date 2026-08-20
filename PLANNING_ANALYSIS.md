# RGB-D 导航规划分析与本轮改进

更新时间：2026-08-20

## 1. 现象和结论

截图中的橙色线是 Nav2 的原始全局路径 `/plan`，青色线是局部控制轨迹
`/local_plan`。本轮 RViz 配置另外显示绿色 `/received_global_plan`，它是控制器
实际收到的路径。这样可以区分“全局规划器的折线路径”和“局部控制器正在跟随的路径”。

旧版在线模式曾通过 `StaticLayer.enabled=false` 保留插件。实际日志证明 Humble 仍会
订阅 RTAB-Map 发布的增长中 `/map`，并不断输出 `StaticLayer: Resizing`。机器人看到
新区域时，全局网格被反复重建，规划器和控制器可能短暂停顿。旧版 `NavFn + DWB`
还容易选择贴着膨胀层边缘的最短可行路线。

当前在线模式保留 `StaticLayer`，但不直接订阅不断增长的 `/map`：`map_padder.py` 将
RTAB-Map 的地图复制到固定尺寸 `/nav_map`，全局 costmap 使用它叠加实时 RGB-D
obstacle layer；定位模式才把 `StaticLayer` 指向保存的 `/map`。这样既保留已观测
障碍，又避免地图增长导致的额外 resize 停顿。

## 2. 当前和本轮的规划链路

```text
Gazebo RGB-D
  -> RTAB-Map /map + map->odom
  -> global costmap (online): fixed /nav_map StaticLayer + live PointCloud obstacle layer
  -> SmacPlanner2D: cost-aware A* + collision-safe path smoothing
  -> stable behavior tree: path valid/expired/goal changed 才重算
  -> Regulated Pure Pursuit: forward carrot + curvature/cost speed regulation
  -> velocity_smoother
  -> collision_monitor (/cmd_vel_safe)
  -> Gazebo
```

本轮使用的两个 Nav2 规划/控制插件都已经存在于当前 Humble 镜像中：

- `nav2_smac_planner/SmacPlanner2D`
- `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

## 3. 为什么 Smac 比原来的 NavFn 更适合本场景

原来的 NavFn 更接近“在网格上找一条较短的可行路径”。当障碍物附近仍然可以通行时，
最短路径可能沿着膨胀层边缘走。它没有单独的“尽量走开阔区域”目标。

本轮 Smac 2D 使用 A*，并设置：

```yaml
cost_travel_multiplier: 6.0
```

它会把代价地图中的高代价单元累计到路径代价中。这样一条稍长但更开阔的路线，
可以优于一条贴着障碍边缘的短路线。Smac 自带的 smoother 再对路径做平滑和碰撞检查，
为局部控制器提供连续性更好的路径方向。

用户将 `inflation_radius` 改到 `0.30 m` 后速度有所改善，但这不是“障碍物距离”参数。
Waffle 加 padding 后内切半径约 `0.27 m`，因此 `0.30 m` 只剩约 `3 cm` 的软代价梯度，
Smac 缺少足够信号去区分“贴边”和“开阔”。当前 clearance-first 配置使用 `0.55 m`
和 `cost_scaling_factor=3.0`，不改变硬 footprint，只扩大离障碍更远的路径代价偏好。

## 4. 为什么改用 Regulated Pure Pursuit

DWB 每个周期采样 56 条速度轨迹，再用多个 Critic 打分。它适合通用局部搜索，但在
当前 RGB-D 在线场景中会出现：

- 全局路径一变就重新评价大量候选轨迹；
- 路径跟随代价和障碍代价互相竞争；
- 障碍物刚进入局部 costmap 时，候选轨迹可能全部被判高代价或无效；
- 机器人短暂停止后才开始转向。

Regulated Pure Pursuit 直接追踪平滑路径上前方的 carrot 点：

```yaml
lookahead_dist: 0.70
min_lookahead_dist: 0.52
max_lookahead_dist: 1.10
lookahead_time: 1.5
use_rotate_to_heading: true
rotate_to_heading_min_angle: 1.20
```

它会提前看向前方路径，而不是等车头贴到障碍物才转弯。同时启用：

- 曲率调速：弯得越急，速度越低；
- cost 调速：靠近代价区域时减速；
- 前向碰撞预测：预测到碰撞时停止；
- collision monitor：作为最后一道硬安全层。

因此它实现的是“全局路径决定绕行方向，前视控制器决定平滑弧线和速度”，不是让
控制器在障碍物前临时随机选择方向。

## 5. 重规划频率的改变

新增行为树：

```text
src/rtabmap_tb3_nav/behavior_trees/navigate_to_pose_stable_replanning.xml
```

`navigate_through_poses_stable_replanning.xml` 是多目标 action 的对应版本；两者都已
由 `demo.launch.py` 注入 Nav2 参数。

行为规则是：

- 每 0.5 秒检查一次路径状态；
- 路径有效时最长约 20 秒才重新计算；
- 目标改变时重新计算；
- 路径失效时重新计算；
- 局部控制器每个控制周期仍然做碰撞预测。

这样不会因为每个 RGB-D 点云更新就立即让全局路径跳变，同时保留了障碍物真正堵住
原路径时的重新规划能力。

## 6. 当前关键参数

文件：[nav2_rgbd_params.yaml](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)

| 参数 | 当前值 | 作用 |
|---|---:|---|
| 全局/局部 `inflation_radius` | `0.55 m` | 硬 footprint 外的软 clearance 梯度 |
| 全局/局部 `cost_scaling_factor` | `3.0` | inflation 梯度衰减 |
| Smac `cost_travel_multiplier` | `6.0` | 更强惩罚高代价、贴障碍路径 |
| RPP `desired_linear_vel` | `0.22 m/s` | 直线段目标速度 |
| RPP lookahead | `0.52--1.10 m` | 提前产生转弯趋势 |
| RPP `use_rotate_to_heading` | `true`, threshold `1.20 rad` | 普通弯道连续跟踪，终点大角度误差时对齐 |
| RPP `regulated_linear_scaling_min_radius` | `0.75 m` | 曲率变大时减速 |
| RPP `cost_scaling_dist` | `0.55 m` | 靠近障碍代价区域时调速 |
| collision monitor slowdown | `1.05 m`, `0.65` | 安全减速区 |
| collision monitor stop | `0.38 m` | 硬停止区 |

## 7. 验证时要看什么

```bash
ros2 topic echo /plan --once
ros2 topic echo /received_global_plan --once
ros2 topic echo /local_plan --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_safe --once
```

判断停顿来源：

- `/cmd_vel` 有速度但 `/cmd_vel_safe` 为零：collision monitor 停止；
- `/cmd_vel` 本身为零：局部控制器没有输出可行速度或正在转向；
- `/received_global_plan` 是平滑路线但 `/local_plan` 贴边：局部控制器或 costmap 问题；
- 两条全局路径都贴边：全局 costmap、Smac 代价或地图输入问题。

## 8. 修改、构建和启动

本轮已经把 clearance-first 配置和新规划链路写入源码。以后只改 YAML、行为树、RViz 或 launch
时，重建工作区即可：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

只有修改 `Dockerfile` 或 Gazebo 模型补丁时才需要重新构建镜像：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/build.sh'
sg docker -c './scripts/start.sh'
```

重新启动后，先等待 Nav2 lifecycle 进入 active，再发送目标：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

## 9. 当前实验结论边界

本轮参数和规划器替换完成后，需要重新跑 A→B、B→A，才能比较：

- 是否减少贴边；
- 是否减少障碍物前的零速停顿；
- 是否降低总耗时；
- 是否仍然没有 Gazebo 非地面碰撞。

此前 `0.50 m + NavFn + DWB` 的双向结果不能直接作为本轮新配置的结果。新的回归数据
应使用新的 label 单独保存，不能覆盖旧基线。
