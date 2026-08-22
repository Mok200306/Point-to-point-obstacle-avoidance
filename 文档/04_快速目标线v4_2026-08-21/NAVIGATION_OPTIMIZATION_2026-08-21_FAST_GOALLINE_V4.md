# 2026-08-21 目标线快速走廊 v4 回归

更新时间：2026-08-21

## 结论

本轮把 `fast_goalline_045_v4` 作为下一轮优化配置完成了 3 次独立 A -> B 回归。
它针对前一轮“第一次绕障时路线偶尔抬得过高、整体时间接近 100 秒”的问题，增加了
固定世界坐标的分段走廊，并让 RPP 在更长前视下以 `0.30 m/s` 跟踪。结果如下：

- Nav2 `status=4`：3/3；
- 过滤 `ground_plane` 后 Gazebo 非地面接触：0/3；
- wall 时间平均 `81.07 +/- 3.46 s`，三次均低于 100 s；
- Gazebo 真实路径平均 `17.382 +/- 0.050 m`；
- 末端采样误差平均 `0.290 +/- 0.053 m`，仅用于 TF/odom 诊断，不替代 Nav2 goal checker；
- 三次轨迹均从目标线平滑进入北侧走廊，通过障碍后逐段回到 `y=0`，没有再出现 `y=-2 m`
  的反向大弯。

这不是“任意未知房间中的全局最优”证明。走廊 schedule 使用了当前 benchmark 的已知
几何先验；在线建图的观测顺序仍会让第一段最大 `y` 在 `1.175--1.380 m` 之间变化。
但三次都保持同一拓扑、无物理碰撞且时间显著低于旧 v3，因此 v4 是当前这个仿真场景的
推荐快速 profile。旧 profile 的参数文档和 Git 提交仍可回退；旧结果目录的整理规则见
[EXPERIMENT_ARCHIVE_INDEX.md](../00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。

## 实验条件

| 项目 | 值 |
| --- | --- |
| Gazebo world | `indoor_obstacle_course_large.world` |
| 起点 | `(-8.5, 0.0, yaw=0)` |
| 目标 | `(8.5, 0.0, yaw=0)` |
| 模式 | `online=true`, `localization=false`, `reset_db=true` |
| 传感器 | Gazebo RGB-D，未使用 `/scan` |
| planner | `rtabmap_tb3_nav/GoalLineSmacPlanner` / SmacPlanner2D |
| controller | `RegulatedPurePursuitController` |
| 运行时 HEAD | `452b45f` |
| 最终可复现代码 | `78bb860` |
| 启动稳定期 | `5 s`，不计入导航 wall 时间 |

## v4 改动

v4 只通过 launch profile 覆盖参数，未缩小 footprint、inflation radius、停止区或碰撞
检测。目标走廊是软代价偏好；lethal cell、footprint、RPP collision check 和
collision monitor 仍然优先。

| 参数 | v3 | v4 | 作用 |
| --- | ---: | ---: | --- |
| `FollowPath.desired_linear_vel` | `0.26` | `0.30 m/s` | 缩短直线段导航时间 |
| `FollowPath.lookahead_dist` | `0.75` | `0.80 m` | 提前进入弧形绕障 |
| RPP 动态前视 | `0.56--1.15 m` | `0.62--1.20 m` | 减少贴着拐角急转 |
| `lookahead_time` | `1.5` | `1.7 s` | 速度变化时保持更稳定的前视 |
| 全局/局部 `cost_scaling_factor` | `3.0` | `4.5` | 让软代价更快回落，不改变硬障碍 |
| 全局/局部 `inflation_radius` | `0.45` | `0.45 m` | 保持原有安全梯度 |
| velocity smoother 上限 | `0.26` | `[0.30, 0, 0.90]` | 与 RPP 目标速度一致 |
| 走廊目标 `y` | v3 单一 `0.75` | `0.95 -> 0.75 -> 0.60 -> 0.58 -> 0` | 随障碍布局逐段回正 |
| 起步稳定期 | 无 | `5 s` | 让在线地图先更新，单独记录且不混入导航时间 |

v4 schedule 的 x/y 控制点为：

```text
x = [-7.2, -3.4, -2.6, -2.25, 2.75, 3.20, 3.50, 7.40, 7.90]
y = [ 0.95,  0.95,  0.75,  0.60, 0.60, 0.68, 0.58, 0.58, 0.00]
```

它表达的是“障碍前提前进入可行走廊、障碍结束后逐步回到目标线”，不是把一条固定
轨迹直接发送给底盘。

## 三次正式结果

| 运行 | status | wall [s] | sim [s] | map 路径 [m] | Gazebo 路径 [m] | 末端误差 [m] | 第一障碍段 y [m] | approx clearance [m] | contacts |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 01 | 4 | 79.01 | 63.50 | 17.341 | 17.336 | 0.361 | 0.731--1.175 | 0.0161 | none |
| 02 | 4 | 78.26 | 63.60 | 17.448 | 17.451 | 0.234 | 0.709--1.380 | 0.0166 | none |
| 03 | 4 | 85.94 | 62.50 | 17.328 | 17.359 | 0.274 | 0.670--1.369 | 0.0171 | none |
| **平均 +/- 标准差** | **3/3** | **81.07 +/- 3.46** | **63.20 +/- 0.49** | **17.377 +/- 0.052** | **17.382 +/- 0.050** | **0.290 +/- 0.053** | **max y=1.308 +/- 0.093** | **0.0166 +/- 0.0004** | **0/3** |

近似 clearance 是 SDF 障碍距离减去 footprint 外接圆半径的保守诊断值；物理碰撞结论
以完整 contacts 过滤结果为准。三次分别监听了 `191064`、`195414`、`186567` 条
Gazebo contacts 消息，均没有 `waffle` 与墙、barrier、crate、pillar 的接触对。

## 轨迹证据

每个目录包含基础参数、profile 覆盖、世界文件、CSV、指标和左右双视图。左侧是
Gazebo SDF 场景与 `/gazebo/model_states` ground truth，右侧是 `/map`、global costmap
和 map-frame 轨迹。

- [run 01](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01)，[双视图](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01/trajectory_comparison.png)
- [run 02](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_02)，[双视图](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_02/trajectory_comparison.png)
- [run 03](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_03)，[双视图](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_03/trajectory_comparison.png)

## 与 v3 对比

| 指标 | `fast_north_045_v3` | `fast_goalline_045_v4` | 变化 |
| --- | ---: | ---: | ---: |
| wall 平均 | `91.08 +/- 0.61 s` | **`81.07 +/- 3.46 s`** | 约快 11.0% |
| Gazebo 路径平均 | `17.492 +/- 0.074 m` | **`17.382 +/- 0.050 m`** | 略短 |
| 3 次成功 | `3/3` | `3/3` | 不变 |
| 非地面 contacts | `0/3` | `0/3` | 不变 |

v4 的时间波动比 v3 大约 `2.85 s`，原因仍可能是在线地图更新和 RGB-D 局部代价地图
的时序；它不是每次像某一张图那样逐像素一致。若论文强调路线一致性，应同时报告
均值、标准差和三张图，不应只挑选 run03。

## 如何运行

只看 Gazebo + RViz2：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

仿真稳定后，在第二个终端发送目标；这里的 5 秒等待用于复现实验启动时序：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

要生成完整双视图和 contacts 证据，使用：

```bash
sg docker -c './scripts/regression_leg.sh \
  --x 8.5 --y 0.0 --yaw 0.0 \
  --settle-seconds 5 \
  --label manual/fast_goalline_045_v4_A_to_B \
  --profile fast_goalline_045_v4'
```

## 后续边界

当前 v4 已达到本场景的“无碰撞、低于 100 s、方向总体一致”验收目标，但还不是理论
极限。下一轮若继续优化，应一次只改一个因素，例如把第一段目标带的代价强度作为
独立实验，或增加地图稳定期；每个候选仍必须完整跑 3 次。不要用缩小 footprint、
关闭 RPP collision check 或削弱 `PolygonStop` 来换取速度。
