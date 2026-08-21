# 2026-08-21 导航优化与正式回归报告

更新时间：2026-08-21

## 当前结论：`fast_north_045_v3`

本轮在已经验证的 `fast_north_045_v2` 上只增加一项受控改动：把固定北侧走廊从
`barrier_west` 延长到 `barrier_center` 的末端，并将走廊中心固定在 `world_y=0.75 m`。
RPP、速度、inflation、footprint、RGB-D 和 collision monitor 均未改变。

同一场景、同一 A→B 目标、三次干净重启均成功，且路线明显比 v2 稳定：

| 指标 | run 01 | run 02 | run 03 | 平均 ± 标准差 |
| --- | ---: | ---: | ---: | ---: |
| Nav2 status | 4 | 4 | 4 | **3/3 成功** |
| wall 时间 [s] | 90.24 | 91.35 | 91.66 | **91.08 ± 0.61** |
| map 轨迹 [m] | 17.22 | 17.20 | 17.11 | **17.18 ± 0.05** |
| Gazebo 轨迹 [m] | 17.57 | 17.39 | 17.52 | **17.49 ± 0.07** |
| 末端 XY 误差 [m] | 0.261 | 0.344 | 0.321 | **0.309 ± 0.035** |
| 最大北向偏移 y [m] | 1.430 | 1.327 | 1.408 | **1.388 ± 0.044** |
| 近似最小 clearance [m] | 0.009 | 0.004 | -0.035 | -0.007 |
| 非地面 Gazebo contacts | none | none | none | **0/3** |

相对 v2 的三次均值：wall 时间约快 `4.46 s`（约 `4.7%`），Gazebo 路径标准差从
`0.386 m` 降至 `0.074 m`，最大 y 偏移标准差从 `0.422 m` 降至 `0.044 m`。因此
v3 更适合当前这个 benchmark 的“快速且路线一致”目标；它不是任意未知房间的通用
最优规划器。

近似 clearance 是 SDF 几何与 footprint 外接圆的诊断值，v3 的数值更紧，不能把它写成
比 v2 更安全；物理碰撞结论以过滤 `ground_plane` 后的 Gazebo contacts 为准。三次完整
三次结果的统计仍保留在本文；原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md) 清理，可从对应 Git 提交恢复。

每个 `trajectory_comparison.png` 左侧是 Gazebo SDF 俯视场景和 ground truth，右侧是
RViz 风格的 `/map`、global costmap 与 map-frame 轨迹。

## v3 改动与根因

v2 只在 `x∈[-7.2,-2.5] m` 的西侧障碍窗口内约束 `world_y=0.95 m`。离开这个窗口后，
规划器有时选择中央障碍物的上方通道，实际轨迹会出现 `y≈1.9 m` 的大弯。当前世界中
`barrier_center` 位于约 `x∈[-2.3,3.1] m、y∈[1.325,1.875] m`，所以其下方的
`y≈0.75 m` 是更直接的连续通道。

v3 profile 的唯一规划覆盖为：

- `side_bias_world_x_min/max = -7.2 / 3.45 m`；
- `side_bias_target_world_y = 0.75 m`；
- `side_bias_target_max_cost = 100`；
- `side_bias_target_distance_scale = 0.60 m`；
- 仍然只修改非 lethal 栅格的软代价，不覆盖 obstacle、inflation、footprint 或安全停止区。

这不是把一条预先画好的轨迹强塞给机器人：Smac 仍会根据当前 costmap 做可行性判断，
RPP 仍负责前视跟踪、曲率调速和碰撞预测，collision monitor 仍是最后的安全过滤器。
但该 hint 明确使用了当前 benchmark 的几何先验，因此迁移到任意新地图前必须关闭或
重新测量走廊范围。

## v2 对照结果

v2 保留为可复现的速度/路线对照：

| 指标 | v2 平均 ± 标准差 |
| --- | ---: |
| wall 时间 [s] | 95.54 ± 2.03 |
| Gazebo 轨迹 [m] | 17.765 ± 0.386 |
| 末端 XY 误差 [m] | 0.194 ± 0.017 |
| 最大北向偏移 y [m] | 1.588 ± 0.422 |
| 成功率 / 非地面碰撞 | 3/3 / 0/3 |

v2 的三次结果统计仍保留在本文和 Git 历史中；原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md) 清理，不与当前 v3/v4 正式结果
混放。

## 当前规划链路

```text
Gazebo RGB-D -> RTAB-Map /map -> map_padder /nav_map
-> global costmap -> GoalLineSmacPlanner/SmacPlanner2D
-> Smac smoother -> stable behavior tree -> RPP
-> velocity_smoother -> collision_monitor -> /cmd_vel_safe -> Gazebo
```

Smac 决定全局绕行拓扑，RPP 负责平滑前视跟踪和曲率/代价调速，collision monitor
是独立的最后安全过滤器，不负责寻找全局路线。

## 本轮速度与安全参数

- RPP `desired_linear_vel=0.26 m/s`；lookahead `0.75 m`，动态范围 `0.56--1.15 m`。
- velocity smoother 最大线速度 `0.26 m/s`，加速度 `0.9 m/s²`，减速度 `1.1 m/s²`。
- Collision Monitor `PolygonSlow.slowdown_ratio=0.75`，`PolygonStop` 未削弱。
- 全局/局部 `inflation_radius=0.45 m`，footprint 和 `padding=0.03 m` 未缩小。
- 全局 planner `cost_travel_multiplier=6.0`，`allow_unknown=true`。

相对 2026-08-20 目标线三次平均约 `113.63 s`，v3 平均约 `91.08 s`，约快 `19.1%`。
相对 v2 的本轮改动只改变 route hint 的范围和目标 y，因此可以把 v3 与 v2 作为一组
较清晰的受控对照；但在线建图和仿真时序仍会带来重复运行波动。

## 探索版本边界

- `fast_north_045_v1`：动态目标带探索版，保留作回退和失败分析。
- `fast_north_045_v3_pilot_A_to_B`：pilot status `4`，wall `90.77 s`，无非地面接触，
  不纳入三次正式均值。
- `fast_north_045_v4_pilot_A_to_B`：status `6`，终点前约 `2.65 m` 停住，证明了动态
  目标带漂移问题，不能作为成功结果。

## 复现命令

当前代码冻结节点为 `6202912`，默认 profile 为 `fast_north_045_v3`：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'"
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_north_045_v3'
```

无 GUI 回归：

```bash
./scripts/regression_leg.sh \
  --x 8.5 --y 0.0 --yaw 0.0 \
  --label optimization_2026-08-21/fast_north_045_v3_A_to_B_new \
  --profile fast_north_045_v3 --startup-timeout 90
```

## 旧配置回退

```text
fast_north_045_v3        当前冻结、三次正式成功
fast_north_045_v2        上一轮固定西侧窗口、三次正式成功
fast_north_045_v1        更早的动态 side-bias 探索版
frozen_goal_line_045_v1  旧目标线 0.45 baseline，速度约 0.22 m/s
```

若需代码级精确复现，请使用结果目录 `experiment.yaml` 中的 `git_commit` 建立 worktree；
不要用当前分支的新二进制声称复现旧提交的 C++ planner 行为。

## 当前判断

本轮已经从“一张偶然漂亮的图”推进到“同一 profile 三次成功、平均约 91.1 s、零非地面
物理接触、路线方差显著下降”的可复现实验阶段。下一步应冻结 v3，单独优化末端停稳
误差，或进入真实 D435i 迁移；不要同时改变 planner、RPP、inflation 和安全层。

## 如何复现旧实验

当前代码仍支持 `navigation_profile:=fast_north_045_v2`，可在新代码上直接复现 v2
参数覆盖。若需要代码级精确复现 v2 当时的源码，可建立不影响当前工作区的 worktree：

```bash
cd /home/w417/RTAB-Map
git worktree add ../RTAB-Map-v2-9823820 9823820
cd ../RTAB-Map-v2-9823820
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true navigation_profile:=fast_north_045_v2'
```

v3 的三个正式结果使用 v3 profile 补丁运行，补丁随后以提交 `6202912` 原样提交；结果
目录中的 `experiment.yaml` 同时保留运行时 HEAD 和可复现提交，避免把工作区补丁误写成
旧版本结果。
