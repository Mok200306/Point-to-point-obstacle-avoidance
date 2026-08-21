# 2026-08-21 目标线速度优化回归

更新时间：2026-08-21

## 结论

本轮针对 `fast_goalline_045_v1` 的两个问题做了受控优化：一是第一根障碍栏外侧
偶尔绕得过高，二是整体耗时仍接近 100 秒以上。新增 profile：
`fast_goalline_045_v2`，代码提交为 `452b45f`。

v2 在同一大场景、同一 A -> B 目标、在线建图模式下完成 3 次正式回归：

- Nav2 `status=4`：3/3；
- Gazebo contacts 过滤 `ground_plane` 后无机器人与障碍物接触：0/3；
- 平均墙钟时间 `88.95 s`，低于 v1 的 `96.53 s`；
- 平均 Gazebo 路径 `17.31 m`，低于 v1 的 `17.52 m`；
- 平均记录末端误差 `0.180 m`，仅作为 TF/odom 诊断值，不替代 Nav2 内部 goal checker；
- 两次轨迹最大北向偏移约 `0.79 m`，一次为 `1.37 m`。因此路线平均改善，但在线
  未知地图带来的偶发高抬路线仍存在，v2 是速度候选，不替换默认的稳定 v3 基线。

## 实验条件

| 项目 | 值 |
| --- | --- |
| Gazebo world | `indoor_obstacle_course_large.world` |
| 起点 | `(-8.5, 0.0)`，车头朝 `+X` |
| 目标 | `(8.5, 0.0, yaw=0)` |
| 模式 | `online=true`, `localization=false`, `reset_db=true` |
| 传感器 | 模拟 RGB-D，无 `/scan` |
| 规划器 | `rtabmap_tb3_nav/GoalLineSmacPlanner` |
| 控制器 | `RegulatedPurePursuitController` |
| 代码提交 | `452b45f` |

## v2 改动

v2 只通过 launch profile 覆盖参数，未改变硬碰撞边界：

| 参数 | v1 | v2 | 意义 |
| --- | ---: | ---: | --- |
| `FollowPath.desired_linear_vel` | `0.26` | `0.28 m/s` | 提高直线段速度 |
| `velocity_smoother.max_velocity[0]` | `0.26` | `0.28 m/s` | 与 RPP 上限一致 |
| 全局/局部 `inflation_radius` | `0.45` | `0.45 m` | 不改变软代价带宽度 |
| 全局/局部 `cost_scaling_factor` | `3.0` | `4.5` | 更快离开软代价，不为完全清空代价带而多绕行 |
| RPP `inflation_cost_scaling_factor` | `3.0` | `4.5` | 与 costmap 梯度解释一致 |
| `side_bias_target_max_cost` | `100` | `140` | 更强地保持分段目标走廊 |
| `side_bias_target_distance_scale` | `0.60` | `0.50 m` | 更快惩罚偏离走廊 |
| 分段目标走廊 | unchanged | unchanged | `y=[0.75,0.75,0.30,0.30,0.50,0.50]` |
| footprint / padding | unchanged | unchanged | `0.60 x 0.48 m` / `0.03 m` |
| `PolygonStop` | unchanged | unchanged | 前方约 `0.38 m` |
| RPP collision check | enabled | enabled | 未关闭安全检查 |

`cost_scaling_factor` 变大表示 inflation 代价随距离衰减得更快；它不是缩小实体车体，
也不是关闭碰撞检测。障碍物、lethal 栅格、footprint、RPP 前向碰撞预测和 collision
monitor 仍然有效。

## 三次正式结果

| 次数 | status | wall [s] | sim [s] | map 路径 [m] | Gazebo 路径 [m] | 末端误差 [m] | 最大 `y` [m] | approx clearance [m] | contacts |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 01 | 4 | 85.46 | 92.10 | 16.75 | 17.24 | 0.153 | 0.787 | -0.017 | none |
| 02 | 4 | 93.85 | 93.60 | 16.53 | 17.18 | 0.219 | 0.782 | -0.030 | none |
| 03 | 4 | 87.53 | 95.20 | 17.48 | 17.50 | 0.168 | 1.372 | -0.060 | none |
| **平均** | **3/3** | **88.95** | **93.63** | **16.92** | **17.31** | **0.180** | **0.980** | **-0.036** | **0/3** |

均值标准差使用 3 次总体标准差：wall `3.57 s`，Gazebo 路径 `0.14 m`，末端误差
`0.028 m`。`approx clearance` 是 SDF 几何距离减去 footprint 外接圆半径的保守诊断，
出现负数不能单独证明碰撞；物理结论以 Gazebo contacts 为准。

每个正式目录当时均包含 `metrics.yaml`、`trajectory.csv`、`gazebo_trajectory.csv`、
`trajectory.png`、`trajectory_comparison.png`、基础参数快照、`profile_overrides.yaml`、
世界文件和 `experiment.yaml`。原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md) 清理，统计和参数仍保留在本文及
提交 `452b45f` 的历史中。

## 与已有 profile 对比

| profile | wall 平均 [s] | Gazebo 路径平均 [m] | 末端误差平均 [m] | 3 次成功 | 最大 `y` 现象 |
| --- | ---: | ---: | ---: | ---: | --- |
| `fast_goalline_045_v1` | 96.53 | 17.52 | 0.193 | 3/3 | `1.35, 1.28, 0.79`，时序敏感 |
| `fast_goalline_045_v2` | **88.95** | **17.31** | **0.180** | **3/3** | `0.79, 0.78, 1.37`，平均更低但仍有 outlier |
| `fast_north_045_v3` | 91.08 | 17.49 | 0.309 | 3/3 | `1.43, 1.33, 1.41`，路线更一致 |

在“无碰撞是硬门槛、满足后时间越短越好”的规则下，v2 比 v1 平均快 `7.58 s`，约
快 `7.9%`。但 v2 的最大 `y` 方差仍明显大于 `fast_north_045_v3`，所以本轮不把
v2 设为默认 profile；默认仍是路线方差较小的 `fast_north_045_v3`。

## 被拒绝的 v3 pilot

新增的 `fast_goalline_045_v3` 只把 RPP 前视改为 `0.70--1.30 m`，希望减少急转。
单次 pilot 结果为：wall `87.80 s`、status `4`、无非地面 contacts，但最大 `y`
仍为 `1.30 m`。它没有解决在线地图观测时序造成的高抬路线，因此不进入正式均值，
也不作为默认配置。pilot 的结论保留在本文，原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](EXPERIMENT_ARCHIVE_INDEX.md) 清理。

## 为什么仍会偶尔绕高

当前是“边建图边导航”。机器人第一次规划时，西侧障碍可能只被 RGB-D 看到一部分，
后续 `/map`、`/nav_map` 和实时 obstacle layer 又会逐步补齐。Smac 只在当前 costmap
上重新选择可行路径，目标线和分段走廊是软偏好，不是硬编码轨迹；如果某次更新把走廊
附近标得更高，规划器仍会暂时选择更高的可行路线。这是在线未知地图的观测顺序问题，
不是 RPP 单独失控。

因此当前没有宣称“任意未知房间中都能每次选择同一条最优路径”。要进一步消除这个 outlier，
下一轮应单独测试“地图相对坐标的走廊偏好”或“先用近距离观测锁定第一障碍，再发送远目标”，
并重新做 3 次回归；不要同时再改 footprint、inflation、RPP 和 collision monitor。

## 如何运行 v2

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v2'
```

另一个终端发送目标：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

要生成同样的完整实验记录，使用：

```bash
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --label manual/fast_goalline_045_v2_A_to_B --profile fast_goalline_045_v2'
```

## 旧结果如何复现

代码 profile 和 Git 提交都保留，没有覆盖旧实验：

```bash
# v1 正式结果使用的源码
git worktree add ../RTAB-Map-8c9d099 8c9d099

# v2 本轮速度候选使用的源码
git worktree add ../RTAB-Map-452b45f 452b45f
```

进入对应 worktree 后，按该目录自己的 `experiment.yaml` 使用同名 profile；运行前先
停止当前 worktree 的 Docker 容器。只在新代码上选择同名参数 profile，不能把它写成
“代码级精确复现”，因为 C++ planner 和 launch 可能已经变化。
