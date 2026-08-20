# 2026-08-20 导航基准实验汇总

## 结论

本次在同一个大室内障碍物场景中完成了 6 次在线 RGB-D A -> B 导航：

- `inflation_radius=0.55 m`：3/3 成功，成功率 100%，无过滤到的非地面 Gazebo 物理接触。
- `inflation_radius=0.45 m`：3/3 成功，成功率 100%，无过滤到的非地面 Gazebo 物理接触。
- 当前冻结的仿真基线为 `0.55 m`。原因是它在相同成功率下保留了更大的近似净空，且平均末端 XY 误差更小；`0.45 m` 作为对照配置保留。

这只是每组 3 次的工程回归对比，不足以证明统计显著性，也不能据此声称对所有场景都达到工程级零碰撞。

## 固定实验条件

| 项目 | 值 |
| --- | --- |
| 系统 | Ubuntu 20.04 主机 + Docker + Ubuntu 22.04/ROS 2 Humble |
| 仿真 | `indoor_obstacle_course_large.world` |
| 起点 A | `(-8.5, 0.0, yaw=0)` |
| 终点 B | `(8.5, 0.0, yaw=0)`，目标坐标系 `map` |
| 模式 | `online=true`, `localization=false`, `reset_db=true` |
| 规划器 | `nav2_smac_planner/SmacPlanner2D` |
| 控制器 | `RegulatedPurePursuitController` |
| 感知 | Gazebo RGB-D 深度生成 PointCloud2，无真实 LiDAR |
| 物理证据 | Gazebo contacts，过滤 `ground_plane` 后检查 `waffle` 与障碍物接触 |
| 轨迹证据 | 左侧 SDF 俯视图 + Gazebo `/gazebo/model_states` 真值，右侧 RViz 风格 `/map` + 全局代价地图 |
| 重置规则 | 每次运行前停止上一套 launch，重建干净容器和仿真，从 A 点开始 |

0.55 运行时的源码快照为 `516b606`，0.45 运行时的源码快照为 `a1389ff`；实验结束后恢复并冻结 0.55，冻结配置提交为 `be1dabe`。

## 逐次结果

`wall` 是墙钟导航耗时；`sim` 是 Gazebo `/clock` 中记录的仿真耗时。路径长度分别来自 RViz/map 轨迹和 Gazebo ground-truth 轨迹。`clearance` 是基于 SDF 障碍物和带 padding footprint 外接圆的近似值，不是 Gazebo 碰撞检测结果。

| 配置 | 次数 | 状态 | wall [s] | sim [s] | map 路径 [m] | Gazebo 路径 [m] | 末端误差 [m] | 近似净空 [m] | 非地面接触 | 产物 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 0.55 | 01 | 4 / 成功 | 115.16 | 241.80 | 18.202 | 18.396 | 0.177 | 0.0832 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_01/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_01/trajectory_comparison.png) |
| 0.55 | 02 | 4 / 成功 | 118.64 | 134.70 | 18.075 | 18.442 | 0.238 | 0.0981 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_02/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_02/trajectory_comparison.png) |
| 0.55 | 03 | 4 / 成功 | 115.48 | 144.30 | 18.076 | 18.328 | 0.278 | 0.0321 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_03/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_03/trajectory_comparison.png) |
| 0.45 | 01 | 4 / 成功 | 115.83 | 132.70 | 17.808 | 18.187 | 0.264 | 0.0214 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_01/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_01/trajectory_comparison.png) |
| 0.45 | 02 | 4 / 成功 | 113.90 | 133.60 | 17.729 | 18.159 | 0.183 | -0.0044 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_02/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_02/trajectory_comparison.png) |
| 0.45 | 03 | 4 / 成功 | 115.01 | 133.50 | 17.864 | 18.174 | 0.365 | 0.0268 | none | [metrics](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_03/metrics.yaml) / [双视图](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_03/trajectory_comparison.png) |

每个结果目录还保存了 `trajectory.csv`、`gazebo_trajectory.csv`、`trajectory.png`、`trajectory_comparison.png`、`metrics.yaml`、当次 Nav2/collision monitor 参数、世界文件和 `experiment.yaml`，每个目录共 9 个文件。

## 分组统计

均值后面的 `+/-` 为 3 次运行的总体标准差（分母为 `n`），不是置信区间。

| 配置 | n | 成功/失败 | 成功率 | wall [s] | sim [s] | map 路径 [m] | Gazebo 路径 [m] | 末端误差 [m] | 平均近似净空 [m] | 观测最小近似净空 [m] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.55 m | 3 | 3 / 0 | 100% | 116.425 +/- 1.569 | 173.600 +/- 48.384 | 18.118 +/- 0.060 | 18.389 +/- 0.047 | 0.231 +/- 0.041 | 0.0711 +/- 0.0283 | 0.0321 |
| 0.45 m | 3 | 3 / 0 | 100% | 114.913 +/- 0.793 | 133.267 +/- 0.403 | 17.800 +/- 0.055 | 18.173 +/- 0.012 | 0.271 +/- 0.074 | 0.0146 +/- 0.0136 | -0.0044 |

### 如何解释

1. `0.45 m` 的墙钟平均值约快 `1.51 s`，map 路径短约 `0.32 m`，Gazebo 真值路径短约 `0.22 m`。这说明较小 inflation 允许更贴近障碍物的路线，并没有在这 3 次中造成导航失败。
2. `0.55 m` 的平均近似净空约高 `0.0565 m`，观测最小值也高于 0；同时末端误差平均小约 `0.0395 m`。对当前“可到达且尽量不贴障碍物”的基线目标，安全裕量比少量时间收益更重要。
3. `0.55` run 01 的仿真时间 `241.8 s` 是一项明显离群值，但墙钟时间仍在同一范围。报告中保留它，不删除或用墙钟时间替换，避免掩盖 Gazebo CPU 负载/仿真时钟差异。
4. 0.45 run 02 的近似净空为负，但 contacts 仍为 none。这两个指标的定义不同：负值只表示带 padding footprint 外接圆的保守估计与障碍物距离有重叠，不能单独证明发生了真实物理碰撞；最终物理碰撞判断以 `gazebo_non_ground_contact: false` 为准。

## 物理接触判定

六个 `metrics.yaml` 均包含：

```yaml
gazebo_ground_truth_received: true
gazebo_non_ground_contact: false
gazebo_contact_pairs: "(none)"
```

`gazebo_contact_messages` 是原始 contacts 消息数量，不是碰撞次数；本次六次均已过滤地面并没有留下 `waffle` 与房间障碍物的接触对。

## 当前冻结结论

- 冻结值：local costmap 和 global costmap 都使用 `inflation_radius: 0.55`。
- 对照值：0.45 的完整结果和参数快照保留在 `smac_rpp_045_A_to_B_run_01..03`。
- 其他规划器、控制器、速度、footprint、世界、起点终点和感知参数不因本次对比改变。
- 可复现参数表见 [FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md](FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md)。

## 后续实验边界

下一轮若要比较速度、footprint、RPP lookahead 或 cost scaling，必须以当前 0.55 冻结基线为父版本，一次只改变一个变量，并沿用同样的三次干净重启、双视图、ground-truth 和 contacts 记录规则。
