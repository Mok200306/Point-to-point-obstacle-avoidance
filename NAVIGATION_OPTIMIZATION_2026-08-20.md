# 2026-08-20 目标线规划优化与三次回归

## 结论

当前源码已在 `inflation_radius=0.45 m` 的冻结基线之上加入目标线偏好规划器，代码提交为
`c18894e`。三次干净重启的 A -> B 回归均满足：

- Nav2 action `status=4`；
- Gazebo contacts 过滤 `ground_plane` 后没有 `waffle` 与墙、barrier、crate、pillar 的接触；
- 目标线偏离障碍后会重新向终点方向收敛；
- 平均墙钟时间比原生 Smac + RPP 的 0.45 基线少约 `1.28 s`。

这证明候选方案在当前大场景中可复现且没有回归，但三次结果仍受在线未知地图的观测顺序影响，
不能表述为任意未知房间中的全局最优规划器。

## 基线与候选

基线是提交 `be04483` 保存的原生 `nav2_smac_planner/SmacPlanner2D` + RPP + 全局/局部
`inflation_radius=0.45 m`。当前候选只增加一个全局规划器插件，保持速度、footprint、
costmap、RPP、场景和起终点不变。

候选规划器 `rtabmap_tb3_nav/GoalLineSmacPlanner` 继承 Nav2 `SmacPlanner2D`：

1. 每次规划调用使用当前实际起点和终点计算线段；
2. 对已知、非 lethal 的自由栅格增加与线段距离相关的二次软代价；
3. `INSCRIBED_INFLATED_OBSTACLE` 以上的障碍和膨胀硬约束不被覆盖，未知栅格也不被伪造为自由空间；
4. 规划完成后恢复原始 costmap，并在临时改写期间持有 costmap 互斥锁；
5. 规划器仍由 Smac 的 A*、inflation cost 和 smoother 决定可行绕行，不是把直线硬编码成可穿墙路径。

当前候选参数是：

```yaml
inflation_radius: 0.45
cost_travel_multiplier: 6.0
line_bias_enabled: true
line_bias_max_cost: 60.0
line_bias_distance_scale: 2.0
line_bias_exponent: 2.0
```

## 三次正式回归

实验条件：`indoor_obstacle_course_large.world`，A `(-8.5, 0.0)` 到 B `(8.5, 0.0)`，
`online=true`、`localization=false`、`reset_db=true`，每次停止旧仿真并重新创建容器和 Gazebo。

| 次数 | 状态 | wall [s] | sim [s] | map 路径 [m] | Gazebo 路径 [m] | 末端误差 [m] | 近似净空 [m] | 最大 `|y|` [m] | contacts |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 01 | 4 / 成功 | 113.90 | 124.40 | 17.942 | 18.312 | 0.156 | -0.0388 | 1.903 | none |
| 02 | 4 / 成功 | 116.72 | 122.80 | 18.140 | 18.280 | 0.170 | -0.0724 | 1.983 | none |
| 03 | 4 / 成功 | 110.28 | 118.60 | 17.326 | 17.339 | 0.160 | -0.0303 | 0.961 | none |
| **平均** | **3/3** | **113.63** | **121.93** | **17.803** | **17.977** | **0.162** | **-0.0471** | -- | **0/3** |

近似净空是 SDF 障碍距离减去 footprint 外接圆半径的保守诊断值，出现负数不能单独证明
发生物理碰撞；本表的物理结论以 Gazebo contacts 为准。

结果目录：

- [run 01](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_01)
- [run 02](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_02)
- [run 03](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_03)

每个目录包含 `metrics.yaml`、`trajectory.csv`、`gazebo_trajectory.csv`、单图、左右双视图、
实际 Nav2/collision monitor 参数快照、世界文件、contacts 摘要和 `experiment.yaml`。
`trajectory_comparison.png` 左侧是 Gazebo SDF 俯视场景与 ground truth，右侧是 RViz 风格
`/map`、全局 costmap 与 map-frame 轨迹。

## 和原生 0.45 基线比较

原生 Smac + RPP 0.45 三次结果的均值来自
[BENCHMARK_2026-08-20_SUMMARY.md](BENCHMARK_2026-08-20_SUMMARY.md)：

| 方案 | 成功率 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | contacts |
| --- | ---: | ---: | ---: | ---: | --- |
| 原生 Smac + RPP，0.45 | 3/3 | 114.91 | 18.173 | 0.271 | 0/3 |
| 目标线二次偏好，0.45 | 3/3 | **113.63** | **17.977** | **0.162** | 0/3 |

在当前验收优先级“无碰撞是硬门槛，门槛满足后时间越短越好”下，目标线候选优于原生 0.45
基线，但时间优势约 `1.1%`，不应夸大为数量级提升。路径长度和末端误差也改善，但样本数
只有三次。

## 路线现象解释

三次中两次选择西侧障碍的南侧，一次选择北侧。在线模式开始时西侧障碍尚未完整进入
RTAB-Map 地图，`allow_unknown=true` 允许 Nav2 先向目标方向行驶；相机看到障碍后，
全局规划器才在当时已知的局部障碍形状上选择绕行方向。因此同一算法不同次运行可能选
不同侧，这是实时未知环境的正常限制，不是目标线代价失效。

三次都在越过西侧 barrier 后逐步回到终点方向；第一次在约 `x=-1.04 m` 回到 `|y|<0.2 m`，
第二次约在 `x=2.39 m` 回到该范围，第三次北侧路线主要受后续障碍的代价梯度影响。RPP
负责沿 Smac 路径连续前视跟踪，目标线插件负责让可行路径在不需要绕障时更快回到起终点方向；
它不能在相机尚未看到的障碍前凭空知道完整场景。

## 当前冻结与边界

当前源码冻结为目标线候选的 0.45 配置，完整参数见
[FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md](FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md)。
原始 0.45 基线仍由 `be04483` 保留，0.55 结果仍作为历史对照保留。

这次验证覆盖的是一个静态、可通行、仿真 RGB-D 场景。它证明了当前链路能完成在线建图、
全局规划、局部跟踪、安全过滤和 A -> B 到达；它没有证明真实 D435i 在遮挡、反光、掉帧、
安装 TF 误差或任意窄通道下都能零碰撞。接入真实设备前仍应先低速验证点云、TF、底盘和
`/cmd_vel_safe`。

## 复现命令

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'"
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

等待 Nav2 lifecycle active 后，在另一个终端运行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --label manual/goal_line_045_A_to_B --profile goal_line_045_v1'
```

重新运行时先停止旧 launch；修改 YAML 或 C++ 插件后必须重新构建并重新启动 Nav2。
