# 2026-08-20 目标线优化版 0.45 参数冻结

## 冻结对象

本文件冻结的是当前目标线规划候选，不覆盖原始基线文件
[FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md](../02_原生规划基准_2026-08-20/FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md)。
原始 0.45 基线提交为 `be04483`；当前优化代码提交为 `c18894e`。

冻结依据是目标线候选三次 A -> B 全部 `status=4`、Gazebo 非地面 contacts 为 none，且
平均墙钟 `113.63 s`，优于原生 0.45 基线平均 `114.91 s`。这不是统计显著性结论，后续
任何参数实验都从当前提交分支出来，保持一次只改一个变量。

## 核心参数

| 类别 | 冻结值 |
| --- | --- |
| 世界 | `indoor_obstacle_course_large.world` |
| 起点 A | `map: (-8.5, 0.0, yaw=0)` |
| 终点 B | `map: (8.5, 0.0, yaw=0)` |
| 在线模式 | `online=true`, `localization=false`, `reset_db=true` |
| 全局规划器 | `rtabmap_tb3_nav/GoalLineSmacPlanner` |
| 规划器基类 | `nav2_smac_planner/SmacPlanner2D` |
| `allow_unknown` | `true` |
| Smac tolerance | `0.25 m` |
| Smac max planning time | `1.0 s` |
| `cost_travel_multiplier` | `6.0` |
| line bias | enabled |
| line bias max cost | `60.0` |
| line bias distance scale | `2.0 m` |
| line bias exponent | `2.0` |
| Smac smoother | `max_iterations=1000`, `w_data=0.20`, `w_smooth=0.40`, refinement=true |
| 局部控制器 | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| RPP desired speed | `0.22 m/s` |
| RPP lookahead | `0.70 m`，动态范围 `0.52--1.10 m`，`lookahead_time=1.5 s` |
| RPP curvature regulation | enabled，min radius `0.75 m`，min speed `0.08 m/s` |
| RPP cost regulation | enabled，distance `0.55 m`，gain `1.0`，inflation factor `3.0` |
| RPP collision check | enabled，carrot horizon `1.0 s` |
| controller frequency | `15 Hz` |
| progress checker | radius `0.20 m`，allowance `20 s` |
| goal checker | XY `0.12 m`，yaw `0.15 rad` |
| footprint | `[[0.30, 0.24], [0.30, -0.24], [-0.30, -0.24], [-0.30, 0.24]]` |
| footprint padding | `0.03 m` |
| local costmap | `6 x 5 m`，resolution `0.05 m`，update `10 Hz`，publish `5 Hz` |
| global costmap | `24 x 17 m`，origin `(-12,-8.5)`，resolution `0.05 m`，update `2 Hz`，publish `1 Hz` |
| local inflation | `cost_scaling_factor=3.0`, `inflation_radius=0.45 m` |
| global inflation | `cost_scaling_factor=3.0`, `inflation_radius=0.45 m` |
| obstacle cloud | `/camera/obstacles`，最大障碍距离 `3.8 m` |
| collision monitor cloud | `/camera/cloud`，高度 `0.08--1.5 m`，timeout `0.5 s` |
| stop polygon | 前向约 `0.38 m`，横向约 `+/-0.29 m` |
| slow polygon | 前向约 `1.05 m`，横向约 `+/-0.32 m`，ratio `0.65` |
| velocity smoother max | `[0.22, 0.0, 0.75]` |
| velocity smoother accel | `[0.8, 0.0, 2.0]` |
| velocity smoother decel | `[-1.0, 0.0, -2.0]` |

主配置：[nav2_rgbd_params.yaml](../../src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)

## 结果证据

- [run 01](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_01)：`113.90 s`，status 4，contacts none
- [run 02](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_02)：`116.72 s`，status 4，contacts none
- [run 03](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_03)：`110.28 s`，status 4，contacts none
- 结果和算法分析：[NAVIGATION_OPTIMIZATION_2026-08-20.md](../03_目标线规划优化_2026-08-20/NAVIGATION_OPTIMIZATION_2026-08-20.md)

## 重构与启动

修改参数或 C++ 插件后，必须停止旧进程、重新构建、重新启动：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'"
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

看到 Gazebo 和 RViz2 后发送 A -> B：

```bash
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --label manual/frozen_goal_line_045_A_to_B --profile goal_line_quad_045_v1'
```

实验脚本会保存当次 YAML、世界文件、map/Gazebo 两条轨迹、左右双视图和 Gazebo contacts
摘要。标签可以写成 `foo/bar`，脚本也兼容误写成 `results/foo/bar`；本次归档已删除历史上
误生成的嵌套 `results/results/` 目录。

## 解释和边界

`inflation_radius=0.45 m` 是软代价梯度，不是要求车体固定离障碍 0.45 m；硬碰撞边界仍由
footprint、padding、RPP collision check 和 collision monitor 决定。目标线偏好只增加已知
自由栅格的偏离代价，不覆盖障碍物，也不替代 RGB-D 对未知区域的实时观测。因此在线模式
仍可能在不同次运行中选择障碍物的不同侧；这不应被写成规划器对未知环境具有完整先验地图。
