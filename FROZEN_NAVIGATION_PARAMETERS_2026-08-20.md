# 2026-08-20 冻结导航参数

## 冻结状态

当前仿真基线已经冻结为 `inflation_radius=0.55 m`，源码配置位于 [nav2_rgbd_params.yaml](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)，冻结提交为 `be1dabe`。

选择依据不是“0.55 一定最快”，而是六次回归中两组都 3/3 成功时，0.55 的近似净空和末端误差更稳健；0.45 仍作为对照数据，不覆盖、不删除。

## 必须保持的参数

| 参数类别 | 冻结值 |
| --- | --- |
| 在线模式 | `online=true`, `localization=false`, `reset_db=true` |
| 规划器 | `nav2_smac_planner/SmacPlanner2D` |
| `allow_unknown` | `true` |
| Smac tolerance | `0.25 m` |
| Smac max planning time | `1.0 s` |
| Smac `cost_travel_multiplier` | `6.0` |
| Smac smoother | `max_iterations=1000`, `w_data=0.20`, `w_smooth=0.40`, refinement=true |
| 控制器 | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| RPP desired linear velocity | `0.22 m/s` |
| RPP lookahead | `0.70 m`，动态范围 `0.52--1.10 m`，`lookahead_time=1.5 s` |
| RPP curvature regulation | enabled，min radius `0.75 m`，min speed `0.08 m/s` |
| RPP cost regulation | enabled，distance `0.55 m`，gain `1.0`，inflation factor `3.0` |
| RPP collision check | enabled，carrot collision horizon `1.0 s` |
| controller frequency | `15 Hz` |
| progress checker | radius `0.20 m`，allowance `20 s` |
| goal checker | XY `0.12 m`，yaw `0.15 rad` |
| footprint | `[[0.30, 0.24], [0.30, -0.24], [-0.30, -0.24], [-0.30, 0.24]]` |
| footprint padding | `0.03 m` |
| local costmap | `6 x 5 m`，resolution `0.05 m`，update `10 Hz`，publish `5 Hz` |
| global costmap | `24 x 17 m`，origin `(-12,-8.5)`，resolution `0.05 m`，update `2 Hz`，publish `1 Hz` |
| local inflation | `cost_scaling_factor=3.0`, `inflation_radius=0.55 m` |
| global inflation | `cost_scaling_factor=3.0`, `inflation_radius=0.55 m` |
| RGB-D obstacle cloud | `/camera/obstacles`，最大障碍距离 `3.8 m` |
| collision monitor cloud | `/camera/cloud`，高度 `0.08--1.5 m`，`source_timeout=0.5 s` |
| collision stop polygon | 前向约 `0.38 m`，横向约 `+/-0.29 m` |
| collision slow polygon | 前向约 `1.05 m`，横向约 `+/-0.32 m`，slowdown ratio `0.65` |
| velocity smoother max | `[0.22, 0.0, 0.75]` |
| velocity smoother accel | `[0.8, 0.0, 2.0]` |
| velocity smoother decel | `[-1.0, 0.0, -2.0]` |

`inflation_radius` 是障碍物周围的代价梯度范围，不是车体尺寸。车体硬约束由 footprint 和 padding 决定；0.55 m 的作用是让靠近障碍物的可行路径变贵，使 Smac 更倾向于开阔区域，同时不把通道几何尺寸直接扩大成 0.55 m。

## 运行冻结基线

### 看到 Gazebo 和 RViz2

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/build.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

等待 Gazebo、RTAB-Map、Nav2 lifecycle 全部启动后发送 A -> B：

```bash
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --label manual/frozen_055_A_to_B --profile frozen_055'
```

这个命令会生成完整 metrics、CSV、双视图和 contacts 记录。若只想发送目标而不保存完整回归产物：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

### 低负载回归

论文数据建议关闭 Gazebo/RViz GUI 以减少 GPU/CPU 调度噪声：

```bash
sg docker -c './scripts/launch_demo.sh gazebo_gui:=false rviz:=false rtabmap_viz:=false online:=true localization:=false reset_db:=true'
```

每次新实验都应先停止 launch，再执行：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
```

启动参数、世界、起终点和 profile 必须写入结果目录的 `experiment.yaml`。

## 修改参数后的规则

1. 只改一个实验变量，并在提交信息中写清变量和值，例如 `benchmark: test lookahead 0.8m`。
2. 修改 `src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml` 后运行：

   ```bash
   sg docker -c './scripts/start.sh'
   sg docker -c "docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'"
   ```

3. 停止旧 launch 后重新启动；已经运行的 Nav2 进程不会动态读取刚保存的 YAML。
4. 结果目录必须保存实际使用的 YAML 快照，不能只保存最后的源码文件。
5. 至少执行 3 次干净重启；若更改 footprint、速度、传感器或世界，还要重新做 contacts 检查。

## 证据定义

- `nav2_status=4` 且 `succeeded=true`：Nav2 action 成功结束。
- `final_xy_error_m`：导航记录最后一个 map-frame pose 到目标 XY 的欧氏距离。
- `trajectory_length_m`：RViz/map-frame 记录轨迹的逐点折线长度。
- `gazebo_trajectory_length_m`：`/gazebo/model_states` 中 `waffle` ground-truth 的逐点折线长度。
- `minimum_approx_clearance_m`：SDF 障碍物到轨迹点的平面距离减去带 padding footprint 外接圆半径，属于保守近似指标。
- `gazebo_non_ground_contact=false`：contacts 输出中过滤 `ground_plane` 后，没有 `waffle` 与房间障碍物的接触对；这是本项目当前的物理无碰撞证据。

近似净空为负不能单独等价为真实碰撞，必须和 Gazebo contacts 一起解读。相反，contacts 为 none 也不能证明相机永远能看见所有障碍物；真实 D435i 仍需验证遮挡、反光、时间同步和安装 TF。

## 数据位置

- 六次汇总：[BENCHMARK_2026-08-20_SUMMARY.md](BENCHMARK_2026-08-20_SUMMARY.md)
- 0.55 三次：[run_01](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_01)、[run_02](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_02)、[run_03](results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_03)
- 0.45 三次：[run_01](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_01)、[run_02](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_02)、[run_03](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_03)
- 冻结 Nav2 配置：[nav2_rgbd_params.yaml](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)
- collision monitor：[collision_monitor_rgbd_params.yaml](src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml)
- 场景：[indoor_obstacle_course_large.world](src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world)
