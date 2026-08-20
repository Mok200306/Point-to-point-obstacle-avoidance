# 仿真导航参数手册

本文对应 `demo.launch.py` 的在线 RGB-D 模式。参数都在
`src/rtabmap_tb3_nav/config/` 中，修改后需要重新构建工作区，重新启动仿真。
它们是仿真基线参数，不是未经实测即可用于真实机器人安全运行的参数。

## 当前基线

| 类别 | 参数 | 当前值 | 作用 |
| --- | --- | ---: | --- |
| 底盘速度 | `FollowPath.max_vel_x` | `0.18 m/s` | 最大前进速度 |
| 底盘速度 | `FollowPath.max_vel_theta` | `0.75 rad/s` | 最大转向速度 |
| 底盘速度 | `acc_lim_x` / `decel_lim_x` | `0.8 / -1.0` | 线加减速度 |
| 底盘速度 | `acc_lim_theta` / `decel_lim_theta` | `2.0 / -2.0` | 角加减速度 |
| 车体模型 | `footprint` | `0.60 x 0.48 m` | 碰撞几何，不要按显示效果缩小 |
| 车体模型 | `footprint_padding` | `0.03 m` | 额外安全余量 |
| 障碍代价 | `inflation_radius` | `0.50 m` | 障碍物外侧代价梯度范围 |
| 障碍代价 | `cost_scaling_factor` | `3.0` | 梯度衰减；越小，远处代价越明显 |
| DWB 障碍 | `BaseObstacle.scale` | `2.0` | 鼓励轨迹远离代价地图障碍 |
| DWB 障碍 | `ObstacleFootprint.scale` | `2.0` | 用真实 footprint 评价轨迹 |
| DWB 路径 | `PathAlign/PathDist.scale` | `12.0 / 16.0` | 路径跟随；过大容易贴着全局路径 |
| DWB 目标 | `GoalAlign/GoalDist.scale` | `12.0 / 16.0` | 接近目标的方向和距离 |
| DWB 预测 | `vx_samples/vtheta_samples` | `8 / 7` | 速度采样数量 |
| DWB 预测 | `sim_time` | `1.25 s` | 每条候选轨迹预测时长 |
| 局部地图 | `width x height` | `6 x 5 m` | 机器人周围实时障碍窗口 |
| 地图频率 | `update_frequency` | `10 Hz` | 局部 costmap 更新频率 |
| RGB-D | `camera_cloud.max_depth` | `3.5 m` | 低延迟安全/障碍点云深度 |
| RGB-D | `obstacle_max_range` | `3.8 m` | costmap 使用的最远障碍距离 |
| 安全层 | `PolygonStop` | 前方约 `0.38 m` | 硬停止，不用于选路 |
| 安全层 | `PolygonSlow` | 前方约 `1.05 m`，`0.65` | 看到近障碍时降速 |

## 怎么调

### 贴边行走

优先按以下顺序调节：

1. 把 `BaseObstacle.scale` 和 `ObstacleFootprint.scale` 一起提高到 `2.5`，观察
   是否明显远离障碍。
2. 把 `PathAlign.scale`、`PathDist.scale` 各降低 `2--4`，允许 DWB 暂时偏离全局
   路径；不要把它们降到零，否则路径跟随会变得不稳定。
3. 将 `inflation_radius` 调到 `0.55 m`，并将 `cost_scaling_factor` 调到 `2.5`，让
   中间区域代价更平滑。每次只改一组，并重新跑 A->B、B->A。

### 看起来“堵住”或窄口无法通过

`inflation_radius` 的彩色区域不是实体墙；实体不可行区域由障碍物和 footprint
碰撞检查决定。先确认窄口的物理宽度大于车体宽度加两侧安全余量，再按顺序：

1. 查看 `/local_costmap/costmap` 是否把整个通道标成 lethal；
2. 确认 `/camera/obstacles` 和 `/camera/cloud` 仍在更新；
3. 只在物理宽度确实足够时，将 `inflation_radius` 从 `0.50` 降到 `0.45`；
4. 不要先缩小 `footprint`，也不要关闭 `ObstacleFootprint`。

### 停车而不是失败

`PolygonSlow` 触发时 `/cmd_vel_safe` 仍会输出，只是速度约为输入的 `65%`；这是安全
减速。若 `PolygonStop` 持续触发，检查点云时间戳、TF 和障碍物是否真正进入前方
停止区。只有 Nav2 action 非 `status=4`、`Failed to make progress` 或出现 Gazebo
非地面接触，才把该次任务判为失败。

### 速度

论文基线建议先保持 `0.18 m/s`。若要加速，按 `0.02 m/s` 递增，并同步检查：
局部 costmap 是否仍为 `10 Hz`、安全点云是否不过期、Gazebo contacts 是否为零。
真实 D435i 上不应直接沿用仿真速度。

## 修改和运行

```bash
cd /home/w417/RTAB-Map
$EDITOR src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml
sg docker -c './scripts/build.sh'
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true'
```

也可以给 launch 指定另一份 Nav2 文件，不改基线文件：

```bash
sg docker -c './scripts/launch_demo.sh nav2_params:=/workspaces/rtabmap_tb3_nav/config/my_nav2.yaml'
```

在线模式会从参数文件的全局插件列表临时移除 `static_layer`；定位模式才使用保存的
RTAB-Map 地图和 `StaticLayer`。

## 实验记录建议

每次导航使用：

```bash
sg docker -c './scripts/run_navigation_trial.sh --x 8.5 --y 0.0 --label A_to_B_trial01'
```

它会在 `results/A_to_B_trial01/` 写出 `trajectory.png`、`trajectory.csv` 和
`metrics.yaml`。图中的灰度背景来自最终 `/map`，红线是 map 坐标系中的实际轨迹；
指标同时记录 action 墙钟时间和 Gazebo 仿真时间。生成的结果默认不提交到 GitHub，
避免把二进制和机器相关日志混入源码；论文数据可另行归档。
