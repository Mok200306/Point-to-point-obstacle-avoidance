# 项目进度总结

更新时间：2026-08-20（障碍距离优先调参、轨迹记录、双向计时与 contacts 回归）

## 当前目标

在 Ubuntu 20.04 + RTX 4090 主机上，用 Docker 运行 Ubuntu 22.04 + ROS 2
Humble，复现无真实 LiDAR 的 TurtleBot3 RGB-D + RTAB-Map + Nav2 室内 A -> B
点到点导航，并在多个静态障碍物之间安全绕行。

## 当前状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| Docker/ROS 2 Humble | 已完成 | `rtabmap-tb3:humble` 镜像和 Compose 已存在 |
| TurtleBot3 Waffle | 已完成 | Gazebo 模型删除 LDS，RGB-D 相机保留 |
| RTAB-Map RGB-D SLAM | 已完成 | 在线发布 `/map` 和 `map -> odom` |
| Nav2 点到点导航 | 已完成 | `NavigateToPose` action 链路已经验证过 |
| RGB-D 局部避障 | 已完成 | `/camera/obstacles` -> voxel costmap |
| 在线建图在线导航 | 本次实现 | 默认启动即同时运行，不再强制预建图 |
| 碰撞安全层 | 已完成双向回归 | `nav2_collision_monitor` 读取降采样 `/camera/cloud`，输出 `/cmd_vel_safe`；A -> B / B -> A 物理接触过滤均未发现障碍碰撞 |
| 贴边路径优化 | 已完成一轮基线调参 | DWB 提高障碍代价、降低路径黏性，inflation 改为 `0.50 m` 平滑梯度；需在更多布局继续统计 |
| 轨迹与计时记录 | 已完成 | `navigation_trial.py` 自动输出 PNG、CSV、YAML，记录墙钟和 Gazebo 仿真时间 |
| 大型障碍场景 | 本次实现 | 20 m x 14 m，错位障碍栏 + 10 个箱体/柱体 |
| 真实 D435i | 软件启动链路已准备，尚未接入实际硬件 | 已加入 RealSense 驱动、USB 映射、相机参数和真实启动文件；仍需真实底盘、TF 与 D435i 实测 |

## 当前能力结论

当前项目已经实现的是一个“仿真环境中的完整 RGB-D 在线建图、点到点导航和障碍物
绕行闭环”，不是只完成了单独的建图或单独的路径规划。已验证的范围是当前
`indoor_obstacle_course_large.world` 静态障碍场景、TurtleBot3 Waffle 模型和
仿真 RGB-D 相机：

- 不需要先运行独立建图步骤；`demo.launch.py` 默认 `online:=true`，RTAB-Map 和
  Nav2 同时运行。
- A -> B、B -> A 都曾返回 Nav2 状态 `4`，目标误差约 `0.10--0.12 m`。
- A -> B / B -> A 的 Gazebo contacts 过滤均未发现机器人与墙、栏杆、箱体或柱体
  的非地面接触。
- 这是在一个设计好的可通行障碍场景中的功能闭环证明，不代表任意未知房间、任意
  狭窄通道或真实机器人已经达到工程级零碰撞。

因此，准确的结论是：点到点避障导航在当前仿真验证场景中已经实现；泛化到未知布局、
复杂遮挡、窄通道和真实 D435i 的部分仍属于下一阶段。

## 2026-08-20 本轮优化结论

贴边的直接原因是旧参数中路径跟随代价明显高于障碍代价，DWB 会追随一条靠近
inflation 边缘的全局路径。当前基线已调整为：

- 局部/全局 `inflation_radius=0.50 m`、`cost_scaling_factor=3.0`；
- `BaseObstacle.scale=2.0`、`ObstacleFootprint.scale=2.0`；
- `PathAlign=12.0`、`PathDist=16.0`，允许局部轨迹离开全局路径换取 clearance；
- `PolygonSlow` 前方约 `1.05 m`、速度比例 `0.65`；硬 `PolygonStop` 仍约 `0.38 m`；
- `vtheta_samples=7`、最大线速度 `0.18 m/s` 保持不变。

这不是简单把膨胀层缩小，而是让“不可碰撞”与“远离障碍”分开：footprint 和
`ObstacleFootprint` 负责硬安全边界，膨胀梯度和 `BaseObstacle` 负责选择更宽松的
轨迹。详细可调参数见 [PARAMETERS.md](PARAMETERS.md)。

在线 RGB-D 首次观察场景时仍可能出现几秒到几十秒的低速或重新规划，这属于当前基线
的性能特征，不应在论文中写成“始终流畅无停顿”。

## 仿真如何运行

要同时看到 Gazebo 和 RViz2，使用以下顺序：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true'
```

保持第二条命令的终端打开，再在另一个终端发送 A -> B 目标：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

停止时按 `Ctrl+C` 结束 launch，需要时再执行 `sg docker -c './scripts/stop.sh'`。
默认地图会重新在线生成；仿真数据库是 `.ros/rtabmap.db`，真实 D435i 默认使用
独立的 `.ros/rtabmap_d435i.db`。

## 仿真与固定 D435i 的关系

固定安装 D435i 当前不会影响仿真结果。仿真入口使用 Gazebo 内置 RGB-D 相机和
TurtleBot3 模型 TF；真实入口只有在显式运行 `real_d435i_nav.launch.py` 时才会
启动 USB 相机、真实相机静态 TF 和真实数据库。RealSense 软件包和 `/dev/bus/usb`
映射只是镜像能力扩展，不会改变 `demo.launch.py` 的 Gazebo 传感器。

需要注意：不能让仿真和真实入口同时运行在同一个 ROS 域中，因为它们会同时创建
`/camera`、`/rtabmap`、Nav2 和速度话题。仿真结果可以继续复现和调参，但它证明的
是仿真传感器、仿真里程计和 Gazebo 动力学下的能力；真实设备还要重新验证深度噪声、
TF、底盘误差、USB 带宽、遮挡和掉帧。

## 当前工具链路

```text
Gazebo RGB-D 相机
  -> /camera/image_raw + /camera/depth/image_raw + /camera/points
  -> RTAB-Map（结合 /odom 和 TF）
  -> /map + map -> odom

/camera/depth/image_raw
  -> point_cloud_xyz
  -> /camera/cloud
  -> collision_monitor
  -> /cmd_vel_safe
  -> Gazebo TurtleBot3 底盘

/camera/cloud
  -> obstacles_detection
  -> /camera/obstacles + /camera/ground
  -> Nav2 global/local costmap
  -> NavFn 全局规划 + DWB 局部控制
  -> /cmd_vel
  -> collision_monitor 安全过滤
  -> /cmd_vel_safe
```

`/scan` 没有发布者，因此当前项目不是 LiDAR 导航。RViz2 只是可视化和发送目标，
真正完成规划的是 RTAB-Map、Nav2 costmap、NavFn、DWB 和 collision monitor 的组合。

## 下一步工作划分

1. 继续仿真测试更窄通道、遮挡、点云掉帧和不同起终点；如果新场景出现真实接近，
   应先调大 footprint、膨胀层或前方停止区，不能通过缩小安全参数来掩盖碰撞风险。
2. 固定安装 D435i，测量 `camera_x/y/z/pitch`，确认真实底盘提供
   `odom -> base_link`，并让底盘订阅 `/cmd_vel_safe`。
3. 先用真实 D435i 在线建图和近距离目标验证，再保存独立数据库，最后切换
   `localization:=true` 做正式导航。

## 本次回归结果

运行容器：`rtabmap_tb3_humble`。由于当前登录会话的 Docker group 权限尚未刷新，
测试使用 `sg docker -c '...'` 执行。

### 本轮双向实验

| 方向 | Nav2 状态 | action 墙钟时间 | 仿真时间 | 末端 XY 误差 | 轨迹样本 | contacts / 障碍接触 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A -> B | `4` | `182.87 s` | `182.87 s` | `0.108 m` | `900` | `464223` / `(none)` |
| B -> A | `4` | `161.07 s` | `161.07 s` | `0.111 m` | `723` | `424417` / `(none)` |

轨迹证据位于本机 `results/A_to_B_clearance/` 和 `results/B_to_A_clearance/`，各目录
包含 `trajectory.png`、`trajectory.csv` 和 `metrics.yaml`。A→B 及 B→A 的 contacts
都过滤了 `ground_plane`；最新完整监听分别记录 464,223 和 424,417 条 contact
消息，因此表中的 `(none)` 指没有 `waffle` 与墙、栏杆、箱体或柱体的接触对。

这两次结果可作为当前论文的仿真 RGB-D 在线建图 + Nav2 DWB 基准模型：成功率在这两次
为 `2/2`，末端误差满足 `XY<=0.12 m`，但样本量太小，不能替代后续多次重复统计。
应继续记录均值、标准差、成功率、总路径长度、最小障碍距离和重规划次数。

本轮 contacts 监听窗口已从 `180 s` 扩大到 `300 s`，覆盖在线启动、建图和目标到达的
完整回归时段；后续回归不要把监听超时前的部分结果当成完整物理证据。

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| A -> B | 通过 | `NavigateToPose finished with status 4`，末端距离约 `0.12 m` |
| B -> A | 通过 | `NavigateToPose finished with status 4`，末端距离约 `0.10--0.12 m` |
| A -> B（本次镜像重建后） | 通过 | `NavigateToPose finished with status 4`，脚本退出码 `0`，末端距离约 `0.10 m`；约 `354653` 条 contacts 过滤后无障碍接触 |
| A -> B（Gazebo GUI + RViz2，窄口调参后） | 通过 | `NavigateToPose finished with status 4`，末端距离约 `0.12 m`；约 `307587` 条 contacts 过滤后无障碍接触 |
| B -> A（干净重启，窄口调参后） | 通过 | `NavigateToPose finished with status 4`，末端距离约 `0.10 m`；约 `350431` 条 contacts 过滤后无障碍接触 |
| A -> B（vtheta=7，无 GUI） | 通过 | status `4`，末端距离约 `0.10 m`；约 `346295` 条 contacts 过滤后无障碍接触 |
| B -> A（vtheta=7，无 GUI） | 通过 | status `4`，末端距离约 `0.12 m`；约 `373473` 条 contacts 过滤后无障碍接触 |
| A -> B（旧在线参数，仅 enabled=false） | 通过但有 resize | status `4`，末端距离约 `0.10--0.12 m`；约 `351460` 条 contacts 过滤后无障碍接触；日志仍有 StaticLayer resize |
| B -> A（旧在线参数，仅 enabled=false） | 通过但有 resize | status `4`，末端距离约 `0.12 m`；约 `434034` 条 contacts 过滤后无障碍接触；日志仍有 StaticLayer resize |
| B -> A（修复后移除 StaticLayer） | 通过 | status `4`，末端距离约 `0.12 m`；约 `342705` 条 contacts 过滤后无障碍接触 |
| A -> B（修复后移除 StaticLayer） | 通过 | status `4`，末端距离约 `0.12 m`；约 `358524` 条 contacts 过滤后无障碍接触 |
| A -> B（最终完整 contacts 监听） | 通过 | status `4`，末端约 `0.10--0.12 m`；`464223` 条 contacts 过滤后无障碍接触 |
| B -> A（最终完整 contacts 监听） | 通过 | status `4`，末端约 `0.12 m`；`424417` 条 contacts 过滤后无障碍接触 |
| RGB-D `/camera/cloud` | 通过 | 实测约 `12.0--12.4 Hz` |
| Nav2 `/camera/obstacles` | 通过 | 实测约 `12.0--12.4 Hz`，PointCloud2 有发布者和订阅者 |
| Nav2 生命周期 | 通过 | `/controller_server`、`/planner_server` 均为 `active [3]` |
| 无 LiDAR | 通过 | `/scan` 没有发布者 |
| 安全层响应 | 通过 | 最新日志出现 slowdown，历史回归出现 stop；说明点云进入安全层 |
| 点云时间戳 | 通过 | 本轮 collision monitor 日志没有过期/时间戳差异警告 |
| Gazebo 障碍接触 | A -> B / B -> A 通过 | 最终完整监听 A -> B `464223` 条、B -> A `424417` 条 contacts 记录中过滤 `ground_plane` 后，均没有 `waffle` 与 `wall/barrier/crate/pillar` 的接触对 |

这次回归证明当前 A/B 路线能够在线建图、在线规划、在线避障并到达目标，且两个
方向都通过了 Gazebo 物理接触过滤。它不等于任意障碍布局都已经达到零碰撞：RGB-D
仍有视野、遮挡、反光和掉帧限制。

此前失败样本中曾记录若干次 `Failed to make progress` 和少量 deadline 提示。复核
`/cmd_vel` 后确认主要死锁点是旧版 `vtheta_samples=20` 产生约 `+/-0.0395 rad/s`
的微小角速度，机器人在反向目标或障碍拐角处左右摆动；`min_speed_theta` 本身没有
改变该采样集合。当前改为 `vtheta_samples=7`，同时使用 `inflation_radius=0.50 m`
和贴近车体的 `PolygonStop=0.38 m`。最新 A -> B / B -> A 控制器日志均没有
`Failed to make progress` 或 `Aborting handle`。

当前回归中仍可能看到几秒至十几秒的速度下降或距离反馈暂时不变：碰撞监视器日志显示这是
`PolygonSlow` 触发后的安全减速，RGB-D 障碍层更新时局部路径也可能被重新计算。它们
不是永久停车或任务失败；若出现 `PolygonStop` 持续触发、进度错误或物理接触，仍应
按失败处理并重新调参。

## 本次代码调整

### 本轮镜像与工作区验证

- Docker 镜像 `rtabmap-tb3:humble` 已重新构建完成。
- 镜像内 `ros-humble-realsense2-camera` 已安装，`realsense2_camera_node` 可发现。
- 容器已按新 Compose 配置重启，`/dev/bus/usb` 已映射，运行用户具备 `video` 和
  `plugdev` 组权限。
- 工作区 `colcon build --symlink-install` 通过。
- `real_d435i_nav.launch.py --show-args` 通过，默认 `use_sim_time=false`、
  `reset_db=false`，默认数据库为独立的 `~/.ros/rtabmap_d435i.db`。
- 当前没有接入实际 D435i，所以不能把设备枚举、真实图像、真实底盘运动写成已通过。

### 真实 D435i 配置

新增：

- `src/rtabmap_tb3_nav/config/real_d435i_camera.yaml`
- `src/rtabmap_tb3_nav/launch/real_d435i_nav.launch.py`

真实启动文件包含 RealSense RGB/depth 对齐、近似同步、IMU、RTAB-Map 在线建图或定位、
`/camera/cloud`、`/camera/obstacles`、Nav2、collision monitor 和相机静态 TF。
真实底盘必须自行发布 `odom -> base_link` 以及需要的 `base_link -> base_footprint`，
并订阅 `/cmd_vel_safe`。

### 启动链路

`src/rtabmap_tb3_nav/launch/demo.launch.py` 现在：

- 默认场景改为 `obstacle_course_large`；
- 默认初始位置改为 `(-8.5, 0.0)`；
- 默认启动 `online:=true`；
- RTAB-Map 和 Nav2 同时启动；
- 增加 `nav2_collision_monitor`；
- RTAB-Map 深度建图范围从 3 m 提高到 4 m，障碍高度上限为 1.5 m。
- 在线模式从全局 `plugins` 列表移除 `static_layer`，使用固定 `40 m x 30 m` rolling
  costmap；定位模式保留 `StaticLayer` 读取已保存地图，避免 Humble 在线 resize。

`online:=false` 只能与 `localization:=true` 配合，用于已有数据库定位。

### Nav2 参数

`src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml` 现在：

- 全局 costmap 为 40 m x 30 m rolling window；
- `allow_unknown: true`，允许地图增长期间 NavFn 规划；
- 局部 costmap 为 6 m x 5 m，10 Hz 更新；
- DWB controller 频率调整为 `10 Hz`，给每次轨迹评价留出完整计算预算，减少
  20 Hz deadline miss；
- 使用矩形 footprint `0.60 m x 0.48 m`，padding `0.03 m`；
- inflation radius `0.50 m`、`cost_scaling_factor=3.0`；这是本仿真场景的折中值，不是固定的真实机器人安全值；
- 线速度 `0.18 m/s`，角速度 `0.75 rad/s`；
- DWB 同时使用 `BaseObstacle` 与 `ObstacleFootprint`，权重为 `2.0/2.0`，并降低
  `PathAlign/PathDist` 到 `12.0/16.0`；
- `SimpleProgressChecker` 使用 `0.20 m / 20 s`，允许机器人在拐角减速或反向目标的初始
  转向时继续
  获得进度；
- DWB 采样为 `vx=8, vy=1, vtheta=7`，`sim_time=1.25 s`，避免微小角速度左右摆动并减少 GUI 开启时的控制
  循环掉频；规划器期望频率为 `5 Hz`；
- 局部/全局障碍观测持续时间约 `0.4--0.5 s`。

### 碰撞监视器和实时性

`src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml`：

- 输入速度：`/cmd_vel`；
- 输出速度：`/cmd_vel_safe`；
- RGB-D 输入：使用 `point_cloud_xyz` 降采样后的 `/camera/cloud`；原始
  `/camera/points` 仅用于诊断，Nav2 costmap 继续使用 `/camera/obstacles`；
- 前方约 `0.38 m` stop polygon；硬停止区贴近车体，避免在拐角还没完成转向时提前
  锁死；
- 前方约 `1.05 m` slowdown polygon，速度降到 `65%`；
- `source_timeout` 收紧到 `0.5 s`，避免安全层继续使用过期点云；
- 不把完整原始点云直接交给 collision monitor，避免 20 Hz controller 因点数过多
  掉频；
- 模拟 RGB-D 相机降为 `320 x 240 @ 15 Hz`，并关闭 Gazebo 相机可视化；
- 增加 `gazebo_gui:=false`，无 GUI 回归时减少控制循环掉频；GUI 回归必须保证同一
  时间只有一套 launch，避免重复 `collision_monitor` 和 `camera_cloud`。

`scripts/patch_turtlebot3_rgbd.sh` 修改 Gazebo SDF 的 `command_topic`，并添加
`cmd_vel:=cmd_vel_safe` ROS remap。官方 URDF 只供 `robot_state_publisher` 发布
TF，并没有 Gazebo 驱动插件，所以不在 URDF 中伪造速度字段。重新构建镜像后才
会应用到 `/opt/ros` 模型。

## 当前大场景

文件：`src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world`

- 房间边界：`x=[-10,10]`，`y=[-7,7]`；
- 起点 A：`(-8.5, 0.0)`；
- 终点 B：`(8.5, 0.0)`；
- 西侧横栏：`x=-4.7`，开口在南/北侧；
- 中部横栏：`y=1.6`，开口在南/北侧；
- 东侧横栏：`x=5.3`，开口在南/北侧；
- 另有西侧、中部、东侧和南北侧的箱体/柱体。

设计意图是阻挡 A -> B 直线，同时保留大于约 1.8 m 的主要绕行通道。实际
可行宽度还要扣除 Waffle footprint、inflation 和 RGB-D 安全区。

## 推荐运行步骤

```bash
cd /home/w417/RTAB-Map
./scripts/build.sh
./scripts/start.sh
./scripts/launch_demo.sh rviz:=true rtabmap_viz:=false
```

性能/安全回归建议使用：

```bash
./scripts/launch_demo.sh gazebo_gui:=false rviz:=false rtabmap_viz:=false
```

等待传感器和 Nav2 出现后，直接运行：

```bash
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0'
```

检查：

```bash
./scripts/shell.sh
ros2 topic echo /map --once
ros2 topic echo /camera/obstacles --once
ros2 topic info /scan
ros2 topic info /cmd_vel_safe
ros2 lifecycle get /controller_server
```

可以运行可选覆盖路线：

```bash
ros2 run rtabmap_tb3_nav explore_demo.py
```

它不是在线导航的前置步骤，只用于让相机先观察完整场景。

## 进度判定标准

下一轮仿真回归需要记录：

| 指标 | 目标 |
| --- | --- |
| A -> B action | 返回状态 4 |
| B -> A action | 返回状态 4 |
| Gazebo 碰撞 | 0 次 |
| `/camera/cloud` | 降采样安全点云持续发布，且时间戳不过期 |
| `/camera/points` | 原始相机点云持续发布，用于诊断 |
| `/camera/obstacles` | 持续有 PointCloud2，供 costmap 使用 |
| `/cmd_vel_safe` | collision monitor 有输出 |
| 窄通道 | 不发生 footprint 与障碍物重叠 |
| 目标精度 | XY <= 0.12 m，yaw <= 0.15 rad |

窄口调参后已完成 GUI + RViz2 的 A -> B，以及干净重启后的 B -> A Gazebo contacts
过滤；旧方案仅设置 `StaticLayer.enabled=false`，日志仍显示地图 resize；最终方案已从
在线 `plugins` 列表移除该插件，并重新完成 A -> B / B -> A 双向回归。后续修改相机、
footprint、inflation 或速度后，仍必须重新执行双向回归，不能沿用旧结果。

## 调参顺序

1. 先确认 `/camera/cloud` 点云持续发布且 collision monitor 不持续报过期。
2. 再确认 `/camera/obstacles` 能被 costmap 看见。
3. 确认 `/cmd_vel` -> collision monitor -> `/cmd_vel_safe` 链路正确。
4. 将最大速度保持在 `0.10--0.18 m/s` 做无碰撞测试。
5. 若仍接近障碍，增大 `PolygonStop`、footprint 或 inflation。
6. 只有在通道通过率稳定后，才逐步提高速度或减小安全余量。

不要先缩小 footprint 或 inflation 来“挤过”通道；真实机器人会比仿真更受
深度噪声、时间同步、底盘误差和地面不平影响。

## 真实 D435i 迁移结论

推荐将 D435i 固定安装在机器人上，再进行正式建图。手持建图可以用于快速采集，
但会带来相机高度、安装姿态、TF 和 footprint 不一致的问题。真实阶段需要：

- `realsense2_camera` 发布 RGB、depth、CameraInfo 和 TF；
- 对齐后的深度图或 RGB-D 点云；
- RTAB-Map RGB-D 参数和真实相机 frame；
- 固定机器人 footprint；
- 先在线建图，再保存数据库，最后切换 localization + Nav2。

真实启动命令（默认使用独立的 D435i 数据库，不混用仿真 `.ros/rtabmap.db`）：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav real_d435i_nav.launch.py camera_serial:=<D435I_SERIAL> camera_x:=0.18 camera_y:=0.0 camera_z:=0.28 camera_pitch:=0.0 online:=true localization:=false reset_db:=false database_path:=~/.ros/rtabmap_d435i.db"'
```

需要根据实物重新测量 `camera_x`、`camera_y`、`camera_z` 和 `camera_pitch`，并用
`tf2_echo base_link camera_link` 验证。手持 D435i 仅用于快速采集或链路验证，正式
导航地图应由固定安装后的相机建立。

## 下一步

1. 已在新镜像上完成默认大场景的 GUI A -> B 和干净重启 B -> A 回归；RealSense
   依赖和 USB 映射改动没有影响仿真链路。
2. 继续记录更窄通道、遮挡和点云掉帧测试；目前不能宣称任意狭窄环境均达到工程级
   零碰撞，因为 RGB-D 仍受视野、遮挡和时延影响。
3. 接入固定安装的 D435i 与真实底盘，先验证 TF、RGB-D 频率和 `/cmd_vel_safe`，再低速
   在线建图，最后保存数据库切换 localization。

## 已知限制

- RGB-D 相机视野主要覆盖车头，不能像 360 度 LiDAR 一样观察侧后方；
- 未看到的未知区域不可能提前得到真实障碍布局；
- `allow_unknown:true` 允许规划穿过未知区，但安全性仍依赖实时深度更新；
- 真实 D435i 的深度反光、黑色物体、阳光和遮挡需要单独标定；
- Docker 当前终端若没有 docker group 权限，需要使用 `sg docker -c '...'`。
