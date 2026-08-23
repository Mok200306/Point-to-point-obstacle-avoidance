# RTAB-Map TurtleBot3 RGB-D + Nav2

这是一个面向 Ubuntu 20.04 主机的 Docker 复现工程。容器内部固定为 Ubuntu
22.04 + ROS 2 Humble，使用 TurtleBot3 Waffle、Gazebo、RTAB-Map、Nav2 和一
个模拟 RGB-D 相机完成无真实 LiDAR 的室内导航。

当前默认场景是 `indoor_obstacle_course_large.world`：20 m x 14 m 封闭房间，
有 3 个错位横向障碍栏和 10 个箱体/柱体。绿色圆点是 A，红色圆点是 B：

```text
A = (-8.5, 0.0)   Gazebo 初始位置，车头朝 +X
B = ( 8.5, 0.0)   场景目标位置
```

当前面向新目标和新起终点的默认配置是 `0.45 m` 的
`adaptive_goal_line_045`：全局规划器是
`rtabmap_tb3_nav/GoalLineSmacPlanner`（继承 SmacPlanner2D），每次规划调用都使用当前
起点、当前目标和实时 costmap，关闭 large 场景固定世界坐标走廊。旧的
完整阶段演进、每个阶段的创新点、代码提交和历史复现入口见
[项目演进与阶段复现总览_2026-08-22.md](文档/00_项目总览/项目演进与阶段复现总览_2026-08-22.md)。正式结果目录和
清理规则见 [EXPERIMENT_ARCHIVE_INDEX.md](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。
全部阶段文档的目录见 [文档/README.md](文档/README.md)。
`fast_goalline_045_v4` 仍作为当前 large 场景的冻结 benchmark，三次独立回归平均
`81.07 s`、3/3 成功、0/3 非地面 contacts；需要复现该历史结果时显式指定 v4。
参数和三次回归见
[NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md](文档/04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md) 与
[FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md](文档/04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md)。历史 v3 参数和三次回归见
[NAVIGATION_OPTIMIZATION_2026-08-21.md](文档/04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21.md) 与
[FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-21.md](文档/04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-21.md)。
结果目录整理和各阶段保留规则见 [EXPERIMENT_ARCHIVE_INDEX.md](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。
旧 profile 仍可通过 `navigation_profile:=fast_north_045_v3`、`fast_goalline_045_v2`
或 `frozen_goal_line_045_v1` 显式回退，原生目标线 0.45 配置仍由历史提交保留。

本轮“目标改动后重新规划”的说明、双目标轨迹和参数快照见
[未知目标实时规划优化说明_2026-08-21.md](文档/05_自适应目标线_2026-08-21至22/未知目标实时规划优化说明_2026-08-21.md)、
[自适应目标线多目标实验记录_2026-08-21.md](文档/05_自适应目标线_2026-08-21至22/自适应目标线多目标实验记录_2026-08-21.md) 和
[自适应目标线参数记录_2026-08-21.md](文档/05_自适应目标线_2026-08-21至22/自适应目标线参数记录_2026-08-21.md)。

## 五点闭环阶段（2026-08-22）

在保持 `adaptive_goal_line_045` 不变的条件下，已完成
`M(-8.5,0) -> A(5,-3) -> B(5,6) -> C(-5,4) -> D(0,0) -> M(-8.5,0)`
的三次闭环回归。三次均为 `5/5` 段 `status=4`，总成功率 `3/3`，Gazebo
contacts 过滤地面后均无非地面接触；平均 wall 时间 `288.563 +/- 7.516 s`，平均
总轨迹 `51.919 +/- 0.877 m`。每次实验都保存左侧 Gazebo 真值和右侧 RViz 风格地图
的 `trajectory_comparison.png`，不会覆盖不同 run 的目录。

实验数据和图像见
[五点闭环导航实验记录_2026-08-22.md](文档/06_五点闭环与顺序验证_2026-08-22/五点闭环导航实验记录_2026-08-22.md)，
冻结参数和完整复现命令见
[五点闭环最终参数_2026-08-22.md](文档/06_五点闭环与顺序验证_2026-08-22/五点闭环最终参数_2026-08-22.md)，阶段边界和下一步见
[多目标导航阶段总结_2026-08-22.md](文档/06_五点闭环与顺序验证_2026-08-22/多目标导航阶段总结_2026-08-22.md)。

随后在不改变 `adaptive_goal_line_045` 的条件下完成了两种新点位顺序的三次验证：
`C -> A -> D -> B -> M -> C` 和 `B -> M -> A -> C -> D -> B`。两组均为 `3/3`
完整 run、`15/15` 分段成功、过滤地面后 `0/3` 非地面接触。三组五点闭环统一比较、
每次左右合成图和实验边界见
[点位顺序跨实验统一分析_2026-08-22.md](文档/06_五点闭环与顺序验证_2026-08-22/点位顺序跨实验统一分析_2026-08-22.md)；当前场景最终冻结和复现入口见
[当前场景算法最终验证总结_2026-08-22.md](文档/06_五点闭环与顺序验证_2026-08-22/当前场景算法最终验证总结_2026-08-22.md)。

当前 profile 的详细参数和命令见
[自适应目标线算法复现手册_2026-08-22.md](文档/05_自适应目标线_2026-08-21至22/自适应目标线算法复现手册_2026-08-22.md)，改变世界、
保存快照和恢复基线见
[跨场景仿真修改与恢复指南_2026-08-22.md](文档/07_跨场景与真实设备/跨场景仿真修改与恢复指南_2026-08-22.md)。

当前冻结基线和下一阶段跨场景验证计划见
[阶段 1：基线冻结与跨场景验证准备](文档/08_下一阶段实验归档_2026-08-22/阶段1_基线冻结与跨场景验证准备_2026-08-22.md)，
后续跨场景和真实设备阶段统一归档在
[下一阶段实验归档](文档/08_下一阶段实验归档_2026-08-22/README.md)。
当前场景 01 的启动、自动 contacts 识别和正式回归模板见
[阶段 2：跨场景验证准备](文档/08_下一阶段实验归档_2026-08-22/阶段2_跨场景验证准备_2026-08-22.md)。

场景 02 的正式跨场景结果、轨迹图和可复现参数见
[场景 02 四点闭环恢复增强报告](文档/08_下一阶段实验归档_2026-08-22/阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md)、
[场景 02 v5 参数冻结手册](文档/08_下一阶段实验归档_2026-08-22/阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md)。
当前 v5 在 `cross_scene_02` 上完成 `M -> N -> X -> Y -> M` 三次独立闭环，
`3/3` run、`12/12` 分段成功、`0/3` 非地面 contacts；它是场景 02 的验收 profile，
不替换默认 `adaptive_goal_line_045`。

自适应配置保留 `inflation_radius=0.45 m` 和完整安全链路，只使用动态目标线软偏好；v4
则额外包含当前 large 场景的分段走廊先验。两者都保留速度、RPP 前视、软代价衰减和
各自 profile 中的受控代价覆盖；5 秒在线建图稳定期只用于正式回归的可重复启动，
不计入导航 wall 时间。完整证据见上面的 v4 报告；v2 是更早的速度对照。

## 1. 这次改动解决什么问题

### 在线建图和导航

默认启动方式已经是在线模式：RTAB-Map 持续接收 RGB-D 图像和深度图并更新
`/map`，Nav2 同时接收目标、计算路径、根据局部深度点云避障并持续重规划。
不再把 `explore_demo.py` 作为导航前置步骤；它现在只是可选的地图覆盖路线。

在线模式能解决“机器人边走边建图”，但不能让全局规划器知道相机尚未看到的
房间细节。当前配置使用固定 `24 m x 17 m` 的 global costmap：`map_padder.py` 将
RTAB-Map 不断增长的 `/map` 复制为固定 `/nav_map`，再由 `StaticLayer` 叠加实时
RGB-D obstacle layer。`allow_unknown:true` 和 `track_unknown_space:true` 允许在地图
尚未完整长出时发送目标；机器人前方遇到未知区域时，RTAB-Map 和 RGB-D 局部代价地图
会边走边更新。固定尺寸的 `/nav_map` 避免 Humble 因 `/map` 尺寸变化反复 resize；定位
模式（`online:=false localization:=true`）才让 `StaticLayer` 直接读取保存的 `/map`。
未知区域的安全性仍依赖前向深度观测。
如果目标被障碍物完全隔开，仍建议先发送附近目标或使用可选探索路线，这是
未知环境导航的正常边界。

### 更保守的避障链路

```text
RGB-D depth
  -> /camera/points (raw sensor stream)
              |
              v
       point_cloud_xyz
              |
              v
       /camera/cloud
          |          |
          v          v
   collision_monitor  obstacles_detection
   /cmd_vel_safe      /camera/obstacles
          |                  |
          v                  v
    Gazebo robot       Nav2 costmaps
                              |
                              v
              SmacPlanner2D + RPP /cmd_vel
```

本项目不订阅 `/scan`。新增的 `nav2_collision_monitor` 会在机器人前方建立
停止区和减速区；Gazebo Waffle 的速度入口已经被镜像构建脚本改成
`cmd_vel_safe`。碰撞监视器读取由 `point_cloud_xyz` 降采样得到的
`/camera/cloud`；原始 `/camera/points` 保留用于诊断，而计算量较大的
`/camera/obstacles` 只用于 Nav2 代价地图。这样避免了完整点云让控制循环掉频，
也避免安全层使用处理延迟过大的障碍点云。

主要安全调整包括：

- 使用约 `0.60 m x 0.48 m` 的矩形 footprint，并额外 padding `0.03 m`；
- RGB-D 障碍高度提高到 `1.5 m`，最大深度范围约 `3.8 m`；
- 局部 costmap 为 `6 m x 5 m`，更新频率 `10 Hz`；
- 全局/局部 inflation radius 当前为速度优先冻结配置的 `0.45 m`，
  `cost_scaling_factor=3.0`；它只改变障碍物外侧的软代价梯度，不会缩小车体 footprint；
  `0.30 m` 对当前约 `0.27 m` 内切半径的 Waffle 几乎没有可用梯度；
- 全局规划器使用目标线偏好的 `GoalLineSmacPlanner`，底层仍是 `SmacPlanner2D`，
  `cost_travel_multiplier=6.0`；已知自由栅格按偏离起终点线的二次距离增加软代价，
  障碍物和硬 footprint 约束不被覆盖；
- 局部控制器使用 Regulated Pure Pursuit（RPP），直线目标速度 `0.26 m/s`，前视距离
  `0.56--1.15 m`（默认 `0.75 m`），普通弯道不主动原地对齐（阈值 `1.20 rad`），根据曲率和 cost 调速，
  提前沿平滑路径弧形转弯，同时保留终点姿态对齐；
- 目标误差收紧为 XY `0.12 m`、yaw `0.15 rad`。
- 模拟相机默认降为 `320 x 240 @ 15 Hz`，为 RTAB-Map、Nav2 和安全层保留 CPU
  余量；这不是 RGB-D 原理上的限制，接入 D435i 时再按实际帧率调节。

窄路口停车问题现在从三处处理：固定 `/nav_map` 不让 `StaticLayer` 随 `/map` resize，稳定
行为树每 `2 s` 检查路径且有效路径约 `20 s` 才重算，RPP 使用更长前视点提前转弯，只有
大角度姿态误差才原地对齐。全局
`GoalLineSmacPlanner` 负责在 Smac 可行路径中偏好回到目标方向，RPP 负责连续跟踪；`PolygonSlow` 前方约
`1.05 m` 保留 `75%` 速度，`PolygonStop` 约 `0.38 m` 仍是最后一道硬停止保护。

当前 v3 只把 benchmark 场景中的固定北侧走廊从 `x∈[-7.2,-2.5]` 延长到
`x∈[-7.2,3.45]`，目标带中心为 `world_y=0.75 m`，用于稳定通过西侧和中央障碍；它是
场景先验，不应未经重新测量直接迁移到真实或新地图。

RViz 中的紫红色区域是 `inflation_radius` 产生的代价梯度，不等于同样大小的实体
墙，也不等于所有这些格子都禁止通行；真正的碰撞判定还会检查 footprint、
RPP 的前向碰撞预测和前方 `PolygonStop`。当前 `0.45 m` 是路径偏好，不是允许车体
贴近障碍的许可证；若只想让显示更易读，可暂时关闭
RViz 的 Global/Local Costmap 图层，不能据此判断实体障碍消失。

看到机器人在窄口短暂停顿时，先区分三种状态：

- `PolygonSlow` 或速度从正常值降到约 `75%`：安全层看到了前方点云，这是预期的减速，
  不是导航失败；
- `PolygonStop` 持续触发：安全层认为障碍已经进入硬停止区，应检查 `/camera/cloud`
  的频率、时间戳和 TF，不能直接缩小 footprint 来强行通过；
- Nav2 返回非 `4`、日志出现 `Failed to make progress` 或 `Aborting handle`：这才是
  本次任务失败，需要清理重复 launch 后重新检查 costmap、Smac 和 RPP 参数。

## 2. 构建和启动

### 最短可视化运行流程

下面两个命令分别在两个终端执行。当前登录会话如果没有刷新 Docker 组权限，
保留 `sg docker -c` 写法即可。

终端 1：先清理可能残留的旧仿真，再启动容器和 Gazebo + RViz2。第二条命令会保持
前台运行，不要关闭这个终端：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=adaptive_goal_line_045'
```

等待约 15--30 秒后，应该看到 Gazebo 的封闭障碍场景和 RViz2 窗口。终端 2：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

机器人从 A `(-8.5, 0.0)` 出发，目标为 B `(8.5, 0.0)`。Gazebo 中观察实体运动和
障碍绕行；RViz2 中观察 `/map`、全局/局部路径、代价地图和 RGB-D 障碍点云。
如果只想在 RViz2 中操作，也可以使用 `Nav2 Goal` 工具点击地图中的 B 点。

恢复原来路线方差较小的 v3 对照时，把启动命令中的 profile 改为：

```bash
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=fast_north_045_v3'
```

v4 的无 GUI 正式回归：

```bash
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5 --label manual/fast_goalline_045_v4_A_to_B --profile fast_goalline_045_v4'
```

结束本次仿真时，在终端 1 按 `Ctrl+C`，然后可选执行：

```bash
sg docker -c './scripts/stop.sh'
```

不要在仿真运行时再启动 `real_d435i_nav.launch.py`；两者默认使用相同的 ROS 域和
`/camera`、`/rtabmap`、Nav2 名称，会互相干扰。

```bash
cd /home/w417/RTAB-Map
chmod +x scripts/*.sh src/rtabmap_tb3_nav/scripts/*.py

# 重新构建镜像，包含 nav2_collision_monitor、低延迟安全点云和无 LiDAR 模型补丁
./scripts/build.sh
./scripts/start.sh
```

如果当前终端还没有重新登录 Docker 用户组，可以临时使用：

```bash
sg docker -c './scripts/build.sh'
sg docker -c './scripts/start.sh'
```

启动完整仿真（默认自适应目标 profile）：

```bash
./scripts/launch_demo.sh rviz:=true rtabmap_viz:=false navigation_profile:=adaptive_goal_line_045
```

冻结的 large 场景 v4 benchmark：

```bash
./scripts/launch_demo.sh rviz:=true rtabmap_viz:=false navigation_profile:=fast_goalline_045_v4
```

如果只验证导航链路，建议先关闭高开销的两个 GUI：

```bash
./scripts/launch_demo.sh gazebo_gui:=false rviz:=false rtabmap_viz:=false reset_db:=true
```

第一次启动建议等待 15--30 秒，直到 Gazebo、`/camera/depth/image_raw`、
RTAB-Map 和 Nav2 都出现。RViz 配置文件使用 `map` 固定坐标系、从上往下看，
显示 SLAM 地图、全局/局部 costmap、两级路径和 RGB-D 障碍点云；Gazebo 则
使用俯视相机显示整个大场景。

RViz 中看不到完整房间并不代表 Gazebo 没有场景：`/map` 只包含相机已经观察
到的区域，启动初期通常只是一小块未知地图。先让机器人移动或发送一个近处
目标，地图会在 `map` 坐标系中逐步长出来。显示异常时先确认左上角
`Fixed Frame` 是 `map`，并暂时关闭 `Global Costmap` 和 `Local Costmap` 的
显示，避免半透明代价地图遮住灰度 SLAM 地图。

打开 RTAB-Map 图形界面：

```bash
./scripts/launch_demo.sh rtabmap_viz:=true
```

回到旧的 12 m x 8 m 小场景：

```bash
./scripts/launch_demo.sh world:=obstacle_course x_pose:=-4.5
```

官方场景仍可用：

```bash
./scripts/launch_demo.sh world:=house
```

## 3. 直接在线 A -> B

默认 `online:=true localization:=false`，所以不需要先运行建图脚本。启动后
可以直接发送目标：

```bash
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0'
```

也可以直接在 RViz 使用 `Nav2 Goal` 工具点击 B 点。脚本会打印剩余距离和
最终 action 状态，Nav2 成功状态为 `4`。

坐标说明：上面坐标适用于当前工程的 map 对齐方式。如果 RViz 中 RTAB-Map
的 `map` 原点与 Gazebo 世界原点不同，以 RViz 中的地图坐标为准；可以先在
机器人附近发送目标，确认 `map -> odom` 稳定后，再发送远处 B 点。

如果想显式写出在线模式：

```bash
./scripts/launch_demo.sh online:=true localization:=false reset_db:=true
```

`online:=false` 只允许和已有数据库的 `localization:=true` 一起使用：

```bash
./scripts/launch_demo.sh online:=false localization:=true reset_db:=false
```

## 4. 可选的地图覆盖路线

在线 A -> B 不要求预先绕场建图。如果想先让 RTAB-Map 覆盖墙和障碍物的两侧，
仍可以在完整仿真运行期间打开第二个终端执行：

```bash
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 run rtabmap_tb3_nav explore_demo.py'
```

大场景的路线是：

```text
(-8.5, 4.8) -> (8.5, 4.8) -> (8.5, -4.8)
-> (-8.5, -4.8) -> (-8.5, 0.0)
```

这条路线是“可选地图覆盖”，不是 Nav2 启动的前置依赖。

## 5. 运行检查

进入容器：

```bash
./scripts/shell.sh
```

检查 RGB-D、地图、代价地图和无 LiDAR 状态：

```bash
ros2 topic list | grep -E 'camera|rtabmap|costmap|cmd_vel|scan'
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/points
ros2 topic echo /camera/obstacles --once
ros2 topic echo /map --once
ros2 topic info /scan
ros2 topic info /cmd_vel_safe
```

预期结果：

- `/camera/obstacles` 有 `PointCloud2` 发布者；
- `/camera/points` 有 Gazebo 相机发布者；`/camera/cloud` 有降采样点云发布者，
  且是 collision monitor 的输入；
- `/map`、`/global_costmap/costmap` 和 `/local_costmap/costmap` 有数据；
- `/scan` 没有 TurtleBot3 LDS 发布者；
- `/cmd_vel` 的输入来自 Nav2 velocity smoother，`/cmd_vel_safe` 的输出来自
  collision monitor，并被 Gazebo 机器人订阅。

查看安全层是否因为数据过期而忽略点云：

```bash
ros2 node info /collision_monitor
ros2 param get /collision_monitor obstacles.topic
ros2 topic echo /polygon_stop --once
```

正常情况下，`obstacles.topic` 应为 `/camera/cloud`。如果日志持续出现
`Latest source and current collision monitor node timestamps differ`，先关闭
Gazebo/RViz GUI，再检查主机负载和 `/camera/points` 频率；不要直接把
`source_timeout` 无限调大，否则会把旧障碍物当成当前障碍物。若控制器掉频，
优先降低点云负载或关闭 GUI，不要把完整 `/camera/points` 直接接给安全层。

如果机器人被安全层停住，可以检查两个可视化多边形：

```bash
ros2 topic echo /polygon_stop --once
ros2 topic echo /polygon_slow --once
```

## 6. 数据库和定位模式

RTAB-Map 数据库位于宿主机：

```text
/home/w417/RTAB-Map/.ros/rtabmap.db
```

默认 `reset_db:=true`，映射启动时用新的数据库。保留数据库并切换定位：

```bash
./scripts/launch_demo.sh online:=false localization:=true reset_db:=false
```

这里的“默认”只指仿真 `demo.launch.py`。真实 D435i 启动文件的
`reset_db` 默认是 `false`，不会因为启动相机而删除已有数据库。

如果想重新建图，停止仿真后使用默认 `reset_db:=true`：

```bash
./scripts/stop.sh
./scripts/start.sh
./scripts/launch_demo.sh online:=true localization:=false reset_db:=true
```

## 7. 调参入口

完整参数表和调参顺序见 [PARAMETERS.md](文档/01_基础环境与问题分析/PARAMETERS.md)，主要参数在
[nav2_rgbd_params.yaml](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)：

- 想更安全：先降低 `max_vel_x`，再增大 `inflation_radius` 和 footprint；
- 想通过更窄的通道：只小幅减小 footprint/inflation，必须保持真实车体有余量；
- 障碍物漏检：检查 `/camera/obstacles`，提高 `max_obstacle_height` 或降低
  `obstacle_min_range`；
- 安全层漏检或持续停住：检查 `/camera/cloud` 和时间戳，确认其 TF 能变换到
  `base_footprint`，然后再调整 `collision_monitor_rgbd_params.yaml` 中的
  `source_timeout`、`PolygonStop` 和高度范围；
- 障碍物已经看见但仍靠近：先提高 Smac 的 `cost_travel_multiplier`，或降低
  `cost_scaling_factor` 让较远位置保留代价；若是转弯太晚，再增大 RPP 的
  `lookahead_dist`，不要先缩小 footprint；
- 目标附近不够精确：调整 `xy_goal_tolerance` 和 `yaw_goal_tolerance`，同时
  保持足够的进场空间。
- 机器人在窄口停住：先检查是否同时运行了两套 launch；执行
  `sg docker -c './scripts/stop.sh'` 后只启动一套，再检查
  `ros2 node list | sort` 中是否只有一个 `/collision_monitor`、`/camera_cloud` 和
  `/camera_obstacles`。如果日志是 `Robot to slowdown`，这是安全层看到近距离障碍
  后的预期减速；如果是 `Robot to stop`，先检查 `/camera/cloud` 的 TF 和时间戳。
  如果日志出现 `StaticLayer: Resizing`，说明在线启动仍加载了旧配置；如果 RPP 长时间
  输出零速度，再检查路径是否有效和 `/cmd_vel_safe` 是否被安全层拦截。不要直接无限
  增大 `source_timeout`，也不要为了“挤过”通道继续缩小真实机器人的 footprint。

碰撞安全区在 [collision_monitor_rgbd_params.yaml](src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml)，
场景在 [indoor_obstacle_course_large.world](src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world)，
启动链路在 [demo.launch.py](src/rtabmap_tb3_nav/launch/demo.launch.py)。

### 轨迹图和导航耗时

需要记录一次完整实验时，使用下面的脚本代替只发送 action 的 `send_goal.py`：

```bash
sg docker -c './scripts/run_navigation_trial.sh --x 8.5 --y 0.0 --yaw 0.0 --label A_to_B_trial01'
sg docker -c './scripts/run_navigation_trial.sh --x -8.5 --y 0.0 --yaw 3.14159265 --label B_to_A_trial01'
```

每次会在 `results/<label>/` 生成 `trajectory.png`、`trajectory.csv` 和 `metrics.yaml`。
PNG 背景来自最终 `/map`，红线是 map 坐标系中的实际 odom 轨迹；YAML 同时记录
action 墙钟时间和 Gazebo 仿真时间。一般结果目录默认被 `.gitignore` 排除，避免日志污染源码；
当前归档只保留三个阶段各三次正式结果；完整保留规则见
[EXPERIMENT_ARCHIVE_INDEX.md](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。双视图、参数快照和 contacts
证据见对应实验目录，汇总会写入 `PROJECT_PROGRESS.md`。

修改 ROS 文件后：

```bash
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   colcon build --symlink-install && source install/setup.bash'
```

只有修改了 `scripts/patch_turtlebot3_rgbd.sh` 或 `Dockerfile`，才需要重新构建
Docker 镜像；因为相机分辨率、帧率、LDS 删除和 `cmd_vel_safe` remap 都写入了
镜像内的 TurtleBot3 SDF：

```bash
./scripts/stop.sh
./scripts/build.sh
./scripts/start.sh
```

### 历史 DWB 对照

下面是旧版 NavFn + DWB 的历史对照，不代表当前 Smac + RPP 配置：

| 方向 | Nav2 | action 墙钟/仿真时间 | 末端 XY 误差 | 轨迹样本 | contacts（过滤地面） |
| --- | ---: | ---: | ---: | ---: | --- |
| A -> B | `4` | `182.87 s` | `0.108 m` | `900` | `464223`，`(none)` |
| B -> A | `4` | `161.07 s` | `0.111 m` | `723` | `424417`，`(none)` |

这组早期轨迹属于历史 NavFn + DWB 对照，原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md) 清理；B 起点的接触
回归曾使用干净重启并从对应端点开始；`scripts/regression_leg.sh` 会先等待
`/controller_server` 和 `/planner_server` 都处于 `active [3]`，再开始 300 s 的
Gazebo contacts 监听并发送目标。这里的 `(none)` 指过滤地面后没有机器人与墙、栏杆、
箱体或柱体的接触对。

这组历史结果可以作为论文中的“仿真 RGB-D 在线建图 + Nav2 DWB 对照组”，但需要同时报告
场景、速度、footprint、膨胀层、目标容差和无障碍接触判据。它不是“真实 D435i 基线”，
也不代表任意未知布局都能零碰撞；当前单次耗时仍包含在线建图和局部重规划带来的低速，
后续应增加多次重复实验并统计均值、标准差、成功率和最小障碍距离。

旧版窄口调参后，Gazebo + RViz2 开启时的 A -> B 返回 Nav2 状态 `4`，末端距离约
`0.12 m`；约 `307587` 条 Gazebo contacts 记录过滤地面后没有机器人与障碍物接触。
干净重启并从 B 出发的 B -> A 也返回状态 `4`，末端距离约 `0.10 m`；约 `350431`
条 contacts 记录同样没有非地面障碍接触。两次验证时 `/controller_server`、
`/planner_server` 和 `/collision_monitor` 都是 `active [3]`。

随后使用旧 DWB 的 `vtheta_samples=7` 做了无 GUI 双向回归：A -> B 返回 status `4`、末端距离
约 `0.10 m`，约 `346295` 条 contacts 过滤后无障碍接触；B -> A 返回 status `4`、
末端距离约 `0.12 m`，约 `373473` 条 contacts 过滤后无障碍接触。两次最新控制器日志
均没有 `Failed to make progress` 或 `Aborting handle`；窄口附近出现的短暂停顿对应
安全层 slowdown 和 RGB-D 障碍层更新后的局部路径重规划，不是碰撞。

旧版固定地图过渡方案曾临时移除 `StaticLayer`，这段记录保留作历史排查依据；当前方案
已经改为 `map_padder.py -> /nav_map -> StaticLayer`；下面的 clearance-first 数值是
历史 0.55 对照，当前连续走廊优化结果见 [优化文档](文档/04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21.md)。

如果 GUI 测试时出现 `Failed to change state for node: collision_monitor`，通常是旧
launch 没有完全退出而产生了重复节点，不是新的避障参数本身失败。先执行
`sg docker -c './scripts/stop.sh'`，确认 `ros2 node list | sort` 中每个关键节点只
出现一次，再重新启动一套仿真。

### 历史 Smac + RPP clearance-first 回归（0.55）

| 方向 | Nav2 status | 墙钟/仿真时间 | trial 最后采样 XY 误差 | contacts（过滤地面） |
| --- | ---: | ---: | ---: | --- |
| A -> B | `4` | `114.52 s` | `0.152 m` | `280327`，`(none)` |
| B -> A | `4` | `110.82 s` | `0.208 m` | `285301`，`(none)` |

这组 0.55 m 轨迹的统计和 contacts 结论仍在本文；原始目录已按
[EXPERIMENT_ARCHIVE_INDEX.md](文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md) 清理，可从整理前提交恢复。
`(none)` 表示过滤 `ground_plane` 后没有 `waffle` 与墙、barrier、crate 或 pillar 的接触对。

`navigation_trial.py` 的末端误差是 action result 前后收到的最后一条 TF/odom 轨迹采样，
不是 Nav2 内部 goal checker 的判定值；本轮以 Nav2 `status=4` 作为成功标准。论文中应
同时报告 action status、配置的 goal tolerance 和独立末端停稳误差，不能把这两个采样值
简单写成“没有到达目标”。

### 当前目标线优化回归（0.45）

目标线候选三次 A -> B 均为 `status=4`，平均墙钟 `113.63 s`，平均 Gazebo 轨迹
`17.977 m`，contacts 过滤地面后均为 none。详细参数、每次轨迹双视图和与原生 0.45
基线的比较见 [NAVIGATION_OPTIMIZATION_2026-08-21.md](文档/04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21.md)。

## 8. 真实 Intel RealSense D435i

软件启动链路已经准备好，但当前机器还没有接入实际 D435i 和真实底盘，因此本节
是可执行的迁移配置，不是硬件已经通过的回归结果。启动文件为
[real_d435i_nav.launch.py](src/rtabmap_tb3_nav/launch/real_d435i_nav.launch.py)，
参数文件为 [real_d435i_camera.yaml](src/rtabmap_tb3_nav/config/real_d435i_camera.yaml)。

真实机器人在启动本项目之前必须已经提供：

- `odom -> base_link` TF，以及 `base_link -> base_footprint`（如果底盘驱动使用
  `base_footprint`）；
- 真实底盘驱动订阅 `/cmd_vel_safe`，而不是绕过安全层直接订阅 `/cmd_vel`；
- 轮速里程计和底盘控制已经能够独立低速前进、停止和原地转向。

先确认主机能看到设备，再重启容器让 USB 映射生效：

```bash
lsusb | grep -i 'Intel\|RealSense' || true
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "ls -l /dev/bus/usb; source /opt/ros/humble/setup.bash; ros2 pkg executables realsense2_camera"'
```

第一次真实建图建议保持低速，先不发送远距离目标，只用底盘遥控让相机观察环境：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav real_d435i_nav.launch.py camera_serial:=<D435I_SERIAL> camera_x:=0.18 camera_y:=0.0 camera_z:=0.28 camera_pitch:=0.0 online:=true localization:=false reset_db:=false"'
```

真实启动默认使用独立数据库 `~/.ros/rtabmap_d435i.db`，按当前 Compose 配置对应
宿主机的 `/home/w417/RTAB-Map/.ros/rtabmap_d435i.db`，不会混用仿真数据库
`.ros/rtabmap.db`。如果确认要从头建立一张新图，先备份这个真实数据库，再显式使用
`reset_db:=true`；也可以通过 `database_path:=...` 指定其他路径。

`camera_x/y/z` 是从 `base_link` 原点指向 `camera_link` 原点的米制坐标：X 向前、
Y 向左、Z 向上。`camera_roll/pitch/yaw` 是安装姿态（弧度）；相机镜头向下安装时
通常需要正的 ROS pitch，但必须用 TF 实测和 RViz 验证。不要直接沿用示例值，尤其
要测量相机高度和俯角。检查 TF：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && ros2 run tf2_ros tf2_echo base_link camera_link"'
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && ros2 run tf2_ros tf2_echo odom base_link"'
```

确认设备和 TF 后，再检查真实 RGB-D 链路：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 topic hz /camera/color/image_raw"'
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 topic hz /camera/aligned_depth_to_color/image_raw"'
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 topic info /camera/cloud && ros2 topic info /camera/obstacles && ros2 topic info /cmd_vel_safe"'
```

在线建图阶段可以边走边导航。确认近距离目标和传感器正常后，再发送目标：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 1.0 --y 0.0 --yaw 0.0"'
```

建图完成后停止在线节点，再用同一固定安装和同一数据库进入定位模式：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav real_d435i_nav.launch.py camera_serial:=<D435I_SERIAL> online:=false localization:=true reset_db:=false database_path:=~/.ros/rtabmap_d435i.db"'
```

手持 D435i 只适合快速采集或验证视觉链路，不适合直接作为正式导航地图。正式流程
应当是：

```text
固定安装 D435i -> 实测相机 TF -> 低速在线建图 -> 近距离导航验证
-> 保存 rtabmap_d435i.db -> 保持相机和 footprint 不变 -> localization + Nav2
```

真实 RGB-D 还需要单独检查黑色/反光物体、阳光、遮挡、USB 带宽和深度掉帧；它不具备
LiDAR 的全向视野，因此在正式迁移前仍需做窄通道和侧后方障碍测试。

## 9. 推荐迁移顺序

建议顺序是：

```text
仿真在线建图导航
  -> 低速 A -> B / B -> A 无碰撞回归
  -> 窄通道、遮挡、相机掉帧测试
  -> 将 D435i 固定到真实机器人
  -> realsense2_camera 发布 RGB/depth/camera_info/TF
  -> RTAB-Map 在线建图
  -> 保存数据库
  -> localization 模式 + Nav2
```

本项目当前已经完成仿真在线建图、在线导航和双向物理接触回归。下一阶段应按以下
顺序接入真实平台：固定安装 D435i，测量并验证 TF，低速检查 RGB-D 和
`/cmd_vel_safe`，再在线建图和近距离导航，最后保存数据库切换到定位模式。手持
D435i 只适合快速采集或验证视觉链路，不适合作为正式导航地图的最终数据源。

## 10. 常见问题

### Docker permission denied

执行 `newgrp docker` 或注销重新登录，再运行 `docker info`。当前用户应在：

```bash
getent group docker
```

### Gazebo/RViz 无窗口

确认 `DISPLAY`，再执行：

```bash
xhost +si:localuser:$(id -un)
DISPLAY=:1 ./scripts/start.sh
```

### 机器人撞到障碍物

先确认 `/camera/obstacles` 有数据、`/cmd_vel_safe` 有 collision monitor 发布者，
并确认新镜像已经重建。将速度降到 `0.10--0.14 m/s` 后再测试；若仍撞，先
扩大 `PolygonStop` 和 footprint，不要先缩小膨胀范围。

### 只有地图没有导航

检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

目标未进入 `map` 坐标系或尚未有可连接的已知区域时，先发机器人附近目标，
再逐步扩大目标距离。
