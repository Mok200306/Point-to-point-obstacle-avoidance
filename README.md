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

## 1. 这次改动解决什么问题

### 在线建图和导航

默认启动方式已经是在线模式：RTAB-Map 持续接收 RGB-D 图像和深度图并更新
`/map`，Nav2 同时接收目标、计算路径、根据局部深度点云避障并持续重规划。
不再把 `explore_demo.py` 作为导航前置步骤；它现在只是可选的地图覆盖路线。

在线模式能解决“机器人边走边建图”，但不能让全局规划器知道相机尚未看到的
房间细节。当前配置使用 40 m x 30 m rolling global costmap、`allow_unknown:
true` 和 `track_unknown_space:true`，因此可以在地图还没有完全长出来时发送
目标；机器人前方遇到未知区域时，RTAB-Map 和 RGB-D 局部代价地图会边走边更新。
在线模式还会由 `demo.launch.py` 从全局 costmap 的 `plugins` 列表中移除
`StaticLayer`：在 Humble 中仅设置 `enabled=false` 仍可能让它订阅 `/map` 并触发
resize。这样 `/map` 每次扩展不会触发全局网格反复重建，NavFn 可以持续使用固定尺寸
的 rolling 网格；定位模式（`online:=false localization:=true`）则重新使用保存地图的
`StaticLayer`。
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
                         DWB /cmd_vel
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
- 局部 costmap 扩大到 `6 m x 5 m`，更新频率 `10 Hz`；DWB 控制频率为 `10 Hz`，
  与 RGB-D 点云和当前仿真 CPU 负载匹配；
- 全局/局部 inflation radius 为 `0.40 m`；这是当前仿真场景的安全折中值，真实机器人必须重新验证；
- 最大线速度降为 `0.18 m/s`，最大角速度降为 `0.75 rad/s`；
- DWB 同时启用 `BaseObstacle` 和 `ObstacleFootprint`；
- 目标误差收紧为 XY `0.12 m`、yaw `0.15 rad`。
- 模拟相机默认降为 `320 x 240 @ 15 Hz`，为 RTAB-Map、Nav2 和安全层保留 CPU
  余量；这不是 RGB-D 原理上的限制，接入 D435i 时再按实际帧率调节。

窄路口停车问题已经针对“膨胀层 + 硬停止区 + DWB 角速度采样”造成的停止死锁调整：
全局/局部 `inflation_radius` 为 `0.40 m`，`PolygonStop` 约 `0.38 m`，`PolygonSlow`
约 `0.95 m` 且保留 `50%` 速度；进度检查器为 `0.20 m / 20 s`，DWB 采样为
`8 x 1 x 7`、`sim_time=1.25 s`。`vtheta_samples=7` 让角速度样本包含约
`-0.75/-0.50/-0.25/0/0.25/0.50/0.75 rad/s`，避免旧版 `vtheta_samples=20`
选到约 `+/-0.0395 rad/s` 后左右摆动。这样机器人有机会在障碍拐角处先减速、转向
并重新规划，同时仍保留近距离硬停止保护。

RViz 中的紫红色区域是 `inflation_radius` 产生的代价梯度，不等于同样大小的实体
墙，也不等于所有这些格子都禁止通行；真正的碰撞判定还会检查 footprint、
`ObstacleFootprint` 和前方 `PolygonStop`。当前 `0.40 m` 是仿真场景的安全折中值，
优先保留它而不是为了让画面变窄直接降低安全余量。若只想让显示更易读，可暂时关闭
RViz 的 Global/Local Costmap 图层，不能据此判断实体障碍消失。

看到机器人在窄口短暂停顿时，先区分三种状态：

- `PolygonSlow` 或速度从正常值降到约 `50%`：安全层看到了前方点云，这是预期的减速，
  不是导航失败；
- `PolygonStop` 持续触发：安全层认为障碍已经进入硬停止区，应检查 `/camera/cloud`
  的频率、时间戳和 TF，不能直接缩小 footprint 来强行通过；
- Nav2 返回非 `4`、日志出现 `Failed to make progress` 或 `Aborting handle`：这才是
  本次任务失败，需要清理重复 launch 后重新检查 costmap 和 DWB 参数。

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
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true'
```

等待约 15--30 秒后，应该看到 Gazebo 的封闭障碍场景和 RViz2 窗口。终端 2：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0"'
```

机器人从 A `(-8.5, 0.0)` 出发，目标为 B `(8.5, 0.0)`。Gazebo 中观察实体运动和
障碍绕行；RViz2 中观察 `/map`、全局/局部路径、代价地图和 RGB-D 障碍点云。
如果只想在 RViz2 中操作，也可以使用 `Nav2 Goal` 工具点击地图中的 B 点。

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

启动完整仿真：

```bash
./scripts/launch_demo.sh rviz:=true rtabmap_viz:=false
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

主要参数在 [nav2_rgbd_params.yaml](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)：

- 想更安全：先降低 `max_vel_x`，再增大 `inflation_radius` 和 footprint；
- 想通过更窄的通道：只小幅减小 footprint/inflation，必须保持真实车体有余量；
- 障碍物漏检：检查 `/camera/obstacles`，提高 `max_obstacle_height` 或降低
  `obstacle_min_range`；
- 安全层漏检或持续停住：检查 `/camera/cloud` 和时间戳，确认其 TF 能变换到
  `base_footprint`，然后再调整 `collision_monitor_rgbd_params.yaml` 中的
  `source_timeout`、`PolygonStop` 和高度范围；
- 障碍物已经看见但仍靠近：提高 `BaseObstacle.scale`，或扩大 collision monitor
  的 `PolygonStop`；
- 目标附近不够精确：调整 `xy_goal_tolerance` 和 `yaw_goal_tolerance`，同时
  保持足够的进场空间。
- 机器人在窄口停住：先检查是否同时运行了两套 launch；执行
  `sg docker -c './scripts/stop.sh'` 后只启动一套，再检查
  `ros2 node list | sort` 中是否只有一个 `/collision_monitor`、`/camera_cloud` 和
  `/camera_obstacles`。如果日志是 `Robot to slowdown`，这是安全层看到近距离障碍
  后的预期减速；如果是 `Robot to stop`，先检查 `/camera/cloud` 的 TF 和时间戳。
  如果 `/cmd_vel` 长时间只在约 `+/-0.0395 rad/s` 摆动，说明运行中的参数不是当前
  配置，确认 `/controller_server` 的 `FollowPath.vtheta_samples` 为 `7`。不要直接
  无限增大 `source_timeout`，也不要为了“挤过”通道继续缩小真实机器人的 footprint。

碰撞安全区在 [collision_monitor_rgbd_params.yaml](src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml)，
场景在 [indoor_obstacle_course_large.world](src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world)，
启动链路在 [demo.launch.py](src/rtabmap_tb3_nav/launch/demo.launch.py)。

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

### 本轮回归证据

窄口调参后，Gazebo + RViz2 开启时的 A -> B 返回 Nav2 状态 `4`，末端距离约
`0.12 m`；约 `307587` 条 Gazebo contacts 记录过滤地面后没有机器人与障碍物接触。
干净重启并从 B 出发的 B -> A 也返回状态 `4`，末端距离约 `0.10 m`；约 `350431`
条 contacts 记录同样没有非地面障碍接触。两次验证时 `/controller_server`、
`/planner_server` 和 `/collision_monitor` 都是 `active [3]`。

随后使用 `vtheta_samples=7` 做了无 GUI 双向回归：A -> B 返回 status `4`、末端距离
约 `0.10 m`，约 `346295` 条 contacts 过滤后无障碍接触；B -> A 返回 status `4`、
末端距离约 `0.12 m`，约 `373473` 条 contacts 过滤后无障碍接触。两次最新控制器日志
均没有 `Failed to make progress` 或 `Aborting handle`；窄口附近出现的短暂停顿对应
安全层 slowdown 和 RGB-D 障碍层更新后的局部路径重规划，不是碰撞。

最终修复重新启动后，在线全局插件列表只剩 `obstacle_layer` 和 `inflation_layer`，
不再加载会随 `/map` 增长而 resize 的 `StaticLayer`。Gazebo + RViz2 下 A -> B 返回
status `4`、末端约 `0.12 m`、`358524` 条 contacts 过滤后无障碍接触；B -> A
返回 status `4`、末端约 `0.12 m`、`342705` 条 contacts 过滤后无障碍接触。两次日志
都没有 `StaticLayer: Resizing`、`PolygonStop`、`Failed to make progress` 或
`Aborting handle`。

如果 GUI 测试时出现 `Failed to change state for node: collision_monitor`，通常是旧
launch 没有完全退出而产生了重复节点，不是新的避障参数本身失败。先执行
`sg docker -c './scripts/stop.sh'`，确认 `ros2 node list | sort` 中每个关键节点只
出现一次，再重新启动一套仿真。

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
