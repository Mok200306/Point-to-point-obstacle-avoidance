# v4 复现、轨迹查看与跨场景验证说明

更新时间：2026-08-21

## 结论先行

`fast_goalline_045_v4` 已经在当前 `indoor_obstacle_course_large.world` 中完成了
“RGB-D 在线建图 + A -> B 点到点导航 + 障碍物绕行”的仿真验收：

- 不使用 `/scan`，障碍物感知来自 Gazebo RGB-D 深度点云；
- `online=true`、`localization=false`、`reset_db=true`，不要求先建一张完整地图；
- 3 次独立 A -> B 均为 Nav2 `status=4`；
- 过滤 `ground_plane` 后非地面 Gazebo contacts 为 `0/3`；
- wall 时间平均 `81.07 +/- 3.46 s`，Gazebo 路径平均 `17.382 +/- 0.050 m`。

因此，当前项目目标在这个仿真场景和固定起终点上已经实现。但这还不是“任意环境
都能自动选出好路线”的证明。v4 使用了当前大场景障碍布局对应的固定世界坐标
分段走廊先验，下一阶段应冻结参数、转向其他场景验证，而不是继续凭单张轨迹图调参。

## 1. 复现 v4

### 1.1 清理、启动和构建

在主机终端执行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
```

只有 Docker 镜像不存在，或修改了 `Dockerfile`、Gazebo 模型补丁时，才需要额外执行：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/build.sh'
sg docker -c './scripts/start.sh'
```

### 1.2 同时打开 Gazebo 和 RViz2

在第二个终端启动 v4：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=-8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

等待 Gazebo、RViz2、RTAB-Map 和 Nav2 lifecycle 节点进入 active。此时可以在 Gazebo
和 RViz2 中观察实时建图、全局代价地图、全局路径和局部跟踪过程。

### 1.3 只发送目标，不保存实验文件

如果只想看一次运行效果：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

这个命令会发送目标并打印 Nav2 status，但不会生成轨迹图。

### 1.4 保存完整轨迹、指标和 contacts

建议使用唯一 label。下面的命令会保存一份新的正式复现记录：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5 --label manual/v4_repro_A_to_B_run_01 --profile fast_goalline_045_v4'
```

运行结束后，主机目录中会出现：

```text
results/manual/v4_repro_A_to_B_run_01/
  experiment.yaml
  metrics.yaml
  nav2_rgbd_params.yaml
  collision_monitor_rgbd_params.yaml
  world.sdf
  trajectory.csv
  gazebo_trajectory.csv
  trajectory.png
  trajectory_comparison.png
```

其中 `metrics.yaml` 应重点检查：

```text
nav2_status: 4
succeeded: true
gazebo_non_ground_contact: false
gazebo_contact_pairs: (none)
```

`experiment.yaml` 会记录本次 label、profile、Git commit、世界、目标和视图定义。
v4 的 profile 覆盖定义在
[`demo.launch.py`](src/rtabmap_tb3_nav/launch/demo.launch.py)；基础参数在
[`nav2_rgbd_params.yaml`](src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml)。

## 2. 如何查看轨迹图

v4 完整复现后执行：

```bash
xdg-open /home/w417/RTAB-Map/results/manual/v4_repro_A_to_B_run_01/trajectory_comparison.png
```

也可以直接在文件管理器中打开该 PNG。`trajectory_comparison.png` 是左右双视图：

- 左侧：Gazebo SDF 障碍物俯视图和 `/gazebo/model_states` 真值轨迹；
- 右侧：RViz 风格的 `/map`、global costmap 和 map 坐标系轨迹。

同一目录中的 `trajectory.png` 是右侧 RViz 风格单图。需要数值分析时查看：

```bash
sed -n '1,120p' results/manual/v4_repro_A_to_B_run_01/metrics.yaml
head -5 results/manual/v4_repro_A_to_B_run_01/trajectory.csv
head -5 results/manual/v4_repro_A_to_B_run_01/gazebo_trajectory.csv
```

仓库中已经保留的三次 v4 证据位于：

- [`run_01`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01)
- [`run_02`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_02)
- [`run_03`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_03)

详细统计见
[`NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md`](NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md)。

## 3. 会不会覆盖以前的轨迹

会不会覆盖取决于 label：

- 使用新的 label，例如 `manual/v4_repro_A_to_B_run_01`，不会覆盖其他目录；
- 再次使用完全相同的 label，会覆盖该目录中的 `metrics.yaml`、CSV、PNG、世界文件和
  参数快照，因为脚本会使用同名路径并以写入模式重新生成文件；
- 不要把新实验 label 写成仓库已保存的
  `optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01`，否则会覆盖归档证据；
- 推荐用 `manual/v4_repro_A_to_B_run_01`、`run_02`、`run_03`，或加入日期和实验目的。

每次独立运行前都建议重新执行 `stop.sh`、`start.sh` 和启动命令。这样 Gazebo、RTAB-Map
数据库和机器人起点不会继承上一次运行状态。`reset_db=true` 会在在线建图启动时清理
本次仿真的 RTAB-Map 数据库，但不会删除 `results/` 中使用其他 label 的轨迹。

## 4. v4 到底是如何完成任务的

当前链路是：

```text
Gazebo RGB-D
  -> /camera/cloud、/camera/obstacles
  -> RTAB-Map 在线更新 /map
  -> map_padder 生成固定尺寸 /nav_map
  -> global costmap：StaticLayer + RGB-D obstacle layer + inflation
  -> GoalLineSmacPlanner：Smac A* + 目标线/走廊软代价
  -> stable replanning behavior tree
  -> Regulated Pure Pursuit：前视跟踪、曲率调速、碰撞预测
  -> velocity_smoother
  -> collision_monitor
  -> /cmd_vel_safe -> Gazebo TurtleBot3
```

v4 的规划不是把一条固定轨迹直接发送给小车：

1. Smac 仍然根据当前 global costmap 搜索可行路径；lethal 障碍、车体 footprint 和未知空间规则仍有效。
2. GoalLineSmacPlanner 对偏离起点到终点直线的自由栅格增加软代价，鼓励障碍物结束后回到目标方向。
3. v4 额外使用当前大场景的 `x/y` 走廊 schedule：`y=0.95 -> 0.75 -> 0.60 -> 0.68 -> 0.58 -> 0`。
   这是软偏好，不是硬轨迹，但它确实包含了当前障碍物布局的先验。
4. RPP 根据全局路径前视点跟踪，负责提前转弯和弯道减速；它不会替代全局规划器寻找另一条拓扑路线。
5. collision monitor 是最后的安全过滤层，触发 slowdown/stop 时可以让机器人减速或停止，不能把它当作路径规划器。

## 5. 当前是否真的实现了项目目标

对于当前仿真验收范围，答案是“是”：只有模拟 RGB-D、没有真实 LiDAR 的情况下，已经完成了在线建图、点到点导航和静态障碍绕行。

但必须限定为：

- 当前 `indoor_obstacle_course_large.world`；
- 当前 A `(-8.5, 0.0)` 到 B `(8.5, 0.0)`；
- 当前 TurtleBot3 Waffle footprint、速度、相机视场和 Gazebo 物理模型；
- 当前 3 次回归样本，而不是所有未知环境的统计保证。

这可以作为“当前场景的 RGB-D 在线导航基线/候选方法”，还不能写成“已经证明适配任意室内环境”。真实 D435i 的遮挡、反光、深度噪声、TF 延迟、安装高度和底盘控制误差也尚未通过硬件回归。

## 6. 能不能直接适配其他环境

需要区分“核心链路可复用”和“v4 profile 可直接泛化”：

### 可以复用的部分

- RGB-D PointCloud2 感知和 RTAB-Map 在线建图链路；
- 固定尺寸 `/nav_map`、global/local costmap、footprint 和碰撞监视器；
- Smac + RPP + velocity smoother 的规划控制结构；
- `status=4`、轨迹长度、末端误差和 contacts 这些评价指标。

### 不能直接保证的部分

v4 schedule 使用固定的 map/world 坐标窗口和北侧走廊。换房间、换障碍物位置、换起终点
方向后，它可能：

- 仍能成功，但绕行路线变长；
- 把代价偏好施加在不合适的一侧；
- 在可行通道附近增加不必要的代价，导致规划失败或等待重规划；
- 在地图坐标范围、房间尺度或障碍拓扑变化后产生与目标不一致的路线。

Smac 的 lethal cell 和 footprint 仍会阻止它主动穿过实体障碍，所以这不是“换场景必然
撞车”；但没有三次新场景回归前，也不能把它写成通用导航方案。

## 7. 下一步：冻结 v4，先做跨场景验证

建议暂时不调 inflation、速度、RPP 或 footprint，按以下顺序验证：

1. 先用现有小场景 `indoor_obstacle_course.world` 做 smoke test。它的标记点约为
   A `(-4.5, 0.0)`、B `(4.5, 0.0)`，启动时必须显式设置 `x_pose:=-4.5`，目标也不能继续使用 `8.5`。
2. 在小场景中先使用 v4 原样运行，观察固定走廊先验是否仍合理；只改变 `world`、初始位姿和目标，不改变 profile。
3. 通过后再制作一个障碍位置/方向不同的第二个大场景，仍然原样运行 v4。只有不修改 v4 走廊、仍能稳定通过，才能说明它有一定跨布局能力。
4. 每个新场景至少 3 次干净重启，记录成功率、wall 时间、Gazebo 路径、末端误差、最小近似净空、最大偏移和物理 contacts。
5. 若 v4 失败，先归类为“固定先验失配”还是“RGB-D/costmap/TF 问题”，不要立即修改多个参数。

小场景启动示例：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh world:=obstacle_course x_pose:=-4.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

当前 `scripts/regression_leg.sh` 的 contacts topic、世界文件和结果快照仍写死为
`indoor_obstacle_course_large`。因此上面的其他场景命令目前只能用于探索性观察；不能
直接把它的输出当作完整 contacts 回归。跨场景正式实验前，应先把回归脚本的 world、
contacts topic、起终点和轨迹绘图参数化，再进行三次正式验证。

## 8. 与真实 D435i 的关系

当前最合理的顺序是：

```text
当前大场景 v4 冻结
  -> 现有小场景 smoke test
  -> 障碍布局变化场景三次回归
  -> 再接入固定安装 D435i 做传感器/TF/延迟验证
  -> 最后才在真实底盘上降低速度做安全测试
```

仿真通过并不等于真实 D435i 已经通过；但如果 v4 在多个仿真布局中不改参数仍能满足
成功、无碰撞和轨迹约束，就可以更有依据地进入真实传感器迁移阶段。
