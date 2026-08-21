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

## 9. 关于“实时建图”和环境记忆的准确解释

### 9.1 我们现在是不是实时建图

是。当前 v4 使用的是：

```text
online=true
localization=false
Mem/IncrementalMemory=true
subscribe_scan=false
```

RTAB-Map 在收到 RGB、深度图和机器人里程计后，持续创建新的视觉/深度节点，估计
机器人位姿并更新 `/map`。与此同时，`/camera/obstacles` 进入 Nav2 的 obstacle layer，
所以机器人可以在建图过程中同时规划和行驶，不需要先单独完成一遍完整建图。

但是“实时建图”不等于“在任何陌生环境中必然找到路线”。当前能力的准确边界是：

- 相机已经观察到的空间会被加入地图，Smac 可以在当前 costmap 中搜索可行路径；
- 行驶过程中发现新的障碍物时，地图和代价地图会更新，Nav2 可以重新规划；
- 相机视野之外、尚未被观测的房间和通道没有可靠几何信息；
- 当前工程没有实现完整的 frontier exploration（主动寻找未知区域并探索整张房间）；
- 如果目标所在区域没有被观测、里程计/TF 不稳定、深度相机漏检，不能保证自动找到目标。

因此，当前 v4 可以称为“RGB-D 在线建图下的点到点导航”，不能称为“对任意陌生
环境都具备自主探索能力”。点到点目标最好位于当前地图已覆盖或机器人能够逐步观察
到的区域内。

### 9.2 小车会不会记住已经走过的地方

会，但需要区分“当前运行记忆”和“下次启动继续使用数据库”：

| 状态 | 当前含义 | 是否保留到下一次启动 |
| --- | --- | --- |
| `Mem/IncrementalMemory=true` | 当前运行中持续加入 RTAB-Map 节点、地图和视觉特征 | 只有数据库没有被删除时才可能保留 |
| `reset_db=true` | 启动 RTAB-Map 时删除已有数据库，再从空图开始 | 否 |
| `reset_db=false` 且 `online=true` | 打开已有数据库并继续增量建图，具体行为依赖当前数据库和位姿连续性 | 是，但这是继续建图，不是纯定位 |
| `online=false localization=true` | 以已有数据库作为工作记忆，`Mem/IncrementalMemory=false`，进行定位 | 是，数据库只读使用 |

仿真数据库的宿主机路径是：

```text
/home/w417/RTAB-Map/.ros/rtabmap.db
```

当前 v4 正式命令明确使用 `reset_db=true`，所以每次正式回归都是一张新的地图；
三次 v4 结果不能解释为机器人跨实验记住了前两次路线。单次运行中，它会记住已经
经过的视觉/深度节点，并可能通过回环检测重新识别已经走过的位置。

如果先建立地图，再往返使用同一环境，推荐使用两种可复现实验：

1. **在线增量模式**：保留数据库，使用 `online=true localization=false reset_db=false`。
   机器人会继续更新地图，同时使用已有内容作为起点。
2. **固定地图定位模式**：停止建图进程后，使用 `online=false localization=true reset_db=false`。
   RTAB-Map 从已有数据库定位机器人，Nav2 使用已发布的 `/map`，不再把新的视觉节点
   加入长期记忆。

已有地图通常会让第二次往返更稳定，因为静态墙体和已经观察过的障碍物不必完全从零
开始建立，初始位姿也可以通过视觉定位获得。但它不保证路径一定更短或更聪明，原因包括：

- 地图可能过期，环境中的桌椅、箱子等动态物体可能已经改变；
- 当前深度相机仍会实时更新 local/global obstacle layer，旧地图不能替代实时避障；
- v4 的固定坐标走廊偏好仍可能把规划引向不合适的方向；
- 相机安装位姿、底盘 footprint 和地图坐标系发生变化时，旧地图可能不再匹配。

所以“记住地图”与“复用上一条轨迹”不是一回事。Nav2 仍然会根据当前起点、终点和
代价地图重新规划，并不会简单播放上次的控制指令。

### 9.3 `reset_db=true` 和数据库复用命令

当前 v4 的全新在线建图仍使用：

```bash
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=-8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

如果要测试“保留上一张图并继续在线建图”，应先正常停止上一轮，再使用新的结果 label，
并把 `reset_db` 改为 `false`：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=-8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=false navigation_profile:=fast_goalline_045_v4'
```

如果要测试“固定地图定位”，必须确保数据库确实由同一个世界、同一个相机/机器人
坐标系建立，再使用：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=-8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=false localization:=true reset_db:=false navigation_profile:=fast_goalline_045_v4'
```

这两类实验都不要使用已归档结果的 label，并应在 `experiment.yaml` 中记录数据库
模式。只有 `reset_db=false` 时才是在验证跨启动记忆；正式 v4 基线仍应继续使用
`reset_db=true`，这样不同实验之间不会相互污染。

## 10. “固定世界坐标分段走廊先验”到底是什么

### 10.1 它不是离散航点，也不是把车逐点开过去

用户看到的 v4 schedule：

```text
x = [-7.2, -3.4, -2.6, -2.25, 2.75, 3.20, 3.50, 7.40, 7.90]
y = [ 0.95,  0.95,  0.75,  0.60, 0.60, 0.68, 0.58, 0.58, 0.00]
```

不是 `NavigateToPose` 依次发送的九个目标点。它也不是 RTAB-Map 从历史轨迹中自动
学习出来的路线，而是 v4 profile 中针对当前 large world 手工写入的 map/world 坐标
偏好。

实际过程是：

1. GoalLineSmacPlanner 遍历 global costmap 中的栅格单元；
2. 根据该单元的 `world_x`，对相邻 schedule 点做线性插值，得到这一列期望的 `target_y`；
3. 单元偏离目标线或该 `target_y` 越远，就增加一个软 cost；
4. lethal cell、inscribed/inflated obstacle cell 和机器人 footprint 不会被这个软代价
   改成可通行；
5. Smac A* 仍然在整张 costmap 上搜索从当前起点到目标点的完整栅格路径；
6. 规划完成后，RPP 连续跟踪整条路径，必要时 Nav2 还会重新规划。

代码会在规划调用期间临时修改可调整的 costmap 栅格，调用继承的 Smac planner 后恢复
原始栅格。因此 schedule 是“路径评价偏好”，不是永久地图，也不是控制器航点。

可以把它理解为：

```text
障碍物和 footprint = 不能穿过的硬约束
目标线偏好和走廊 schedule = 可以偏离、但会影响得分的软约束
Smac A* = 根据硬约束 + 软代价搜索完整路径
RPP = 连续跟踪这条路径并根据曲率/障碍风险调速
```

### 10.2 为什么 v4 仍然不是完全自主的通用规划

正常的未知环境导航确实不应该提前知道“必须经过哪几个世界坐标”。在线建图解决的
是“边走边获得地图”，Smac 解决的是“在当前地图中从 A 搜索到 B”；而 v4 schedule
额外告诉规划器：“在这个已知 benchmark 中，障碍物大致应该从哪一侧绕，并在何处
逐渐回到目标线。”它是为了让当前大场景的重复实验更稳定，并不是 RTAB-Map 的记忆。

因此换场景时：

- 地图和 costmap 会重新使用 RGB-D 观测建立，Smac 仍有基础避障能力；
- 但 `x/y` schedule 仍处于旧坐标系，可能偏向错误的一侧或不必要地惩罚合理通道；
- 如果新场景没有相同的障碍拓扑和坐标范围，不能直接宣称 v4 已经泛化；
- 真正验证通用能力，应在不改 v4 参数的前提下先测试新场景；如果失败，再把失败
  区分为“固定先验失配”还是“感知、TF、地图/代价地图故障”。

要验证规划器本身，而不是验证 benchmark 先验，后续应增加一个“通用 profile”：关闭
`side_bias_target_schedule_enabled` 和固定 `side_bias`，只保留 goal-line soft bias、
真实 costmap 和 Smac/RPP。两者需要分开报告：

| profile | 主要验证对象 | 是否含当前大场景坐标先验 |
| --- | --- | --- |
| `fast_goalline_045_v4` | 当前大场景的快速、重复性基线 | 是 |
| 后续 `generic_rgbd_045` | RGB-D + costmap + Smac + RPP 的跨场景能力 | 否 |

在没有完成新场景实验前，不能把 v4 的 3/3 成功写成“任意场景均可导航”。

## 11. 下一步工作和真实 D435i 的进入条件

### 11.1 推荐执行顺序

当前阶段不再同时调速度、inflation、RPP 和 footprint。建议按以下顺序推进：

1. **保留 v4 基线**：继续保存提交、参数快照和三次 large-world 轨迹；不覆盖归档目录。
2. **小场景 smoke test**：只改变 world、起点和终点，保持 v4 参数不变，验证 schedule
   失配时机器人是否仍能完成基本导航。
3. **新障碍布局场景**：制作障碍物位置/方向不同的第二个场景；先直接运行 v4，记录它
   是否会主动选择可行通道。
4. **通用 profile 对照**：关闭固定 schedule，和 v4 做同一场景、同一起终点、至少三次
   对比，分别记录成功率、耗时、路径长度、末端误差、最小净空和物理 contacts。
5. **数据库复用实验**：分别比较 `reset_db=true` 在线建图、`reset_db=false` 继续建图、
   `localization=true` 固定地图三种模式，确认“地图记忆”带来的真实收益。
6. **再做硬件迁移**：先验证 D435i 数据、TF 和底盘闭环，再在低速近距离目标上测试；
   不要一接入相机就直接发送远距离导航目标。

小场景观察命令：

```bash
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh world:=obstacle_course x_pose:=-4.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

然后在另一个终端发送小场景目标：

```bash
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 4.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

注意：当前 `regression_leg.sh` 的 large-world contacts topic 和 `world.sdf` 快照仍是
写死的。小场景和新场景在脚本参数化前只能作为探索性观察，不能直接当作与 v4 同等级
的物理零碰撞正式证据。

### 11.2 什么时候可以接入真实 D435i

不是必须等到“任意未知场景都完美”，但至少要满足下面的硬门槛：

- large world v4 已冻结，且现有 3 次无碰撞结果可复现；
- 至少一个不同障碍布局的仿真场景完成 3 次成功/失败统计；
- 已把回归脚本的 world、contacts topic、目标和结果快照参数化；
- 真实底盘可以独立完成低速前进、停止和原地旋转；
- 底盘稳定发布 `odom -> base_link`，并提供正确的 `base_link -> base_footprint`；
- D435i 的 `camera_link` 安装位姿通过实测和 RViz 验证，而不是继续使用示例值；
- `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/cloud`
  和 `/camera/obstacles` 稳定有数据，TF 时间戳没有明显延迟；
- `/cmd_vel_safe` 的安全输出经过低速空载测试，机器人不会绕过 collision monitor。

满足这些条件后，可以进入真实 D435i 的“传感器/TF/低速闭环验证”，但仍不等于可以
立刻在真实环境中高速导航。真实迁移建议分四步：

```text
USB/驱动验证
  -> 固定安装 TF + RViz 对齐验证
  -> 低速在线建图和近距离直线目标
  -> 小范围静态障碍绕行，再逐步增加距离和速度
```

第一次真实测试建议使用独立数据库 `~/.ros/rtabmap_d435i.db`、
`online=true localization=false reset_db=false`，先人工低速移动观察地图，不发送
远距离目标。确认地图、TF、深度点云和安全速度链路都稳定后，才测试 1 m 左右目标；
建图稳定后再另开一次 `localization=true` 实验验证已保存地图的复用。

当前结论是：**仿真链路已经达到接入真实 D435i 做低速传感器验证的准备阶段，但还没有
达到真实底盘正式导航验收阶段。** 真实 D435i 会验证传感器噪声、安装误差、遮挡、USB
带宽和底盘控制误差；这些问题不能由当前 Gazebo 3/3 结果替代。

## 12. 为什么只执行构建命令看不到 Gazebo 和 RViz2

下面三条命令的作用分别是：

```text
stop.sh       停止并删除旧的 Compose 容器
start.sh      启动 ros2 容器，但不启动 Gazebo、RViz2 或 Nav2
colcon build 只编译 ROS 2 工作区，不运行任何节点
```

所以只执行这三条命令时，既不会出现 Gazebo，也不会出现 RViz2；这不是“无图导航”，
而是还没有启动仿真入口。启动仿真还必须执行 `launch_demo.sh`，并保持该终端运行。

### 12.1 正确的 v4 可视化启动顺序

终端 1 执行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=-8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

最后一条命令会以前台方式启动：

```text
Gazebo gzserver + Gazebo GUI
TurtleBot3 Waffle
Gazebo RGB-D 相机
RTAB-Map online SLAM
Nav2 global/local costmap、Smac、RPP
RViz2
```

不要关闭这个终端。等待 Gazebo 和 RViz2 窗口出现、Nav2 lifecycle 节点 active，并让
RTAB-Map 先运行几秒，再在终端 2 发送目标：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
```

这里的 `online=true localization=false` 表示“运行时边建图边导航”，不是无图：
RTAB-Map 会从一张很小的初始地图开始，随着相机移动逐步更新 `/map`；Gazebo 世界本身
始终存在，RViz2 显示的是当前已经观测到的地图、代价地图和路径。

### 12.2 如果已经执行 launch 仍然没有窗口

先确认容器和图形环境：

```bash
echo "$DISPLAY"
xhost
sg docker -c 'docker compose ps'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && ros2 node list"'
```

正常情况下，`docker compose ps` 中应有运行中的 `ros2`，节点列表中应逐步出现
`/gazebo`、`/rtabmap`、`/planner_server`、`/controller_server` 和 `/rviz2`。如果节点
存在但窗口没有出现，通常是当前终端没有可用的桌面 `DISPLAY`、通过 SSH 无 X11 转发，
或 X server 拒绝了容器连接。`scripts/start.sh` 已经执行了 `xhost` 授权，但它不能
替代无图形桌面本身。

同时确认 launch 参数没有被关闭：

```text
gazebo_gui:=true
rviz:=true
```

`rtabmap_viz:=false` 只关闭 RTAB-Map 自己的可视化窗口，不会关闭 Gazebo 或 RViz2。

### 12.3 修改起点和终点的正确方法

起点和终点由两个不同参数控制：

| 内容 | 控制方式 | 坐标系 |
| --- | --- | --- |
| Gazebo 中机器人出生位置 | `x_pose:=... y_pose:=...` | Gazebo world 坐标 |
| Nav2 终点位置 | `send_goal.py --x ... --y ... --yaw ...` | 默认 `map` 坐标 |
| Nav2 终点姿态 | `send_goal.py --yaw ...` | 弧度 |

例如把当前大场景的 A→B 改成 B→A，不需要改代码：

```bash
# 终端 1：从原 B 点出生
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=8.5 y_pose:=0.0 gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'

# 终端 2：发送原 A 点作为终点
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x -8.5 --y 0.0 --yaw 3.1415926 --settle-seconds 5"'
```

如果要使用自定义起终点，只需替换下面四个数值，并确认起点、终点都不在实体障碍物
内部：

```bash
# 起点：Gazebo world 坐标
sg docker -c './scripts/launch_demo.sh world:=obstacle_course_large x_pose:=START_X y_pose:=START_Y gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'

# 终点：map 坐标，yaw 使用弧度
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x GOAL_X --y GOAL_Y --yaw GOAL_YAW --settle-seconds 5"'
```

上面的 `START_X`、`GOAL_X` 等是占位符，不能原样输入；例如 `START_X` 要替换为
`-6.0`，`GOAL_X` 要替换为 `6.5`。也可以在 RViz2 中选择 `Nav2 Goal`，在当前
`map` 上点击目标位置并拖动箭头设置目标朝向。

### 12.4 改完起终点后是否需要重新编译

只改变 `world`、`x_pose`、`y_pose`、`--x`、`--y` 或 `--yaw` 时，不需要重新执行
`colcon build`。完整流程是：

1. 在 launch 终端按 `Ctrl+C` 停止当前仿真；
2. 执行 `stop.sh`，确保旧 Gazebo、RTAB-Map 和数据库进程退出；
3. 用新的 `x_pose/y_pose` 重新执行 `launch_demo.sh`；
4. 等待地图和 TF 稳定，再用新的 `--x/--y/--yaw` 发送目标；
5. 若要保存结果，使用新的 `--label`，不要覆盖旧实验目录。

只有修改了 `src/`、launch 文件、YAML 参数、Dockerfile 或 Gazebo 模型时，才需要重新
构建工作区或镜像。更改起终点不会自动改变代码中的 v4 schedule。

### 12.5 自定义起终点时最容易忽略的坐标问题

`x_pose/y_pose` 是 Gazebo spawn 参数，而 `send_goal.py` 的目标默认是 `map` 坐标。
当前 large-world A/B 实验中两者近似对齐，所以可以使用 `-8.5/8.5`；换场景或换起点
后，RTAB-Map 可能产生不同的 `map -> odom` 原点，不能无条件假设 Gazebo world 坐标
和 RViz 的 map 坐标完全相同。

建议自定义目标时：

1. 先启动仿真并观察 RViz 的 `map`；
2. 先发送机器人附近的短距离目标，确认 `map -> odom` 稳定；
3. 再在 RViz 中点击目标，或使用 RViz 显示的 map 坐标发送远距离目标；
4. 起点必须有足够的空地，目标必须不在障碍物和 inflation 的 lethal 区域内。

此外，当前 launch 只暴露了 Gazebo 的 `x_pose` 和 `y_pose`，没有暴露初始 `yaw_pose`。
机器人初始朝向沿用 TurtleBot3 spawn launch 的默认值；终点朝向仍可通过 `--yaw` 或
RViz 设置。如果实验确实需要自定义初始朝向，后续需要单独给 launch 增加 yaw 参数，
这属于代码改动，不能通过当前命令临时传入。

最后，`fast_goalline_045_v4` 只对当前 large world 的 A→B 走廊完成了正式验证。修改
起点、终点、方向或 world 后，它可以作为压力测试运行，但结果应保存为新的实验，不应
继续标记为 v4 A→B 验收，也不能把失败直接归因于“没有地图”。

## 13. 目标改变后的固定路线修正（2026-08-21）

本轮发现：把目标改到 A `(5.0,-3.0)` 后，机器人仍先沿旧的 `(8.5,0.0)` 路线行驶。复核 `GoalLineSmacPlanner` 后确认，v4 的 `side_bias_target_schedule` 使用了当前 large 场景的固定 world 坐标软代价。它不是离散航点回放，但会在每次 A* 规划中反复偏好旧走廊，因此不能作为新目标和新场景的默认方法。

现在 `demo.launch.py` 的默认 profile 已切换为 `adaptive_goal_line_045`。该 profile：

- 关闭 `side_bias_enabled`、固定 world 坐标范围和分段 schedule；
- 每次规划使用实际 start、当前 goal 和实时 global costmap；
- 保留 Smac、RPP、目标线软偏好、RGB-D costmap、在线 RTAB-Map 和 collision monitor；
- RViz 的 `/goal_line` 仅显示当前实际起点到目标的黑色参考线，Nav2 的 `/plan` 才是实际跟踪路径。

首轮连续双目标验证为 `(-8.5,0.0) -> A(5.0,-3.0) -> B(5.0,6.0)`：两段均 `status=4`，wall 时间分别为 `76.711 s` 和 `60.227 s`，总计 `136.938 s`；Gazebo contacts 过滤地面后非地面接触为 `(none)`。红色轨迹表示起点到 A，蓝色轨迹表示 A 到 B，黑色虚线表示每段当前起点到当前目标的直线参考。完整图像、CSV、指标和参数快照见：

- [未知目标实时规划优化说明_2026-08-21.md](未知目标实时规划优化说明_2026-08-21.md)
- [自适应目标线多目标实验记录_2026-08-21.md](自适应目标线多目标实验记录_2026-08-21.md)
- [自适应目标线参数记录_2026-08-21.md](自适应目标线参数记录_2026-08-21.md)
- [双视图轨迹图](results/自适应目标线_多目标_修正版_2026-08-21/trajectory_comparison.png)

这次结果证明了“目标改变后重新规划”在当前 large 场景的一次回归，但还不等于任意未知环境泛化。下一步应保持 adaptive profile 不变，换起点、终点和障碍布局做三次回归，再决定是否进入真实 D435i 的低速传感器/TF 验证。
