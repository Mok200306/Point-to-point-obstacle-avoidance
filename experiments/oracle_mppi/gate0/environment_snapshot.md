# Gate 0 环境快照

> 本文件是 Oracle 实验的环境冻结记录。主机信息已核对；容器内 ROS/Gazebo/Nav2/RTAB-Map 的精确版本在首次启动容器后由命令补齐。实验结果中的 `experiment.yaml` 还会记录运行时 commit、world、profile 和 Goal。

## 记录元数据

| 字段 | 值 |
|---|---|
| 快照日期 | 2026-08-27 |
| 仓库 | `/home/w417/RTAB-Map` |
| 分支 | `exp/oracle-mppi-2026-08-27` |
| 基线 commit | `69472452fcf8482db9a9e314ceebc927893df553`（Gate 0 取证脚本提交前的算法基线；正式运行 commit 见每个 run） |
| 基线提交 | `docs: add complete navigation technical report` |
| 任务书 | `/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx` |
| ROS 时间口径 | `use_sim_time=true`；Gazebo `/clock` 为时间源 |

## 主机

```text
OS: Ubuntu 20.04.6 LTS
Kernel: 5.15
GPU: NVIDIA GeForce RTX 4090
NVIDIA driver: 580.82.09
GPU memory: 24564 MiB
Docker: 28.1.1
```

## 容器与软件版本

实测输出已保存为 [software_versions.txt](software_versions.txt)。关键版本如下：

| 组件 | 实测版本/值 |
|---|---|
| ROS | Humble |
| Nav2 bringup / MPPI / RPP / collision monitor | 1.1.20 |
| RTAB-Map ROS | 0.23.7 |
| Gazebo ROS | 3.9.0 |
| Gazebo classic | 11.10.2 |
| TurtleBot3 Gazebo | 2.3.8 |
| Python / PyYAML / matplotlib / numpy | 3.10.12 / 5.4.1 / 3.5.1 / 1.21.5 |
| RMW_IMPLEMENTATION | 未显式设置，使用镜像默认中间件 |

版本命令：

```bash
docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash; ros2 pkg xml nav2_bringup; ros2 pkg xml nav2_mppi_controller; gazebo --version; python3 --version'
```

## Docker 配置

- compose service：`ros2`
- image：`rtabmap-tb3:humble`
- network：`host`
- IPC：`host`
- GPU：`gpus: all`
- TurtleBot3 model：`waffle`
- ROS domain：默认 `0`
- Gazebo GUI 通过宿主机 X11 显示
- 项目以 `/workspaces/rtabmap_tb3_nav` 读写挂载

## 主要节点、话题与 TF

```text
Gazebo camera -> /camera/image_raw, /camera/depth/image_raw, /camera/camera_info
             -> rtabmap (online mapping)
             -> /map, map -> odom

/camera/depth/image_raw -> point_cloud_xyz -> /camera/cloud
/camera/cloud -> obstacles_detection -> /camera/obstacles, /camera/ground

/map -> map_padder -> /nav_map
/nav_map + /camera/obstacles -> global/local costmap
global costmap -> GoalLineSmacPlanner -> planner_server
planner path -> RPP -> /cmd_vel -> velocity_smoother -> collision_monitor
collision_monitor -> /cmd_vel_safe -> TurtleBot3 base
```

Gate 0 不使用真实或模拟 `/scan`；这是 RGB-D-only 约束的一部分。

预期核心 TF：

```text
map -> odom -> base_footprint -> base_link -> camera_link / camera optical frames
```

首次启动后建议保存：

```bash
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list | sort && ros2 run tf2_tools view_frames"'
```

## Gate 0 固定输入

```text
world: indoor_obstacle_course_large.world
profile: adaptive_goal_line_045
online: true
localization: false
reset_db: true
rviz: true
gazebo_gui: true
use_sim_time: true
controller: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
planner: rtabmap_tb3_nav/GoalLineSmacPlanner
```

## 输入文件哈希

```text
world: src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
sha256: 93e36a103b2564fe6e3af7deda97c27b88712d96648dc45ca9e5e7221df9adb8
nav2: src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml
sha256: 95084f5f3652d39a4ac7cf951ccc42f83c7bccf690e50e15b83c517302c39c99
launch: src/rtabmap_tb3_nav/launch/demo.launch.py
sha256: 792ff55e4cb3e075507033f130a7c260d10d52dcd806119b0495b341c6533bfa
```

如果任一版本、镜像或输入文件发生变化，必须建立新的环境快照并说明其对 Gate 的影响，不能静默覆盖本节。
