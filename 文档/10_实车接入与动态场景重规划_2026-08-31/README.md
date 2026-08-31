# 阶段 4：静态实车接入准备与动态场景重规划

更新时间：2026-08-31
当前状态：完成软件接入准备，尚未进行真实底盘运动和真实 D435i 验收。

## 文档目的

本阶段把项目从“静态仿真已完成”推进到“可以安全开始真实设备分级验收”。本阶段不
把 Gazebo 的成功率、耗时、轨迹或 contacts 写成实车结果，也不覆盖场景02 v13和旧
场景03的任何正式证据。

关联文件：

- [阶段4总结](阶段4_静态实车接入准备与场景03重规划总结_2026-08-31.md)
- [实车启动文件](../../src/rtabmap_tb3_nav/launch/real_d435i_nav.launch.py)
- [实车底盘接口配置](../../src/rtabmap_tb3_nav/config/real_water_ii_s.yaml)
- [只读预检程序](../../src/rtabmap_tb3_nav/scripts/real_robot_preflight.py)
- [动态场景03现有报告](../08_下一阶段实验归档_2026-08-22/05_场景03_动态障碍/阶段3_场景03动态障碍三次正式回归报告_2026-08-31.md)

## 本阶段已完成

1. 当前新实验和真实 launch 默认使用四点闭环 v13：
   `adaptive_goal_line_050_recovery_v13_line_tiebreaker`。
2. `adaptive_goal_line_045` 保留为通用对照和旧场景03历史 profile；历史结果不重命名、
   不覆盖、不重新统计。
3. 真实 launch 强制 `use_sim_time=false`，拒绝用 Gazebo 时钟启动实体机器人。
4. 真实 launch 不创建假里程计、不创建机器人模型；底盘驱动必须在外部提供
   `/odom` 和 `odom→base_link`。
5. Nav2、RPP、速度平滑和 collision_monitor 的实车参数在启动时生成临时有效快照，
   支持 profile、真实车体 footprint、初始速度/加速度、安全区和相机 topic 配置。
6. 底盘只允许接收 `/cmd_vel_safe`。`/cmd_vel` 只作为 collision_monitor 的输入，
   不允许直接接到底盘。
7. 增加只读 `real_robot_preflight.py` 和根目录包装脚本。它只创建订阅和参数查询，
   不发布 Twist、不调用 WATER TCP API、不切换软急停。
8. 新增 WATER II-S 参数记录，包括尺寸、网络地址、TCP 端口、控制接口范围、里程计
   边界和初始限速。

## 当前工作基础 profile

| profile | 当前用途 | 是否带固定世界坐标走廊 |
| --- | --- | --- |
| `adaptive_goal_line_050_recovery_v13_line_tiebreaker` | 四点闭环主线、后续实车静态起点 | 否 |
| `adaptive_goal_line_045` | 通用对照、旧场景01/场景03历史复现 | 否 |
| `fast_goalline_045_v4` | large 场景历史速度 benchmark | 是，不得迁移到新场景或实车 |

实车使用 v13 不代表实车已经达到 v13 仿真的 0.28 m/s。真实 launch 默认把首次上电
速度限制为 `0.12 m/s`，等待底盘和场地实测后再逐级放开。

## WATER II-S 已知接口与边界

资料来源：`/home/w417/文档/底盘/` 下的 WATER API 手册、底盘使用手册和规格书。

| 项目 | 记录值 | 验收要求 |
| --- | --- | --- |
| 型号 | WATER II-S | 现场核对铭牌和固件 |
| 车体尺寸 | 直径约 0.505 m，高约 0.280 m | 现场测量含载荷外廓 |
| 自重/额定载荷 | 45 kg / 50 kg | 记录本次实验载荷 |
| 驱动 | 两驱动轮，差速运动 | 确认底盘驱动模式 |
| 默认底盘地址 | `192.168.10.10` | 现场确认实际地址 |
| API TCP 端口 | `31001` | 现场确认固件端口 |
| 直接控制 | `/api/joy_control` | 仅由经过审核的底盘驱动调用 |
| 软急停 | `/api/estop` | 软件和硬件急停分开验证 |
| 速度接口范围 | 线速度约 -0.5～0.5 m/s，角速度约 -1～1 rad/s | 仍以厂商当前固件为准 |
| 单条控制命令 | 约 0.5 s，连续控制需高于 2 Hz | 驱动必须有 watchdog |
| 里程计 | 资料说明有编码器，但未确认标准 ROS `/odom` | 必须由厂商 ROS 驱动/SDK发布 |
| `/robot_status` 位姿 | 可用于状态观察 | 不能直接冒充 ROS odom |

仓库不保存底盘密码，也不在 launch 中主动扫描端口或发送运动/急停请求。

## ROS 接口契约

目标 TF：

```text
map → odom → base_link → camera_link
                         ├→ camera_color_optical_frame
                         └→ camera_depth_optical_frame
```

正式静态实车目标记录时，应把 `runtime_snapshot_dir` 指向本次目标的独立目录，并将
同一路径传给记录器的 `--runtime-snapshot-dir`；记录器会把有效导航、碰撞监视、相机
参数和运行时元数据复制到该次结果目录。若没有有效快照，不能把该次记录当作完整正式
证据。

必须存在：

| 接口 | 发布/订阅方 | 说明 |
| --- | --- | --- |
| `/odom` (`nav_msgs/Odometry`) | 底盘驱动发布 | 不能用地图位姿伪造 |
| `odom→base_link` | 底盘驱动发布 | 时间戳和 frame 必须稳定 |
| `base_link→camera_link` | 实车 launch 静态 TF | XYZ/RPY 必须实测替换暂定值 |
| 彩色、深度、CameraInfo | D435i 发布 | 深度和彩色必须对齐 |
| `/camera/cloud` | 点云节点发布 | 低延迟避障输入 |
| `/camera/obstacles` | 障碍物处理节点发布 | costmap 输入 |
| `/cmd_vel` | Nav2/速度平滑发布 | 只能进入 collision_monitor |
| `/cmd_vel_safe` | collision_monitor 发布 | 底盘驱动唯一速度入口 |

## 启动和只读预检

启动前先接入厂商底盘驱动和 D435i，但不要发送导航目标：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
```

启动真实导航准备链（默认 v13、真实时间、低速）：

```bash
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 launch rtabmap_tb3_nav real_d435i_nav.launch.py navigation_profile:=adaptive_goal_line_050_recovery_v13_line_tiebreaker online:=true localization:=false reset_db:=false rviz:=true rtabmap_viz:=false runtime_snapshot_dir:=/workspaces/rtabmap_tb3_nav/实车记录/预检_v13"'
```

另一个终端运行只读预检：

```bash
./scripts/real_robot_preflight.sh --duration 8 --output 实车记录/预检_v13/preflight.json
```

只有现场人员确认物理急停可用、周围无危险并希望把报告标为完整通过时，才额外加
`--estop-confirmed`。这个参数只记录人工确认，不会操作急停。

预检失败时的常见含义：

- 缺 `/odom` 或 `odom→base_link`：底盘 ROS 驱动/SDK尚未完成；
- 缺 CameraInfo 或深度图：D435i、USB带宽或驱动参数有问题；
- 缺 `/cmd_vel_safe` 订阅者：底盘尚未接入安全速度入口；
- 出现 Gazebo graph：停止仿真，避免同名 topic/TF 混用；
- `use_sim_time=true`：不能进入实体机器人测试；
- `base_link→camera_link` 不可用：先处理安装 TF，不发送目标。

## 静态实车分级放行

每一级必须由人工急停监护，并记录日期、载荷、底盘固件、相机序列号和 Git commit。
任一级失败都回退到安全状态，不跳级。

| 级别 | 动作 | 放行条件 |
| --- | --- | --- |
| H0 | 断电/急停/轮子与线缆目视检查 | 硬急停可触发，人员站位安全 |
| H1 | 只启动 D435i | 彩色、深度、CameraInfo、IMU稳定 |
| H2 | 只启动底盘驱动 | `/odom`、时间戳、`odom→base_link`稳定 |
| H3 | 空载极低速直行/旋转/软件停止 | 仅通过审核的底盘驱动；停止可重复 |
| H4 | 启动 collision_monitor 和预检 | `/cmd_vel_safe` 是唯一底盘速度入口 |
| H5 | 固定相机 TF 和点云 | RViz中点云方向、地面、高度过滤正确 |
| H6 | 约 1 m 无障碍直线目标 | v13 实车低速参数，人工跟随 |
| H7 | 单个静态箱体绕行 | 无碰撞，保留轨迹/耗时/误差/最小间距 |
| H8 | 多次室内静态点到点 | 达到论文规定的重复次数后才进入动态实车 |

H0-H4 是底盘安全和接口验收，不应被“导航成功”替代。H6 之前不得发送远距离目标；
H8 之前不得把动态障碍物带到实体机器人旁边。

## 实车实验记录要求

每次静态实车实验建立新的目录，不覆盖前一次记录，至少保存：

- `trajectory_comparison.png`（如实车记录器暂不支持，则保存 RViz/rosbag 证据并注明）；
- 规划轨迹 CSV、底盘 `/odom` CSV、目标和最终位姿；
- `metrics.yaml`：成功、耗时、路径长度、末端误差、最小净空、碰撞/急停和失败原因；
- 当前 Git commit、有效 v13 参数快照、collision_monitor 参数快照和相机参数快照；
- D435i 序列号、分辨率、深度范围、相机外参和底盘载荷；
- TF、topic、频率和人工急停确认记录。

真实设备没有 Gazebo contacts；应改为底盘碰撞/急停事件、人工观察和视频/rosbag证据，
不能写成 `0/3 Gazebo contacts`。

## 场景03重新设计，而不是复制场景02

已有 `indoor_obstacle_course_cross_scene_03_dynamic.world` 及其 045 基线结果保留为
历史诊断；它没有验证“障碍物稳定挡住小车必经路线”的核心问题。新的场景03应使用
独立世界文件和独立结果目录，最小设计为：

```text
起点 ───────────────→ 目标点
              ↑
        动态障碍物进入必经通道
```

建议只改变一个环境因素：在空旷直线走廊中加入一个可重复、可观测、可记录的动态箱体
或低速移动目标。固定起点和目标在同一直线上，动态障碍物从侧方横穿或在前方进入，
使机器人必须减速、停车、等待、绕行或重新规划。不得向规划器写入固定路线、障碍物
未来真值或额外 waypoint。

新的实验顺序：

1. 无动态障碍直线路径，确认 v13 静态基线；
2. 动态障碍静止挡路，确认当前障碍层和 collision_monitor 是否停车；
3. 动态障碍低速同向；
4. 动态障碍横穿；
5. 动态障碍接近机器人；
6. 在同一世界逐步加入检测、最近邻/Kalman 跟踪、恒速预测和动态局部规划。

每个新世界先 smoke，再做三次正式回归；每次保存轨迹图、CSV、metrics、参数快照、
世界快照和 contacts。旧场景03结果不能混入新场景统计。

推荐消融：

```text
A 当前障碍层
→ B 障碍检测 + 跟踪
→ C 跟踪 + 恒速预测（1～2 s）
→ D 预测 + 动态局部规划
```

第一验收目标是危险区域可靠停车且无碰撞，之后才要求等待和绕行。RPP、目标线规划器
和 collision_monitor 本身不等于动态预测算法。

## 当前边界

- 本阶段没有真实设备在线，实车成功率、耗时、轨迹和 contacts 均为“未测”；
- WATER API 文档已整理，但没有在本阶段主动连接 `192.168.10.10:31001`；
- 未确认厂商是否提供标准 ROS `/odom`，不能用 `/api/robot_status` 代替；
- 相机安装外参、footprint、底盘轮径/轮距和加速度仍须现场测量或由驱动提供；
- 场景02 v13 的 3/3 结果是静态 Gazebo证据；旧场景03是 045 基线，2/3 无碰撞，不能
  证明动态预测或实车安全。

## 阶段出口

满足以下条件后，才进入静态实车正式实验：

1. H0-H5 全部通过并有记录；
2. `/odom` 和 `odom→base_link` 连续稳定；
3. D435i 彩色/深度/CameraInfo/IMU和点云方向正确；
4. 底盘只订阅 `/cmd_vel_safe`；
5. 约 1 m 低速目标在空旷地面可人工随时停止；
6. 每次记录可追溯到 Git commit、参数和硬件身份。

本阶段完成的是“可以开始按 H0-H8 验收”，不是“已经完成 H0-H8”。
