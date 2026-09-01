# WATER SDK → ROS 2 桥接接入说明

更新时间：2026-09-01
适用工作区：`/home/w417/RTAB-Map`
当前研究基础：`adaptive_goal_line_050_recovery_v13_line_tiebreaker`

## 1. 先给结论

厂家 SDK 不是现成的 ROS 2 底盘驱动，它的链路是：

```text
WaterChassis Python API
        ↓
本机 FastAPI Gateway
        ↓
TCP 192.168.10.10:31001
        ↓
WATER II-S 底盘
```

本项目增加 ROS 2 bridge，把唯一安全速度入口接到 SDK：

```text
D435i USB ───────────────┐
                         ▼
移动笔记本 Docker/ROS 2
  RTAB-Map → Nav2 → collision_monitor
                         │ /cmd_vel_safe
                         ▼
              water_chassis_ros_bridge.py
                         ▼
             WaterChassis → Gateway → WATER
```

台式机当前不参与控制链路。后续迁移时，台式机替换移动笔记本即可，不应让两台电脑同时
运行两个底盘控制器。

当前 SDK 提供厂家地图位姿和速度反馈，但没有经过确认的标准 ROS 编码器
`/odom` 与 `odom→base_link` 发布接口。API 的 `current_pose` 只能用于状态观察，
不能直接冒充 ROS `/odom`。

## 2. 已接入的文件

| 文件 | 作用 |
| --- | --- |
| `water_chassis_ros_bridge.py` | 只订阅 `/cmd_vel_safe`，调用 `set_velocity()`，发布状态；可选发布暂定积分里程计 |
| `water_chassis_bridge.launch.py` | 只启动 bridge，供接口测试，不启动相机、RTAB-Map 或 Nav2 |
| `real_d435i_nav.launch.py` | 在完整实车链中可选启动 bridge，默认仍关闭 |
| `real_water_ii_s.yaml` | 记录 WATER II-S 网络、接口、速度和里程计契约 |
| `tools/mock_chassis_server.py` | 本机 mock TCP 底盘，不接触真实设备 |
| `tools/self_test.py` | SDK 离线自测 |

厂家 `main.py` 保留为独立 SDK 示例，不与本项目 Nav2 同时使用。

## 3. 两个必须分开的开关

| 模式 | `water_enable_motion` | `water_allow_provisional_odom` | 行为 | 用途 |
| --- | ---: | ---: | --- | --- |
| 诊断默认模式 | `false` | `false` | 读取状态；不转发速度；不发布 bridge 的 `/odom`/TF | 首次接入、确认网络和状态 |
| 软件联调模式 | `true` | `true` | 转发 `/cmd_vel_safe`；用 SDK 反馈速度积分暂定 `/odom`/TF | mock 或受控软件联调 |
| 真实编码器模式 | `true` | `false` | 转发 `/cmd_vel_safe`；bridge 不发布 `/odom`/TF | 外部编码器驱动已提供正式 `/odom` 和 TF |

默认两个开关都是 `false`。`enable_motion=true` 只表示明确授权该 bridge 成为当前
唯一的速度控制拥有者，不代表已经具备正式实车导航条件。`allow_provisional_odom=true`
只是临时积分结果，不能作为论文实车指标。

如果厂家 ROS 驱动已经订阅 `/cmd_vel_safe` 并发布编码器 `/odom`，应使用厂家驱动，
并保持 `use_water_bridge:=false`；不能让厂家驱动和本 bridge 同时控制底盘。

## 4. 笔记本第一次接入方式

第一次现场建议采用“一台移动笔记本承担 ROS 2 计算”的结构：

1. D435i 固定在车上，USB 3.0 数据线接移动笔记本；
2. 移动笔记本通过 WATER Wi-Fi 或网线连接底盘所在局域网；
3. 移动笔记本 Docker 内运行相机驱动、RTAB-Map、Nav2、collision_monitor 和 bridge；
4. 台式机先不要运行 ROS 2、Gazebo 或任何底盘控制节点；
5. 现场必须确认硬件急停可触达，且没有厂家自动导航任务和第二个速度控制器。

启动前：

```bash
cd /home/w417/RTAB-Map
./scripts/start.sh
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'
```

没有 NVIDIA GPU 的笔记本可使用 CPU 覆盖：

```bash
docker compose -f docker-compose.yml -f docker-compose.cpu.yml up -d
```

## 5. 先做 mock，不接真实小车

mock 使用 `127.0.0.1:31002`，不会访问 `192.168.10.10`。

终端 A：

```bash
cd /home/w417/RTAB-Map
python3 water_chassis_sdk_v5_1_cn_complete/water_chassis_sdk_cn_v5_1/tools/mock_chassis_server.py \
  --host 127.0.0.1 --port 31002
```

终端 B：

```bash
cd /home/w417/RTAB-Map
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 launch rtabmap_tb3_nav water_chassis_bridge.launch.py \
   water_robot_host:=127.0.0.1 water_robot_port:=31002 water_gateway_port:=18082 \
   water_enable_motion:=true water_allow_provisional_odom:=true'
```

终端 C 检查：

```bash
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 topic info /cmd_vel_safe -v && ros2 topic echo /odom --once && \
   ros2 topic echo /water_chassis/status --once'
```

只有 mock 服务运行时，才用下面的命令验证速度链路：

```bash
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   timeout 2 ros2 topic pub --rate 10 /cmd_vel_safe geometry_msgs/msg/Twist \
   "{linear: {x: 0.08}, angular: {z: 0.0}}"'
```

停止发布后，mock 底盘应在 watchdog/bridge 超时保护下回到零速度。这个命令不能复制到
真实底盘现场。诊断模式下即使收到 `/cmd_vel_safe` 也不应发生 mock 位移，且 bridge
不应发布 `/odom`。

## 6. 真实 WATER 接入顺序

### 6.1 只读连接

确认底盘实际 IP 后先使用诊断模式：

```bash
docker compose exec ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   ros2 launch rtabmap_tb3_nav water_chassis_bridge.launch.py \
   water_robot_host:=<现场确认的底盘IP> water_robot_port:=31001 \
   water_gateway_port:=8080 water_enable_motion:=false \
   water_allow_provisional_odom:=false'
```

这一步只检查 Gateway、底盘连接和 `/water_chassis/status`，不发送速度命令。不要同时
启动厂家自动导航任务。

### 6.2 正式实车前的里程计

正式静态导航必须具备厂家或另行适配的编码器 `/odom`、`odom→base_link`，并确认
唯一的 `/cmd_vel_safe` 控制拥有者。当前 SDK 的暂定积分 `/odom` 只能用于 mock/
软件联调，不能写成正式实车轨迹、成功率或精度结果。没有正式 `/odom` 时，不得发送
Nav2 导航目标。

### 6.3 完整实车链

```text
Nav2 → /cmd_vel → collision_monitor → /cmd_vel_safe
      → 唯一底盘速度拥有者 → WATER 电机
```

如果 bridge 是唯一速度拥有者、且外部编码器驱动已经提供正式里程计：

```text
use_water_bridge:=true
water_enable_motion:=true
water_allow_provisional_odom:=false
```

只有软件联调才同时设置 `water_allow_provisional_odom:=true`。无论哪种模式，
`/cmd_vel` 都不能直接连接 WATER。

## 7. 迁移到台式机

迁移只更换 ROS 2 宿主机，不改变 v13 profile、`/cmd_vel_safe` 契约或 H0→H8 顺序：

1. 将仓库复制或克隆到台式机；
2. D435i 改接台式机 USB 3.0；
3. 台式机连接 WATER 同一网络并记录实际 IP；
4. 重建 Docker 镜像和 ROS 包；
5. 只在台式机启动一个 ROS 2 控制链；
6. 笔记本退出 ROS 2，或仅做 SSH/RViz 观察。

```bash
cd /home/w417/RTAB-Map
./scripts/build.sh
./scripts/start.sh
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install'
```

## 8. 本轮离线验证结果

2026-09-01 在没有真实 WATER 和 D435i 在线的情况下完成：

- Docker 镜像重建通过，WATER SDK Python 依赖安装通过；
- ROS 2 `colcon build --symlink-install` 通过；
- SDK `tools/self_test.py` 通过；
- mock bridge 成功建立状态链、`/cmd_vel_safe` 订阅、暂定 `/odom` 和 TF；
- mock 速度命令能够使 mock 位姿前进，停止发布后速度回到零；
- 诊断模式不会转发速度，也不会发布 bridge 的 `/odom`；
- 直接终止 bridge 节点后，SDK 自动启动的 Gateway 端口关闭且 Gateway 子进程清理；
- 真实底盘没有被连接、扫描或运动控制。

当前结论是“WATER SDK 已完成 ROS 2 软件桥接和 mock 验证”，不是“真实底盘已经完成
静态导航”。真实成功率、耗时、轨迹、contacts 和最小净空仍全部为“未测”。

## 9. 仍存在的边界

- 未确认厂家是否提供可直接使用的 ROS 2 编码器 `/odom` 驱动；
- 相机外参、车体 footprint、轮径/轮距和加速度仍须现场测量；
- bridge 暂定积分 `/odom` 不可用于论文最终实车指标；
- 真实设备没有 Gazebo contacts，碰撞证据必须来自急停/底盘事件、视频或 rosbag；
- 任何真实运动都必须由人工急停监护，并先完成 H0→H5；
- 实车仍使用 v13 规划基础，但首轮速度上限保持 `0.12 m/s`，不能直接套用仿真
  `0.28 m/s`。

相关入口：

- [实车现场操作手册](实车接入现场操作手册_2026-09-01.md)
- [阶段4 README](README.md)
- [WATER II-S 接口配置](../../src/rtabmap_tb3_nav/config/real_water_ii_s.yaml)
- [WATER ROS 2 bridge](../../src/rtabmap_tb3_nav/scripts/water_chassis_ros_bridge.py)
- [bridge 独立启动文件](../../src/rtabmap_tb3_nav/launch/water_chassis_bridge.launch.py)
