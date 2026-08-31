# 阶段 3：场景 03 动态障碍验证

## 1. 本阶段目的

在场景 02 的静态障碍布局上只增加一个环境因素：一个可检测、可重复往返的
Gazebo 动态障碍。场景 01、场景 02 原世界文件和正式结果不修改、不覆盖。

本阶段首先使用通用跨场景基线 `adaptive_goal_line_045`，验证当前
RGB-D 在线建图、实时 costmap、GoalLineSmacPlanner、RPP 控制器和
collision_monitor 对运动障碍的自然响应。暂不把场景 02 的
`adaptive_goal_line_050_recovery_v13_line_tiebreaker` 宣称为新场景默认值，
也不迁移 large 场景 v4 的固定走廊。

## 2. 世界与动态因素

新世界文件：

`src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_cross_scene_03_dynamic.world`

它由场景 02 世界复制而来，静态墙体、障碍物、起点和目标坐标保持不变，只增加：

| 项目 | 设置 |
| --- | --- |
| 模型名 | `dynamic_obstacle` |
| 几何体 | `0.8 m × 0.8 m × 0.8 m` 碰撞箱 |
| 初始位置 | `x=-4.0 m, y=-2.8 m, z=0.4 m` |
| 运动范围 | 固定 `x=-4.0 m`，`y=-2.8…-1.0 m` |
| 速度 | `0.18 m/s`，到边界反向 |
| 驱动 | `libgazebo_ros_planar_move.so` + `dynamic_obstacle_driver.py` |
| 传感器路径 | Gazebo RGB-D → `/camera/cloud`、`/camera/obstacles` |
| 安全路径 | `/camera/cloud` → `collision_monitor` |

运动障碍不向 Nav2 发布路线、地图或 waypoint。它只通过真实碰撞几何和深度观测
影响实时代价地图与控制安全层。

## 3. 证据要求

每个 smoke 或正式 run 都必须保存：

- `trajectory_comparison.png`、`trajectory.png`；
- 在线规划轨迹、Gazebo 真值轨迹和 `动态障碍轨迹.csv`；
- `metrics.yaml`、`多目标指标.yaml`；
- 有效运行时参数 `导航参数.yaml`、`碰撞监视参数.yaml`，以及源码参数快照；
- `实验参数.yaml`、`运行时元数据.yaml`；
- `世界文件.sdf`；
- `gazebo_contacts_raw.log.gz` 和 `gazebo_contacts_summary.yaml`。

正式成功仍要求全部目标段 `status=4`、无非地面 contacts 且证据完整。安全停止
即使没有碰撞，也记为失败/边界，不混入成功率。

## 4. 实验顺序

1. 单段 `M→N` smoke test；
2. smoke 通过后，独立重启仿真并做三次 `M→N→X→Y→M` 正式回归；
3. 逐次记录动态障碍的实际轨迹、导航轨迹、耗时、末端误差、contacts 和失败原因；
4. 若基线失败，先分析是观测、地图残影、重规划时序、控制/安全停止还是物理接触，
   再决定是否提出一个新的通用 profile。不得直接把失败归因于“参数不够”并盲目调参。

## 5. 当前状态

| 项目 | 状态 |
| --- | --- |
| 场景 03 世界文件 | 已创建，待 smoke 验证 |
| 动态模型与驱动 | 已实现并通过启动日志验证 |
| 归档链路 | 已增加有效参数、动态障碍轨迹和 contacts 压缩保存 |
| smoke | 待执行 |
| 三次正式回归 | 待 smoke 通过后执行 |

场景 02 v13 正式结果仍独立保存在
`results/05_跨场景验证/场景02/01_正式验收/`，本阶段不改写它。
