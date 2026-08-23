# 项目进度总结

更新时间：2026-08-23（cross_scene_02 M→N 基线与四点闭环 v5 三次验证完成）

项目演进和阶段复现总览：[项目演进与阶段复现总览_2026-08-22.md](../00_项目总览/项目演进与阶段复现总览_2026-08-22.md)
；正式结果索引：[EXPERIMENT_ARCHIVE_INDEX.md](../00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。

## 最新进度：cross_scene_02 规范化世界 M→N 复测

### 最新验收结果：原参数基线与四点闭环恢复增强

在不改变障碍物布局的前提下，先使用原参数 `adaptive_goal_line_045` 独立复测
`M(-8.5,0) -> N(8.5,0)` 三次，三次均为 `status=4`，wall 平均
`141.745 ± 12.048 s`，Gazebo 轨迹平均 `25.862 m`，非地面 contacts 为 `0/3`。
报告见
[阶段2_场景02_MN原参数基线三次复测报告_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_MN原参数基线三次复测报告_2026-08-23.md)。

随后针对四点闭环中的偶发安全停止、局部死角和恢复不足，保留实时 RGB-D 规划拓扑，
新增 `adaptive_goal_line_050_recovery_v5`，完成
`M -> N -> X -> Y -> M` 三次独立实验。结果为 `3/3` 完整 run、`12/12` 分段
`status=4`、非地面 contacts `0/3`，总 wall 平均 `545.459 ± 26.327 s`，
Gazebo 轨迹平均 `100.885 m`。报告、轨迹索引和参数手册见
[阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md)、
[阶段2_场景02_四点闭环恢复增强轨迹图索引_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_四点闭环恢复增强轨迹图索引_2026-08-23.md) 和
[阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md)。

本次 v5 没有加入场景 02 的固定路径或障碍 waypoint；`side_bias` 和固定世界坐标走廊
均关闭，四个目标由脚本逐段发送，规划器每段从当前实时 costmap 重新规划。v5 是
场景 02 的验收 profile，不替换默认 `adaptive_goal_line_045`。

用户调整后的 `indoor_obstacle_course_cross_scene_02.world` 已完成运行快照规范化：
将静态模型实际位姿提升到顶层，删除 `<state>` 和内嵌 `waffle`，由启动脚本生成唯一
机器人。保持 `adaptive_goal_line_045`、`inflation_radius=0.45 m` 和在线 RGB-D
建图不变，重新执行 M(-8.5,0) -> N(8.5,0) 三次。

本轮目标只通过 `NavigateToPose` 发送，未回放固定 waypoint；但该 profile 仍包含旧
基准场景的 `side_bias_*` 世界坐标软先验，所以当前结论是“在线规划跨场景复测”，
还不是完全无场景先验的未知环境泛化验证。

结果为 `2/3` 成功、`1/3` 失败，三次非地面 Gazebo contacts 均为 0：

| run | Nav2 状态 | wall [s] | Gazebo 轨迹 [m] | 末端误差 [m] | 结论 |
| --- | ---: | ---: | ---: | ---: | --- |
| run_01 | 4 | 139.961 | 26.517 | 0.150 | 成功 |
| run_02 | 4 | 142.932 | 26.409 | 0.128 | 成功 |
| run_03 | 6 | 62.696 | 2.927 | 14.594 | 安全停止，未碰撞 |

规范化复测报告：[阶段2_场景02_MN规范化世界三次复测报告_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_MN规范化世界三次复测报告_2026-08-22.md)

规范化场景目录：[场景02_MN规范化世界_2026-08-22](../../results/05_跨场景验证/场景02/场景02_MN规范化世界_2026-08-22)

规范化旧复测的 `run_03` 在西侧第一道障碍前触发 RPP collision projection 和
controller patience 超时；该失败证据仍保留。之后的原参数三次复测未重现该失败，
四点闭环则经过 v1～v4 排查后由 v5 通过，失败分类和改动边界见
[阶段2_场景02_失败版本与恢复优化分析_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_失败版本与恢复优化分析_2026-08-23.md)。

## 历史进度：cross_scene_01 跨场景九次验证

在用户手动确认障碍物布局后的
`indoor_obstacle_course_cross_scene_01.world` 上，保持
`adaptive_goal_line_045`、在线 RGB-D 建图和 `inflation_radius=0.45 m`，完成三组
五点闭环顺序、每组 3 次，共 `9/9` run 成功、`45/45` 目标段 `status=4`，Gazebo
contacts 过滤 `ground_plane` 后为 `0/9` 非地面接触。新场景的障碍物位置没有被本轮
启动命令改动；仅将用户保存的机器人状态清理为由 `x_pose/y_pose` 统一 spawn，避免
重复 `waffle`。

| 顺序 | 平均 wall [s] | 平均轨迹 [m] | run 成功率 | 分段成功率 | 非地面接触 |
| --- | ---: | ---: | ---: | ---: | ---: |
| M-A-B-C-D-M | 292.229 +/- 6.826 | 50.557 +/- 0.408 | 3/3 | 15/15 | 0/3 |
| C-A-D-B-M-C | 288.647 +/- 2.752 | 49.297 +/- 0.110 | 3/3 | 15/15 | 0/3 |
| B-M-A-C-D-B | 342.412 +/- 7.375 | 62.252 +/- 1.464 | 3/3 | 15/15 | 0/3 |

正式报告：[阶段2_跨场景场景01九次实验总结_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_跨场景场景01九次实验总结_2026-08-22.md)

参数和复现命令：[阶段2_跨场景场景01复现参数_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_跨场景场景01复现参数_2026-08-22.md)

机器可读表：[阶段2_场景01九次实验汇总.csv](../08_下一阶段实验归档_2026-08-22/阶段2_场景01九次实验汇总.csv)

这次结果证明的是“当前算法在一个障碍布局变化后的静态仿真场景可以在线建图和实时
逐段导航”，不是任意未知环境的 100% 保证。下一步才是 cross_scene_02/03、动态
障碍物和真实 D435i。

## 结果归档整理（2026-08-22）

results/ 已按实验类型整理为 5 个分类目录，当前保留 13 个实验集合和 37 个
metrics.yaml。场景 02 的原始快照与规范化复测使用不同目录，没有删除或覆盖任何正式
运行证据。

详细目录映射和下一场景建议见
[阶段2结果归档整理与下一阶段计划_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2结果归档整理与下一阶段计划_2026-08-22.md)，
物理结果入口见 [results/README.md](../../results/README.md)。

## 当前场景内历史顺序验证：原 large 场景

在保持 `adaptive_goal_line_045`、`inflation_radius=0.45 m` 和
`indoor_obstacle_course_large.world` 不变的条件下，已完成三组五点闭环：

```text
原始：M -> A -> B -> C -> D -> M
实验1：C -> A -> D -> B -> M -> C
实验2：B -> M -> A -> C -> D -> B
```

三组各运行三次，合计 `9/9` 完整 run 成功、`45/45` 目标段 `status=4`，Gazebo
contacts 过滤 `ground_plane` 后均无非地面接触。新结果目录为：

- `results/04_自适应目标线多目标/自适应目标线_顺序实验1_CADBMC_2026-08-22/run_01..run_03/`
- `results/04_自适应目标线多目标/自适应目标线_顺序实验2_BMACDB_2026-08-22/run_01..run_03/`

每个 run 均保存左右合成 `trajectory_comparison.png`、在线地图轨迹、Gazebo 真值
轨迹、分段 CSV、参数快照、世界文件和 contacts 统计；结果不会覆盖此前五点闭环。

统一数据分析：[点位顺序跨实验统一分析_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/点位顺序跨实验统一分析_2026-08-22.md)

实验 1 报告：[自适应目标线顺序实验1报告_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/自适应目标线顺序实验1报告_2026-08-22.md)

实验 2 报告：[自适应目标线顺序实验2报告_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/自适应目标线顺序实验2报告_2026-08-22.md)
最终验证和复现入口：[当前场景算法最终验证总结_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/当前场景算法最终验证总结_2026-08-22.md)

三组平均指标：

| 顺序 | wall [s] | 总轨迹 [m] | run 成功率 | 分段成功率 | 非地面接触 |
| --- | ---: | ---: | ---: | ---: | ---: |
| M-A-B-C-D-M | 288.563 +/- 7.516 | 51.919 +/- 0.877 | 3/3 | 15/15 | 0/3 |
| C-A-D-B-M-C | 296.649 +/- 2.088 | 49.714 +/- 0.041 | 3/3 | 15/15 | 0/3 |
| B-M-A-C-D-B | 325.836 +/- 2.538 | 58.005 +/- 0.023 | 3/3 | 15/15 | 0/3 |

准确边界：这些结果证明当前 profile 在当前 Gazebo 大场景中可以在线 RGB-D 建图、
改变起点和目标顺序并实时逐段导航；不代表任意未知场景都能保证成功。下一阶段应
按 [跨场景仿真修改与恢复指南_2026-08-22.md](../07_跨场景与真实设备/跨场景仿真修改与恢复指南_2026-08-22.md)
只改变一个环境因素并重新做三次验证。

## 最新进度（2026-08-22）

已先在不改变 `adaptive_goal_line_045` 的条件下完成双目标三次基线验证，三次均为
`2/2` 成功、非地面 contacts 为 `0/3`，随后完成五点闭环三次正式实验：

```text
M(-8.5,0) -> A(5,-3) -> B(5,6) -> C(-5,4) -> D(0,0) -> M(-8.5,0)
```

五点结果为 `3/3` run 成功、`15/15` 分段成功、`0/3` 非地面物理接触；总 wall
时间 `288.563 +/- 7.516 s`，总轨迹长度 `51.919 +/- 0.877 m`。三次左右合成图、
CSV、参数快照和 contacts 记录已保存在
`results/04_自适应目标线多目标/自适应目标线_五点闭环_2026-08-22/run_01..run_03/`。

本阶段冻结文件为
[五点闭环最终参数_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/五点闭环最终参数_2026-08-22.md)，实验原始表为
[五点闭环导航实验记录_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/五点闭环导航实验记录_2026-08-22.md)，阶段
总结为 [多目标导航阶段总结_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/多目标导航阶段总结_2026-08-22.md)。
当前结论是“当前 Gazebo 场景中的 RGB-D 在线多目标导航基线已通过”，下一步是跨场景
验证，再进入真实 D435i；不能把这三次结果直接表述为任意未知环境的保证。

## 当前结论（最新）

当前 benchmark 推荐 profile 是 `fast_goalline_045_v4`：三次干净 A -> B 均为
`status=4`，Gazebo contacts 过滤地面后均无非地面接触，平均 wall
`81.07 ± 3.46 s`，平均 Gazebo 路径 `17.382 ± 0.050 m`。它是 large 场景冻结 benchmark；
`demo.launch.py` 当前默认值已切换为不依赖固定场景走廊的 `adaptive_goal_line_045`。
需要复现 v4 时仍显式指定 profile。

v4 的三次双视图和完整指标见
[NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md](../04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md)，
冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md](../04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md)。
结果目录、删除范围和历史恢复规则见
[EXPERIMENT_ARCHIVE_INDEX.md](../00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。
v4 的复现命令、轨迹覆盖规则和跨场景验证计划见
[V4_REPRODUCTION_AND_CROSS_SCENE_VALIDATION_2026-08-21.md](../07_跨场景与真实设备/V4_REPRODUCTION_AND_CROSS_SCENE_VALIDATION_2026-08-21.md)。

本轮新增通用目标 profile `adaptive_goal_line_045`，并将 `demo.launch.py` 默认值切换到
该 profile。它关闭 v4 的 `side_bias_target_schedule`、固定 `world_x/world_y` 走廊和
场景方向提示，仅保留按每次规划调用重新计算的当前起点到当前目标直线软偏好。连续
`(-8.5,0.0) -> A(5.0,-3.0) -> B(5.0,6.0)` 回归两段均为 `status=4`，总 wall
`136.94 s`，Gazebo contacts 过滤地面后无非地面接触。详细记录见
[未知目标实时规划优化说明_2026-08-21.md](../05_自适应目标线_2026-08-21至22/未知目标实时规划优化说明_2026-08-21.md)、
[自适应目标线多目标实验记录_2026-08-21.md](../05_自适应目标线_2026-08-21至22/自适应目标线多目标实验记录_2026-08-21.md) 和
[自适应目标线参数记录_2026-08-21.md](../05_自适应目标线_2026-08-21至22/自适应目标线参数记录_2026-08-21.md)。

本轮新增代码提交 `452b45f` 和候选 profile `fast_goalline_045_v2`。它在三次干净
A -> B 中达到 `88.95 ± 3.57 s`、Gazebo 路径 `17.31 ± 0.14 m`、3/3 成功、0/3
非地面 contacts；两次最大 `y` 约 `0.79 m`，一次 `1.37 m`。因此 v2 是速度候选，
没有覆盖默认 profile。详细实验表和双视图见
[NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V2.md](../04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V2.md)。

候选冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V2_2026-08-21.md](../04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V2_2026-08-21.md)。

v3 仍作为路线方差较小的历史对照，v2 仍作为更早的速度对照；它们没有被覆盖。

详细实验表和双视图：[NAVIGATION_OPTIMIZATION_2026-08-21.md](../04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21.md)

冻结参数：[FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-21.md](../04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-21.md)

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
| 贴边路径优化 | v4 已完成三次正式回归并冻结 | `GoalLineSmacPlanner`，inflation `0.45 m`；v4 为 `0.30 m/s`、分段走廊、cost scaling `4.5`，不改硬 footprint/stop |
| 任意新目标重新规划 | 已完成首轮双目标回归 | `adaptive_goal_line_045` 关闭固定场景走廊，按当前 start/goal + live costmap 重规划 |
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
- 最新 A -> B、B -> A 都返回 Nav2 状态 `4`；trial 最后 TF/odom 采样误差分别为
  `0.152 m` 和 `0.208 m`，该采样值是诊断指标，不等同于 Nav2 goal checker 的内部判定值。
- A -> B / B -> A 的 Gazebo contacts 过滤均未发现机器人与墙、栏杆、箱体或柱体
  的非地面接触。
- 这是在一个设计好的可通行障碍场景中的功能闭环证明，不代表任意未知房间、任意
  狭窄通道或真实机器人已经达到工程级零碰撞。

因此，准确的结论是：点到点避障导航在当前仿真验证场景中已经实现；泛化到未知布局、
复杂遮挡、窄通道和真实 D435i 的部分仍属于下一阶段。

## 历史 Smac + RPP clearance-first 阶段（0.55 对照）

旧版 `NavFn + DWB` 的问题不是没有全局/局部规划，而是全局路径可能贴着膨胀层边缘，
DWB 又会较强地追随这条路径；在线模式中直接订阅增长中的 `/map` 也会让 StaticLayer
反复 resize，放大“前方停下来重新规划”的现象。当前源码已经改为：

- 在线模式使用固定 `24 m x 17 m` `/nav_map`，由 `map_padder.py` 从 RTAB-Map `/map`
  更新；全局 costmap 保留 `StaticLayer`，但不再直接订阅增长中的原始地图；
- `SmacPlanner2D` + `cost_travel_multiplier=6.0`，让高代价边缘路径在累计代价上
  更不占优；
- Smac 内置 path smoother，给控制器更连续的路径方向；
- `RegulatedPurePursuitController` 当前 v3 使用 `0.56--1.15 m` 自适应前视，按曲率、cost
  和前向碰撞预测调速，提前在障碍拐角前转向；普通弯道优先连续跟踪，终点大角度
  误差仍允许原地对齐；
- 稳定行为树每 `2 s` 检查路径，路径有效时约 `20 s` 才重算，目标改变或路径失效时
  立即重规划；
- 该历史 clearance-first 对照使用全局/局部 `inflation_radius=0.55 m`、
  `cost_scaling_factor=3.0`；`0.30 m` 对约 `0.27 m` 内切半径的 Waffle 只剩约 `3 cm`
  软梯度，容易让路径贴边；
  `footprint=0.60 x 0.48 m` 和 padding `0.03 m` 仍是安全边界。

这不是把“当前位置到终点直线”硬编码为最高分，而是由 Smac 的目标启发式保持目标
方向偏好、由 costmap 累计代价惩罚贴障碍路径、由 RPP 前视实现连续绕行。详细参数和
手动调整步骤见 [PARAMETERS.md](../01_基础环境与问题分析/PARAMETERS.md)，阶段结果归档见
[EXPERIMENT_ARCHIVE_INDEX.md](../00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)，目标线阶段记录见
[NAVIGATION_OPTIMIZATION_2026-08-20.md](../03_目标线规划优化_2026-08-20/NAVIGATION_OPTIMIZATION_2026-08-20.md)。

当前新配置已完成 A -> B / B -> A 独立回归；旧 DWB 的结果只能作为历史对照，不能
直接充当 Smac + RPP 的论文数据。

## 仿真如何运行

要同时看到 Gazebo 和 RViz2，使用以下顺序：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false reset_db:=true navigation_profile:=adaptive_goal_line_045'
```

保持第二条命令的终端打开，再在另一个终端发送 A -> B 目标：

```bash
cd /home/w417/RTAB-Map
sg docker -c 'docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /workspaces/rtabmap_tb3_nav/install/setup.bash && ros2 run rtabmap_tb3_nav send_goal.py --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5"'
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
  -> SmacPlanner2D 全局规划 + RPP 局部控制
  -> /cmd_vel
  -> collision_monitor 安全过滤
  -> /cmd_vel_safe
```

`/scan` 没有发布者，因此当前项目不是 LiDAR 导航。RViz2 只是可视化和发送目标，
真正完成规划的是 RTAB-Map、Nav2 costmap、SmacPlanner2D、RPP 和 collision monitor
的组合。

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

### 旧版 DWB 双向实验（历史对照）

| 方向 | Nav2 状态 | action 墙钟时间 | 仿真时间 | 末端 XY 误差 | 轨迹样本 | contacts / 障碍接触 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A -> B | `4` | `182.87 s` | `182.87 s` | `0.108 m` | `900` | `464223` / `(none)` |
| B -> A | `4` | `161.07 s` | `161.07 s` | `0.111 m` | `723` | `424417` / `(none)` |

这组早期轨迹和 contacts 证据已按 [EXPERIMENT_ARCHIVE_INDEX.md](../00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)
归档；原始目录可从整理前提交恢复。A→B 及 B→A 的 contacts 都过滤了 `ground_plane`；
最新完整监听分别记录 464,223 和 424,417 条 contact
消息，因此表中的 `(none)` 指没有 `waffle` 与墙、栏杆、箱体或柱体的接触对。

这两次结果可作为当前论文的仿真 RGB-D 在线建图 + Nav2 DWB 历史对照：成功率在这两次
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
| A -> B（旧 DWB vtheta=7，无 GUI） | 通过 | status `4`，末端距离约 `0.10 m`；约 `346295` 条 contacts 过滤后无障碍接触 |
| B -> A（旧 DWB vtheta=7，无 GUI） | 通过 | status `4`，末端距离约 `0.12 m`；约 `373473` 条 contacts 过滤后无障碍接触 |
| A -> B（旧在线参数，仅 enabled=false） | 通过但有 resize | status `4`，末端距离约 `0.10--0.12 m`；约 `351460` 条 contacts 过滤后无障碍接触；日志仍有 StaticLayer resize |
| B -> A（旧在线参数，仅 enabled=false） | 通过但有 resize | status `4`，末端距离约 `0.12 m`；约 `434034` 条 contacts 过滤后无障碍接触；日志仍有 StaticLayer resize |
| B -> A（修复后移除 StaticLayer） | 通过 | status `4`，末端距离约 `0.12 m`；约 `342705` 条 contacts 过滤后无障碍接触 |
| A -> B（修复后移除 StaticLayer） | 通过 | status `4`，末端距离约 `0.12 m`；约 `358524` 条 contacts 过滤后无障碍接触 |
| A -> B（Smac + RPP clearance-first） | 通过 | status `4`，`114.52 s`；trial 最后采样误差 `0.152 m`；`280327` 条 contacts 过滤后无障碍接触 |
| B -> A（Smac + RPP clearance-first） | 通过 | status `4`，`110.82 s`；trial 最后采样误差 `0.208 m`；`285301` 条 contacts 过滤后无障碍接触 |
| RGB-D `/camera/cloud` | 通过 | 实测约 `12.0--12.4 Hz` |
| Nav2 `/camera/obstacles` | 通过 | 实测约 `12.0--12.4 Hz`，PointCloud2 有发布者和订阅者 |
| Nav2 生命周期 | 通过 | `/controller_server`、`/planner_server` 均为 `active [3]` |
| 无 LiDAR | 通过 | `/scan` 没有发布者 |
| 安全层响应 | 通过 | 最新日志出现 slowdown，历史回归出现 stop；说明点云进入安全层 |
| 点云时间戳 | 通过 | 本轮 collision monitor 日志没有过期/时间戳差异警告 |
| Gazebo 障碍接触 | A -> B / B -> A 通过 | 最终完整监听 A -> B `464223` 条、B -> A `424417` 条 contacts 记录中过滤 `ground_plane` 后，均没有 `waffle` 与 `wall/barrier/crate/pillar` 的接触对 |

这次回归证明当前 A/B 路线能够在线建图、在线规划、在线避障并到达目标，且两个
方向都通过了 Gazebo 物理接触过滤。Nav2 `status=4` 是本轮成功标准；trial 最后
采样误差需要结合 TF/odom 时间对齐理解，不能替代 action 内部 goal checker。它不等于
任意障碍布局都已经达到零碰撞：RGB-D 仍有视野、遮挡、反光和掉帧限制。

此前失败样本中曾记录若干次 `Failed to make progress` 和少量 deadline 提示。复核
`/cmd_vel` 后确认主要死锁点是旧版 `vtheta_samples=20` 产生约 `+/-0.0395 rad/s`
的微小角速度，机器人在反向目标或障碍拐角处左右摆动；`min_speed_theta` 本身没有
改变该采样集合。旧 DWB 对照使用 `vtheta_samples=7` 和 `inflation_radius=0.50 m`
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
- 在线模式使用 `map_padder.py` 将 `/map` 复制到固定 `24 m x 17 m` 的 `/nav_map`，
  全局 costmap 保留 `StaticLayer` 读取 `/nav_map`；定位模式读取已保存的 `/map`，
  避免 Humble 因原始地图扩展反复 resize。

`online:=false` 只能与 `localization:=true` 配合，用于已有数据库定位。

### Nav2 参数

`src/rtabmap_tb3_nav/config/nav2_rgbd_params.yaml` 现在：

- 全局 costmap 为固定 `24 m x 17 m` 地图包络，分辨率 `0.05 m`；
- `allow_unknown: true`，允许在线地图增长期间规划；
- 局部 costmap 为 6 m x 5 m，10 Hz 更新；controller 为 15 Hz；
- `SmacPlanner2D` 使用 `cost_travel_multiplier=6.0`，并启用内置路径平滑；
- `RegulatedPurePursuitController` 当前使用 `desired_linear_vel=0.26 m/s`、前视距离
  `0.56--1.15 m`，启用曲率/代价调速和前向碰撞预测；
- 使用矩形 footprint `0.60 m x 0.48 m`，padding `0.03 m`；
- inflation radius `0.45 m`、`cost_scaling_factor=3.0`；这是仿真路径偏好参数，不是固定的真实机器人安全值；
- v3 线速度上限 `0.26 m/s`，角速度上限 `0.85 rad/s`；
- `SimpleProgressChecker` 使用 `0.20 m / 20 s`，允许机器人在拐角减速或反向目标的初始
  转向时继续
  获得进度；
- controller 频率为 `15 Hz`；规划器期望频率为 `2 Hz`；稳定行为树每 `2 s` 检查路径，
  有效路径约 `20 s` 才重算；
- 局部/全局障碍观测持续时间约 `0.4--0.5 s`。

### 碰撞监视器和实时性

`src/rtabmap_tb3_nav/config/collision_monitor_rgbd_params.yaml`：

- 输入速度：`/cmd_vel`；
- 输出速度：`/cmd_vel_safe`；
- RGB-D 输入：使用 `point_cloud_xyz` 降采样后的 `/camera/cloud`；原始
  `/camera/points` 仅用于诊断，Nav2 costmap 继续使用 `/camera/obstacles`；
- 前方约 `0.38 m` stop polygon；硬停止区贴近车体，避免在拐角还没完成转向时提前
  锁死；
- 前方约 `1.05 m` slowdown polygon，速度降到 `75%`；
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
| 目标判定 | Nav2 action status `4`；配置的 goal checker 为 XY `0.12 m`、yaw `0.15 rad` |

本轮 clearance-first 配置已完成无 GUI 的 A -> B / B -> A Gazebo contacts 过滤；
当前在线 `plugins` 保留 `static_layer`，但读取固定 `/nav_map`，而不是直接读取增长中的
`/map`。后续修改相机、
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

## 当前连续走廊优化回归（0.45）

v3 三次 A -> B 均为 `status=4`，wall `90.238 / 91.352 / 91.659 s`，平均
`91.083 ± 0.61 s`，平均 Gazebo 轨迹 `17.492 m`，最大 y 偏移 `1.388 ± 0.044 m`，
contacts 过滤地面后均为 none。详细参数、每次轨迹双视图和 v2 对照见
[NAVIGATION_OPTIMIZATION_2026-08-21.md](../04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21.md)。

v2 三次结果仍保留为对照：平均 `95.541 ± 2.03 s`，平均 Gazebo 轨迹 `17.765 m`，
最大 y 偏移 `1.588 ± 0.422 m`，3/3 成功、0/3 非地面碰撞。

当前分支可直接用 `navigation_profile:=fast_north_045_v2` 恢复 v2 的参数覆盖；如果
要求代码级精确复现，应按结果目录 `experiment.yaml` 的 `git_commit` 建立 worktree，
不要用新版本二进制冒充旧提交。v3 只新增 launch profile，未改动 C++ planner 算法。

## 2026-08-20 六次参数基准（历史对照）

本轮严格按固定规则完成了同一大场景、同一 A -> B 目标、每组 3 次干净重启的对比：

- `inflation_radius=0.55 m`：3/3 成功，墙钟 `116.425 +/- 1.569 s`，平均近似净空 `0.0711 m`，无非地面 Gazebo 接触；
- `inflation_radius=0.45 m`：3/3 成功，墙钟 `114.913 +/- 0.793 s`，平均近似净空 `0.0146 m`，无非地面 Gazebo 接触；
- 当时的 clearance-first 冻结值为 `0.55 m`；后续依据速度优先规则冻结 0.45，
  并在此基础上验证目标线优化候选。六次结果仍作为历史对照保留。

每次实验均保存了 `metrics.yaml`、map/Gazebo 两条轨迹 CSV、单图、左右双视图、参数快照、世界文件、contacts 摘要和 `experiment.yaml`。详细表格见
[BENCHMARK_2026-08-20_SUMMARY.md](../02_原生规划基准_2026-08-20/BENCHMARK_2026-08-20_SUMMARY.md)，冻结参数与复现命令见
[FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md](../02_原生规划基准_2026-08-20/FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md)。

历史冻结配置提交：`be1dabe`；0.45 对照切换提交：`a1389ff`；目标线候选提交：
`c18894e`；固定西侧窗口 v2 提交：`9823820`；当前连续走廊 v3 提交：`6202912`。
后续参数实验应从当前冻结版本创建新提交，一次只改变一个变量，并保持同样的 3 次回归
与物理 contacts 证据。
