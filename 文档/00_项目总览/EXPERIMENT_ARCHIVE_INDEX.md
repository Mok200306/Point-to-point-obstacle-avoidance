# 实验归档索引

更新时间：2026-08-23

本文是结果目录的唯一整理入口。当前工作树只保留每个正式阶段最有代表性的三次
完整回归；pilot、失败检查、重复单次结果和被后续阶段替代的原始目录不再占用仓库
空间。被 Git 跟踪的移除结果仍可从整理前的提交 `873fd39` 恢复；原先就被 `.gitignore`
忽略的临时结果没有进入 Git，删除后不再有本地副本。

结果物理目录已经按实验类型分为 01_原生规划基准、02_目标线规划优化、
03_V4快速目标线、04_自适应目标线多目标 和 05_跨场景验证，具体目录和保存规则
见 results/README.md。

项目从基础链路到五点闭环的技术演进见
[项目演进与阶段复现总览_2026-08-22.md](../00_项目总览/项目演进与阶段复现总览_2026-08-22.md)。

## 当前保留结果

### 阶段 1：原生 Smac + RPP 基准（2026-08-20）

保留 0.45 m 组，因为它在无非地面物理接触的前提下平均墙钟时间更短，符合本项目
“无碰撞是硬门槛，满足后时间越短越好”的比较规则。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`smac_rpp_045_A_to_B_run_01`](../../results/01_原生规划基准/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_01) | 115.83 | 18.187 | 0.264 | none |
| 02 | [`smac_rpp_045_A_to_B_run_02`](../../results/01_原生规划基准/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_02) | 113.90 | 18.159 | 0.183 | none |
| 03 | [`smac_rpp_045_A_to_B_run_03`](../../results/01_原生规划基准/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_03) | 115.01 | 18.174 | 0.365 | none |
| 平均 | 3/3 成功 | **114.91** | **18.173** | **0.271** | **0/3** |

代码快照：`a1389ff`；参数和实验条件见
[BENCHMARK_2026-08-20_SUMMARY.md](../02_原生规划基准_2026-08-20/BENCHMARK_2026-08-20_SUMMARY.md) 与
[FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md](../02_原生规划基准_2026-08-20/FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md)。

### 阶段 2：目标线四点软偏好（2026-08-20）

保留 `GoalLineSmacPlanner` 的三次正式回归。这一阶段验证了全局规划仍受 costmap
碰撞约束，同时对当前位置到目标的直线增加软偏好，障碍物后能向目标线回收。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`goal_line_quad_045_A_to_B_run_01`](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_01) | 113.90 | 18.312 | 0.156 | none |
| 02 | [`goal_line_quad_045_A_to_B_run_02`](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_02) | 116.72 | 18.280 | 0.170 | none |
| 03 | [`goal_line_quad_045_A_to_B_run_03`](../../results/02_目标线规划优化/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_03) | 110.28 | 17.339 | 0.160 | none |
| 平均 | 3/3 成功 | **113.63** | **17.977** | **0.162** | **0/3** |

代码快照：`c18894e`；阶段报告见
[NAVIGATION_OPTIMIZATION_2026-08-20.md](../03_目标线规划优化_2026-08-20/NAVIGATION_OPTIMIZATION_2026-08-20.md)，冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md](../03_目标线规划优化_2026-08-20/FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md)。

### 阶段 3：快速分段目标线 v4（2026-08-21）

这是当前仿真场景的推荐 profile。保留三次完整结果，三次均低于 100 s，且保持
无非地面 Gazebo 接触。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`fast_goalline_045_v4_A_to_B_run_01`](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01) | 79.01 | 17.336 | 0.361 | none |
| 02 | [`fast_goalline_045_v4_A_to_B_run_02`](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_02) | 78.26 | 17.451 | 0.234 | none |
| 03 | [`fast_goalline_045_v4_A_to_B_run_03`](../../results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_03) | 85.94 | 17.359 | 0.274 | none |
| 平均 | 3/3 成功 | **81.07 +/- 3.46** | **17.382 +/- 0.050** | **0.290 +/- 0.053** | **0/3** |

运行时 profile 提交：`452b45f`；最终可复现提交：`78bb860`，当前文档提交为其后的
`873fd39`。详细报告见
[NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md](../04_快速目标线v4_2026-08-21/NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md)，
冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md](../04_快速目标线v4_2026-08-21/FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md)。

### 阶段 4：自适应目标线双目标基线（2026-08-21）

保持 `adaptive_goal_line_045` 不变，连续执行 M->A->B 三次，确认新目标会触发新的
规划调用。三次均为 `2/2` 成功，且过滤地面后没有非地面 contacts。正式三次结果目录为
`results/04_自适应目标线多目标/自适应目标线_当前基线三次验证_2026-08-22/run_01..run_03`；早期的修正版单次
目录仍作为历史代表保留，避免混淆正式三次统计与首轮单次证据。三次汇总、证据边界和
缺失字段说明见
[自适应目标线三次基线验证报告_2026-08-22.md](../05_自适应目标线_2026-08-21至22/自适应目标线三次基线验证报告_2026-08-22.md)。

| 运行 | wall [s] | 轨迹 [m] | 分段成功 | contacts | 非地面接触 |
| --- | ---: | ---: | ---: | ---: | --- |
| 01 | 144.10 | 26.312 | 2/2 | 351,739 | none |
| 02 | 132.77 | 25.483 | 2/2 | 324,207 | none |
| 03 | 136.05 | 25.542 | 2/2 | 322,923 | none |
| 平均 | **137.64** | **25.779** | **6/6** | - | **0/3** |

### 阶段 5：自适应目标线五点闭环（2026-08-22）

这是当前 `adaptive_goal_line_045` 在本场景的多目标闭环基线。保留三次完整结果，
每次包含双视角轨迹图、分段 CSV、参数快照、世界文件和完整 contacts 统计。

| 运行 | 结果目录 | wall [s] | 总轨迹 [m] | 分段成功 | 末端误差范围 [m] | 非地面接触 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 01 | [`run_01`](../../results/04_自适应目标线多目标/自适应目标线_五点闭环_2026-08-22/run_01) | 283.769 | 51.128 | 5/5 | 0.100--0.221 | none |
| 02 | [`run_02`](../../results/04_自适应目标线多目标/自适应目标线_五点闭环_2026-08-22/run_02) | 282.744 | 51.488 | 5/5 | 0.097--0.246 | none |
| 03 | [`run_03`](../../results/04_自适应目标线多目标/自适应目标线_五点闭环_2026-08-22/run_03) | 299.175 | 53.142 | 5/5 | 0.130--0.231 | none |
| 平均 +/- 标准差 | - | **288.563 +/- 7.516** | **51.919 +/- 0.877** | **15/15** | **0.097--0.246** | **0/3** |

完整实验表和图像见
[五点闭环导航实验记录_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/五点闭环导航实验记录_2026-08-22.md)，冻结
参数见 [五点闭环最终参数_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/五点闭环最终参数_2026-08-22.md)。

### 阶段 6：顺序实验 1，C -> A -> D -> B -> M -> C（2026-08-22）

保持 `adaptive_goal_line_045` 不变，将 Gazebo 起点改为 C，并改变目标顺序。三次
均为 `5/5` 成功，过滤地面后没有非地面接触。

| 运行 | 结果目录 | wall [s] | 总轨迹 [m] | 分段成功 | 非地面接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`run_01`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验1_CADBMC_2026-08-22/run_01) | 293.795 | 49.771 | 5/5 | none |
| 02 | [`run_02`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验1_CADBMC_2026-08-22/run_02) | 298.734 | 49.681 | 5/5 | none |
| 03 | [`run_03`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验1_CADBMC_2026-08-22/run_03) | 297.418 | 49.689 | 5/5 | none |
| 平均 +/- 标准差 | - | **296.649 +/- 2.088** | **49.714 +/- 0.041** | **15/15** | **0/3** |

完整分析和三张左右合成图见
[自适应目标线顺序实验1报告_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/自适应目标线顺序实验1报告_2026-08-22.md)。

### 阶段 7：顺序实验 2，B -> M -> A -> C -> D -> B（2026-08-22）

保持同一 profile，将 Gazebo 起点改为 B，并执行第二种闭环顺序。三次均为 `5/5`
成功，过滤地面后没有非地面接触。

| 运行 | 结果目录 | wall [s] | 总轨迹 [m] | 分段成功 | 非地面接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`run_01`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验2_BMACDB_2026-08-22/run_01) | 328.891 | 58.034 | 5/5 | none |
| 02 | [`run_02`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验2_BMACDB_2026-08-22/run_02) | 325.941 | 57.978 | 5/5 | none |
| 03 | [`run_03`](../../results/04_自适应目标线多目标/自适应目标线_顺序实验2_BMACDB_2026-08-22/run_03) | 322.676 | 58.003 | 5/5 | none |
| 平均 +/- 标准差 | - | **325.836 +/- 2.538** | **58.005 +/- 0.023** | **15/15** | **0/3** |

完整分析和三张左右合成图见
[自适应目标线顺序实验2报告_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/自适应目标线顺序实验2报告_2026-08-22.md)。

三种五点闭环的统一对比见
[点位顺序跨实验统一分析_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/点位顺序跨实验统一分析_2026-08-22.md)，当前场景最终验收和复现入口见
[当前场景算法最终验证总结_2026-08-22.md](../06_五点闭环与顺序验证_2026-08-22/当前场景算法最终验证总结_2026-08-22.md)。

### 阶段 8：cross_scene_01 跨场景九次五点闭环（2026-08-22）

这是在用户手动确认后的新障碍布局上进行的正式泛化验证。保持
`adaptive_goal_line_045` 不变，三种点位顺序各运行三次，共 9 次，全部 `5/5`
分段成功，非地面 Gazebo contacts 为 `0/9`。

| 顺序 | 结果目录 | 平均 wall [s] | 平均轨迹 [m] | 分段成功 | 非地面接触 |
| --- | --- | ---: | ---: | ---: | --- |
| M-A-B-C-D-M | [`run_01..03`](../../results/05_跨场景验证/场景01/跨场景场景01_五点闭环_MABCDM_2026-08-22) | 292.229 +/- 6.826 | 50.557 +/- 0.408 | 15/15 | 0/3 |
| C-A-D-B-M-C | [`run_01..03`](../../results/05_跨场景验证/场景01/跨场景场景01_顺序实验1_CADBMC_2026-08-22) | 288.647 +/- 2.752 | 49.297 +/- 0.110 | 15/15 | 0/3 |
| B-M-A-C-D-B | [`run_01..03`](../../results/05_跨场景验证/场景01/跨场景场景01_顺序实验2_BMACDB_2026-08-22) | 342.412 +/- 7.375 | 62.252 +/- 1.464 | 15/15 | 0/3 |

完整分析、9 张双视角轨迹图和复现命令见
[阶段2_跨场景场景01九次实验总结_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_跨场景场景01九次实验总结_2026-08-22.md)，
参数冻结见
[阶段2_跨场景场景01复现参数_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_跨场景场景01复现参数_2026-08-22.md)。

### 阶段 9：cross_scene_02 规范化世界 M→N 复测（2026-08-22）

本阶段先将用户保存快照中的静态障碍物实际位姿提升到顶层，删除 `<state>` 和内嵌
`waffle`，再保持 `adaptive_goal_line_045` 不变完成三次独立 M→N 回归。结果为
`2/3` 成功，三次非地面 Gazebo contacts 均为 `0`；run_03 在西侧第一道障碍前因
RPP collision projection 和 controller patience 超时而安全停止。

| 运行 | 结果目录 | wall [s] | Gazebo 轨迹 [m] | 末端误差 [m] | 状态 | 非地面接触 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| run_01 | [`run_01`](../../results/05_跨场景验证/场景02/场景02_MN规范化世界_2026-08-22/run_01) | 139.961 | 26.517 | 0.150 | 4 | none |
| run_02 | [`run_02`](../../results/05_跨场景验证/场景02/场景02_MN规范化世界_2026-08-22/run_02) | 142.932 | 26.409 | 0.128 | 4 | none |
| run_03 | [`run_03`](../../results/05_跨场景验证/场景02/场景02_MN规范化世界_2026-08-22/run_03) | 62.696 | 2.927 | 14.594 | 6 | none |

完整分析、3 张双视角轨迹图和规范化说明见
[阶段2_场景02_MN规范化世界三次复测报告_2026-08-22.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_MN规范化世界三次复测报告_2026-08-22.md)，
机器可读表见
[阶段2_场景02_MN规范化世界三次复测汇总.csv](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_MN规范化世界三次复测汇总.csv)。

### 阶段 10：cross_scene_02 原参数基线与四点闭环恢复增强（2026-08-23）

先不改变参数完成 `M -> N` 三次原参数复测，`adaptive_goal_line_045` 为 `3/3`
成功、`0/3` 非地面 contacts，平均 wall `141.745 ± 12.048 s`。结果目录为
[`场景02_MN原参数基线_2026-08-23`](../../results/05_跨场景验证/场景02/场景02_MN原参数基线_2026-08-23)，
报告为
[阶段2_场景02_MN原参数基线三次复测报告_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_MN原参数基线三次复测报告_2026-08-23.md)。

在四点闭环原始版本和 recovery_v1～v4 的失败排查后，v5 完成
`M(-8.5,0) -> N(8.5,0) -> X(-3,-4) -> Y(5,5) -> M(-8.5,0)` 三次独立实验。
结果为 `3/3` 完整 run、`12/12` 分段成功、`0/3` 非地面 contacts；平均总 wall
`545.459 ± 26.327 s`，平均 Gazebo 轨迹 `100.885 ± 4.765 m`。结果目录为
[`场景02_四点闭环_恢复增强v5_2026-08-23`](../../results/05_跨场景验证/场景02/场景02_四点闭环_恢复增强v5_2026-08-23)。

完整分析和图索引见
[阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_四点闭环恢复增强实验报告_2026-08-23.md) 和
[阶段2_场景02_四点闭环恢复增强轨迹图索引_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_四点闭环恢复增强轨迹图索引_2026-08-23.md)；
冻结参数和复现命令见
[阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md](../08_下一阶段实验归档_2026-08-22/阶段2_场景02_恢复增强v5参数冻结与复现手册_2026-08-23.md)。

场景 02 的 v1～v4 候选结果以及旧 M→N 失败证据不删除，统一保留在
`results/05_跨场景验证/场景02/`，但不纳入 v5 正式三次均值。

## 已完成但不保留原始目录的对照

2026-08-20 的 0.55 m 组也完成了 3/3 成功回归，平均 wall `116.425 s`、Gazebo 路径
`18.389 m`、末端误差 `0.231 m`，非地面接触 `0/3`。它的原始目录已从工作树移除，
统计数据仍保留在 [BENCHMARK_2026-08-20_SUMMARY.md](../02_原生规划基准_2026-08-20/BENCHMARK_2026-08-20_SUMMARY.md)。
这次清理不把 0.55 写成失败，只把它从“正式结果目录”降为历史对照。

早期 `NavFn + DWB` 的 `182.87 s`、Smac+RPP 的 `0.30 m` 失败样本，以及更早的单次
0.45/0.55 结果也只在阶段分析和 Git 历史中保留。它们不再和三次正式回归并列，避免
论文或复现实验误把 pilot、失败运行当成当前基线。

## 删除范围

本次删除分为三类：

1. 顶层单次或双向旧结果：`results/A_to_B_clearance`、`results/B_to_A_clearance`、
   `results/smac_rpp_030_A_to_B`、`results/smac_rpp_045_A_to_B`、
   `results/smac_rpp_045_B_to_A`、`results/smac_rpp_055_A_to_B`、
   `results/smac_rpp_055_final_A_to_B`、`results/smac_rpp_055_final_B_to_A`。
2. 正式阶段内被替代的目录：benchmark 的 0.55 三次；2026-08-20 目标线的 pilot、
   check、`cost12` 和 `cost30` 目录；2026-08-21 除 v4 三次正式回归外的 v1、v2、v3、
   `fast_north`、pilot 和 settled-pilot 目录。
3. 误生成的嵌套目录：`results/results/02_目标线规划优化/optimization_2026-08-20`。

4. 本次整理删除的重复双目标单次目录：
   `results/自适应目标线_多目标_起点到A到B_2026-08-21`。它已被后续修正版单次结果
   和正式三次双目标结果替代。

本次不删除源码、场景、冻结参数、阶段报告或 profile。已跟踪的旧结果和旧文档仍能从
`873fd39` 或更早的 Git 提交恢复；被忽略的临时结果只保留本索引和阶段报告中的统计。

精确删除的结果目录如下：

```text
results/A_to_B_clearance
results/B_to_A_clearance
results/smac_rpp_030_A_to_B
results/smac_rpp_045_A_to_B
results/smac_rpp_045_B_to_A
results/smac_rpp_055_A_to_B
results/smac_rpp_055_final_A_to_B
results/smac_rpp_055_final_B_to_A

results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_01
results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_02
results/benchmark_2026-08-20/smac_rpp_055_A_to_B_run_03

results/optimization_2026-08-20/goal_line_045_cost30_pilot_A_to_B
results/optimization_2026-08-20/goal_line_045_pilot_A_to_B
results/optimization_2026-08-20/goal_line_045_unknown35_pilot_A_to_B
results/optimization_2026-08-20/goal_line_quad_045_check_A_to_B
results/optimization_2026-08-20/smac_rpp_045_cost12_A_to_B_run_01
results/optimization_2026-08-20/smac_rpp_045_cost12_A_to_B_run_02
results/optimization_2026-08-20/smac_rpp_045_cost12_A_to_B_run_03
results/optimization_2026-08-20/smac_rpp_045_cost12_pilot_A_to_B
results/optimization_2026-08-20/smac_rpp_045_cost30_pilot_A_to_B

results/optimization_2026-08-21/fast_goalline_045_v1_A_to_B_run_01
results/optimization_2026-08-21/fast_goalline_045_v1_A_to_B_run_02
results/optimization_2026-08-21/fast_goalline_045_v1_A_to_B_run_03
results/optimization_2026-08-21/fast_goalline_045_v1_pilot_A_to_B
results/optimization_2026-08-21/fast_goalline_045_v1_pilot2_A_to_B
results/optimization_2026-08-21/fast_goalline_045_v2_A_to_B_run_01
results/optimization_2026-08-21/fast_goalline_045_v2_A_to_B_run_02
results/optimization_2026-08-21/fast_goalline_045_v2_A_to_B_run_03
results/optimization_2026-08-21/fast_goalline_045_v2_pilot_A_to_B
results/optimization_2026-08-21/fast_goalline_045_v3_pilot_A_to_B
results/optimization_2026-08-21/fast_goalline_045_v4_pilot_A_to_B
results/optimization_2026-08-21/fast_goalline_045_v4_settled_pilot_A_to_B
results/optimization_2026-08-21/fast_north_045_v1_A_to_B_run_01
results/optimization_2026-08-21/fast_north_045_v1_A_to_B_run_02
results/optimization_2026-08-21/fast_north_045_v2_A_to_B_run_01
results/optimization_2026-08-21/fast_north_045_v2_A_to_B_run_02
results/optimization_2026-08-21/fast_north_045_v2_A_to_B_run_03
results/optimization_2026-08-21/fast_north_045_v2_pilot_A_to_B
results/optimization_2026-08-21/fast_north_045_v3_A_to_B_run_01
results/optimization_2026-08-21/fast_north_045_v3_A_to_B_run_02
results/optimization_2026-08-21/fast_north_045_v3_A_to_B_run_03
results/optimization_2026-08-21/fast_north_045_v3_pilot_A_to_B
results/optimization_2026-08-21/fast_north_045_v4_pilot_A_to_B

results/results/optimization_2026-08-20/goal_line_045_cost30_check_A_to_B

results/自适应目标线_多目标_起点到A到B_2026-08-21
```

其中 benchmark 的 0.55 结果、2026-08-21 中间 profile 和顶层旧双向结果已被 Git 跟踪；
顶层 `smac_rpp_030/045/055`、2026-08-20 pilot/check、以及嵌套目录原先已被忽略。

## 版本和复现

当前推荐运行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=adaptive_goal_line_045'
```

完整回归记录：

```bash
sg docker -c './scripts/multi_waypoint_regression.sh \
  --goal A:5.0:-3.0:0.0 --goal B:5.0:6.0:0.0 \
  --goal C:-5.0:4.0:3.1415926 --goal D:0.0:0.0:0.0 \
  --goal M:-8.5:0.0:3.1415926 \
  --start-name M --start-x -8.5 --start-y 0.0 \
  --label 04_自适应目标线多目标/复现/adaptive_goal_line_045_five_waypoints --profile adaptive_goal_line_045'
```

当前分支仍保留 v2、v3、v4 的 launch profile。要复现旧阶段的代码级行为，按结果目录
`experiment.yaml` 中的 `git_commit` 建立独立 worktree；不要只在新代码上选择同名 profile
并声称二进制完全相同。

恢复本次清理前的任意结果：

```bash
git restore --source=873fd39 -- results/03_V4快速目标线/optimization_2026-08-21/fast_goalline_045_v2_A_to_B_run_01
```

将命令中的路径替换为归档前、且曾被 Git 跟踪的结果路径即可。原先被 `.gitignore`
忽略的顶层单次结果、2026-08-20 pilot/check 结果和嵌套 `results/results` 不能用 Git
恢复。恢复后不要把临时结果重新加入正式归档，除非它通过同样的三次干净重启、轨迹图
和 Gazebo contacts 验收。

## 以后保存规则

新实验的 label 必须带上对应的分类前缀和场景名，例如
05_跨场景验证/场景02/跨场景场景02_MABCDM_2026-08-22/run_01，避免把新结果
混入旧场景目录。

- 每个候选 profile 先提交源码和参数，再运行 3 次独立回归。
- 每个正式结果目录保存 `experiment.yaml`、实际参数快照、`metrics.yaml`、两套轨迹
  CSV、单视图和左右双视图、世界文件以及 contacts 证据。
- 每个阶段只把最好的一组 3 次结果放进 `results/`；pilot 和失败结果只保留统计结论，
  必要时从 Git 历史恢复。
- 每次清理先更新本文，再执行明确路径的删除，最后运行 `git diff --check`、检查失效
  引用并提交。

## 当前发展结论

项目已经完成当前 Gazebo 大室内场景中的 RGB-D 在线建图、全局 Smac/目标线规划、RPP
局部跟踪、速度平滑、collision monitor 安全过滤和多目标点到点避障闭环。
`adaptive_goal_line_045` 是当前通用 profile；v4 仍是当前场景的快速历史 benchmark，
不是任意未知地图的最优性证明。下一轮应以 adaptive profile 作为父版本，一次只改一个
因素，并继续以 3 次成功、无非地面接触、wall 时间、路径长度和轨迹一致性共同验收。
