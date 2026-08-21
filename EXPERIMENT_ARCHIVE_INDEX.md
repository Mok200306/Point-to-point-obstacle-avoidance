# 实验归档索引

更新时间：2026-08-21

本文是结果目录的唯一整理入口。当前工作树只保留每个正式阶段最有代表性的三次
完整回归；pilot、失败检查、重复单次结果和被后续阶段替代的原始目录不再占用仓库
空间。被 Git 跟踪的移除结果仍可从整理前的提交 `873fd39` 恢复；原先就被 `.gitignore`
忽略的临时结果没有进入 Git，删除后不再有本地副本。

## 当前保留结果

### 阶段 1：原生 Smac + RPP 基准（2026-08-20）

保留 0.45 m 组，因为它在无非地面物理接触的前提下平均墙钟时间更短，符合本项目
“无碰撞是硬门槛，满足后时间越短越好”的比较规则。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`smac_rpp_045_A_to_B_run_01`](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_01) | 115.83 | 18.187 | 0.264 | none |
| 02 | [`smac_rpp_045_A_to_B_run_02`](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_02) | 113.90 | 18.159 | 0.183 | none |
| 03 | [`smac_rpp_045_A_to_B_run_03`](results/benchmark_2026-08-20/smac_rpp_045_A_to_B_run_03) | 115.01 | 18.174 | 0.365 | none |
| 平均 | 3/3 成功 | **114.91** | **18.173** | **0.271** | **0/3** |

代码快照：`a1389ff`；参数和实验条件见
[BENCHMARK_2026-08-20_SUMMARY.md](BENCHMARK_2026-08-20_SUMMARY.md) 与
[FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md](FROZEN_NAVIGATION_PARAMETERS_2026-08-20.md)。

### 阶段 2：目标线四点软偏好（2026-08-20）

保留 `GoalLineSmacPlanner` 的三次正式回归。这一阶段验证了全局规划仍受 costmap
碰撞约束，同时对当前位置到目标的直线增加软偏好，障碍物后能向目标线回收。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`goal_line_quad_045_A_to_B_run_01`](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_01) | 113.90 | 18.312 | 0.156 | none |
| 02 | [`goal_line_quad_045_A_to_B_run_02`](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_02) | 116.72 | 18.280 | 0.170 | none |
| 03 | [`goal_line_quad_045_A_to_B_run_03`](results/optimization_2026-08-20/goal_line_quad_045_A_to_B_run_03) | 110.28 | 17.339 | 0.160 | none |
| 平均 | 3/3 成功 | **113.63** | **17.977** | **0.162** | **0/3** |

代码快照：`c18894e`；阶段报告见
[NAVIGATION_OPTIMIZATION_2026-08-20.md](NAVIGATION_OPTIMIZATION_2026-08-20.md)，冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md](FROZEN_NAVIGATION_PARAMETERS_OPTIMIZED_2026-08-20.md)。

### 阶段 3：快速分段目标线 v4（2026-08-21）

这是当前仿真场景的推荐 profile。保留三次完整结果，三次均低于 100 s，且保持
无非地面 Gazebo 接触。

| 运行 | 结果目录 | wall [s] | Gazebo 路径 [m] | 末端误差 [m] | 物理接触 |
| --- | --- | ---: | ---: | ---: | --- |
| 01 | [`fast_goalline_045_v4_A_to_B_run_01`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_01) | 79.01 | 17.336 | 0.361 | none |
| 02 | [`fast_goalline_045_v4_A_to_B_run_02`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_02) | 78.26 | 17.451 | 0.234 | none |
| 03 | [`fast_goalline_045_v4_A_to_B_run_03`](results/optimization_2026-08-21/fast_goalline_045_v4_A_to_B_run_03) | 85.94 | 17.359 | 0.274 | none |
| 平均 | 3/3 成功 | **81.07 +/- 3.46** | **17.382 +/- 0.050** | **0.290 +/- 0.053** | **0/3** |

运行时 profile 提交：`452b45f`；最终可复现提交：`78bb860`，当前文档提交为其后的
`873fd39`。详细报告见
[NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md](NAVIGATION_OPTIMIZATION_2026-08-21_FAST_GOALLINE_V4.md)，
冻结参数见
[FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md](FROZEN_NAVIGATION_PARAMETERS_FAST_GOALLINE_045_V4_2026-08-21.md)。

## 已完成但不保留原始目录的对照

2026-08-20 的 0.55 m 组也完成了 3/3 成功回归，平均 wall `116.425 s`、Gazebo 路径
`18.389 m`、末端误差 `0.231 m`，非地面接触 `0/3`。它的原始目录已从工作树移除，
统计数据仍保留在 [BENCHMARK_2026-08-20_SUMMARY.md](BENCHMARK_2026-08-20_SUMMARY.md)。
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
3. 误生成的嵌套目录：`results/results/optimization_2026-08-20`。

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
```

其中 benchmark 的 0.55 结果、2026-08-21 中间 profile 和顶层旧双向结果已被 Git 跟踪；
顶层 `smac_rpp_030/045/055`、2026-08-20 pilot/check、以及嵌套目录原先已被忽略。

## 版本和复现

当前推荐运行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/launch_demo.sh gazebo_gui:=true rviz:=true rtabmap_viz:=false online:=true localization:=false reset_db:=true navigation_profile:=fast_goalline_045_v4'
```

完整回归记录：

```bash
sg docker -c './scripts/regression_leg.sh --x 8.5 --y 0.0 --yaw 0.0 --settle-seconds 5 --label manual/fast_goalline_045_v4_A_to_B --profile fast_goalline_045_v4'
```

当前分支仍保留 v2、v3、v4 的 launch profile。要复现旧阶段的代码级行为，按结果目录
`experiment.yaml` 中的 `git_commit` 建立独立 worktree；不要只在新代码上选择同名 profile
并声称二进制完全相同。

恢复本次清理前的任意结果：

```bash
git restore --source=873fd39 -- results/optimization_2026-08-21/fast_goalline_045_v2_A_to_B_run_01
```

将命令中的路径替换为归档前、且曾被 Git 跟踪的结果路径即可。原先被 `.gitignore`
忽略的顶层单次结果、2026-08-20 pilot/check 结果和嵌套 `results/results` 不能用 Git
恢复。恢复后不要把临时结果重新加入正式归档，除非它通过同样的三次干净重启、轨迹图
和 Gazebo contacts 验收。

## 以后保存规则

- 每个候选 profile 先提交源码和参数，再运行 3 次独立回归。
- 每个正式结果目录保存 `experiment.yaml`、实际参数快照、`metrics.yaml`、两套轨迹
  CSV、单视图和左右双视图、世界文件以及 contacts 证据。
- 每个阶段只把最好的一组 3 次结果放进 `results/`；pilot 和失败结果只保留统计结论，
  必要时从 Git 历史恢复。
- 每次清理先更新本文，再执行明确路径的删除，最后运行 `git diff --check`、检查失效
  引用并提交。

## 当前发展结论

项目已经完成当前 Gazebo 大室内场景中的 RGB-D 在线建图、全局 Smac/目标线规划、RPP
局部跟踪、速度平滑、collision monitor 安全过滤和 A -> B 点到点避障闭环。v4 是当前
场景的快速推荐基线，不是任意未知地图的最优性证明。下一轮应从 v4 作为父版本，一次
只改一个因素，并继续以 3 次成功、无非地面接触、wall 时间、路径长度和轨迹一致性
共同验收。
