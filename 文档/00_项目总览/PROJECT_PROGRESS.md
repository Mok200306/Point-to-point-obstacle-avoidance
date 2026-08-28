# 项目进度总结

更新时间：2026-08-28（完成 Oracle Gate 4 PredictionCritic 与零风险回归）

## 当前状态

项目已完成两个静态跨场景 Gazebo 世界的 RGB-D 在线建图和多目标导航验证，当前进入
“场景 03 单因素泛化验证准备”阶段，尚未进入真实 D435i 正式实验。

| 模块 | 状态 |
| --- | --- |
| Docker / Ubuntu 22.04 / ROS 2 Humble | 已完成 |
| TurtleBot3 Waffle RGB-D 仿真 | 已完成 |
| RTAB-Map online RGB-D SLAM | 已完成并验证 |
| Nav2 全局/局部规划和 RPP | 已完成并验证 |
| GoalLineSmacPlanner 动态目标线软偏好 | 已完成 |
| collision_monitor 和 Gazebo contacts 证据 | 已完成 |
| large 场景 A→B v4 benchmark | 3/3 成功，平均约 81.07 s |
| cross_scene_01 三组五点闭环 | 9/9 成功，45/45 段，0/9 非地面 contacts |
| cross_scene_02 M→N 原参数 | 3/3 成功，0/3 非地面 contacts |
| cross_scene_02 四点闭环 v5 | 3/3 成功，12/12 段，0/3 非地面 contacts |
| cross_scene_02 四点闭环 v13 | 3/3 成功，12/12 段，0/3 非地面 contacts |
| 场景 03 | 尚未创建 |
| 真实 D435i | 软件入口准备，尚未实机验收 |
| Oracle Gate 4 PredictionCritic | 硬验收 PASS；3+3 zero-risk 回归通过，动态收益尚未验证 |

## 最新正式结果：场景02 v13

世界：`indoor_obstacle_course_cross_scene_02.world`
profile：`adaptive_goal_line_050_recovery_v13_line_tiebreaker`
任务：`M(-8.5,0) -> N(8.5,0) -> X(-3,-4) -> Y(5,5) -> M(-8.5,0)`

| run | 总耗时 s | 总轨迹 m | 最大末端误差 m | 段成功 | 非地面 contacts |
| --- | ---: | ---: | ---: | --- | --- |
| 01 | 539.848 | 98.210 | 0.162 | 4/4 | none |
| 02 | 540.191 | 98.042 | 0.162 | 4/4 | none |
| 03 | 449.438 | 85.223 | 0.132 | 4/4 | none |
| 平均 | **509.826** | **93.825** | **0.162** | **12/12** | **0/3** |

`509.826 s` 是三次平均值，不代表每次固定 510 秒。M→N 的在线首次建图仍有路径
和耗时方差；N→X 已不再出现旧 v5 的南侧 `y≈-6` 长回环。

正式报告和复现入口：
[场景02正式验收索引](../08_下一阶段实验归档_2026-08-22/03_场景02_正式验收/README.md)。

## 当前算法边界

系统确实是在线建图和在线规划：RTAB-Map持续更新 `/map`，每个目标段由当前实时
costmap重新规划；黑色起终点连线只用于可视化和目标线软偏好，不是预置路径。

当前证据只支持“在两个设计好的静态仿真场景中完成导航”，不支持“任意未知环境、
动态障碍、任意窄通道或真实 D435i 保证 100% 成功”。

## 归档入口

- [阶段2跨场景验证长篇总览](../08_下一阶段实验归档_2026-08-22/00_阶段总览/阶段2跨场景验证总览与当前状态_2026-08-24.md)
- [实验归档索引](EXPERIMENT_ARCHIVE_INDEX.md)
- [项目交接资料](../09_项目交接_2026-08-24/README.md)
- [结果 README](../../results/README.md)

## 下一步

1. 先按交接手册复现 v13 smoke test，不修改冻结参数；
2. 复制场景 02 为新的 `indoor_obstacle_course_cross_scene_03.world`，只改变一个因素；
3. 运行单段 smoke test，再做三次完整回归；
4. 若失败，按感知/TF、全局规划、局部控制、恢复、场景几何分类，不覆盖 v13；
5. 场景 03 稳定后，才进入真实 D435i 的低速分级验证。

## Oracle 实验当前状态

Oracle 任务书的 Gate 0–3 已通过，Gate 4 已在分支
`exp/oracle-g4-critic-2026-08-28` 完成硬验收。新增 `PredictionCritic` 按
`tau_k=(t_eval-t_msg)+k*model_dt` 读取 `/oracle/predicted_occupancy`，T1–T5 和
pluginlib 加载均通过。Reactive 与全零风险 Oracle 各运行 3 次，均为 3/3 成功、
0/3 非地面 Gazebo contacts，控制周期 P95 约 0.1003 s。

这不是动态 Oracle 收益结论：全零风险回归只验证插件接入不会改变静态基线。正式日志
中的 progress recovery、run02 的 stale 回退和 launch-wide planner teardown caveat
均已记录在 [Gate 4 报告](../../experiments/oracle_mppi/reports/GATE4_REPORT_2026-08-28.md)。
下一步是 Gate 5 的 S1/S2 动态闭环 smoke，必须保持 Reactive 与 Oracle 的公平对照。
