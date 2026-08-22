# 实验结果归档

更新时间：2026-08-22

results/ 只保存实验原始证据，不保存阶段分析正文。阶段结论和复现说明位于
[文档/08_下一阶段实验归档_2026-08-22](../文档/08_下一阶段实验归档_2026-08-22/)。

## 当前目录

| 目录 | 内容 | 当前正式结果 |
| --- | --- | ---: |
| [01_原生规划基准](01_原生规划基准) | 原生 Smac + RPP，0.45 m 基准 | 3 次 |
| [02_目标线规划优化](02_目标线规划优化) | GoalLineSmacPlanner 目标线软偏好 | 3 次 |
| [03_V4快速目标线](03_V4快速目标线) | large 场景快速目标线 v4 | 3 次 |
| [04_自适应目标线多目标](04_自适应目标线多目标) | 双目标、五点闭环和顺序实验 | 13 个 metrics |
| [05_跨场景验证](05_跨场景验证) | cross_scene_01 及 cross_scene_02 场景变化验证 | 场景 01：9 次；场景 02：原始快照 M→N 3 次，规范化世界复测 3 次 |

当前共保留 13 个原始实验集合、37 个 metrics.yaml：场景 02 新增原始快照和规范化世界
各 3 次 M→N 证据，其余为此前的正式三次回归运行和 1 次
早期修正版单次结果。每个正式运行目录内的 PNG、CSV、metrics、参数快照、世界快照和
contacts 证据均保持原样。

## 结果定位

- 原生基准：01_原生规划基准/benchmark_2026-08-20/
- 目标线优化：02_目标线规划优化/optimization_2026-08-20/
- V4：03_V4快速目标线/optimization_2026-08-21/
- 自适应目标线：04_自适应目标线多目标/
- 跨场景场景 01：05_跨场景验证/场景01/

全部阶段的耗时、轨迹长度、成功率和物理接触汇总见
[EXPERIMENT_ARCHIVE_INDEX.md](../文档/00_项目总览/EXPERIMENT_ARCHIVE_INDEX.md)。

## 保存规则

新实验必须使用新的 label，不能复用已有 run_01、run_02 或 run_03。建议把结果分类
前缀直接写入 label，例如：

~~~bash
--label "05_跨场景验证/场景02/跨场景场景02_MABCDM_2026-08-22/run_01"
~~~

回归脚本会把它写入 results/ 下对应的分类目录。每次正式运行至少保存：

- metrics.yaml 和实验参数；
- 在线地图/规划轨迹与 Gazebo 真值轨迹 CSV；
- trajectory.png 和 trajectory_comparison.png；
- 实际导航参数、碰撞监视参数和世界文件快照；
- Gazebo contacts 统计。

只移动目录或更新链接，不修改历史运行目录中的证据文件。新场景应复制当前冻结
profile，使用新的世界文件和新的结果分类；不要把新结果写入旧场景目录。

## 常用检查

~~~bash
cd /home/w417/RTAB-Map
find results -type f -name metrics.yaml | sort
find results/05_跨场景验证 -maxdepth 4 -name trajectory_comparison.png | sort
git diff --check
~~~
