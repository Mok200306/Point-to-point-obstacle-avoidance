# Gate 1 参数调优与候选审计记录

日期：2026-08-28  
分支：`exp/oracle-mppi-2026-08-27`

## 目的

在不引入任何未来障碍物信息的情况下，让 Nav2 MPPI 成为可重复运行的静态 Reactive
baseline。RPP 配置、Gate 0 结果和 `main` 分支不在本轮被替换。

## 候选演进

| 候选 | 关键设置 | 结果 | 是否进入正式基线 |
|---|---|---|---|
| 20 Hz 初始候选 | `time_steps=56`、`model_dt=0.05`、`batch_size=1000`、圆形 critic | 单次 smoke 可到达，但正式候选出现控制周期超时、终点前反复恢复 | 否 |
| 15 Hz 候选 | 试图降低周期 | 配置生成不完整，缺少 `global_costmap`，判定无效 | 否 |
| 多边形 footprint 候选 | 保持 MPPI 但让 Cost/Obstacle critic 使用 polygon footprint | CPU 计算负载过高，控制响应超时并卡住 | 否 |
| 10 Hz 低负载候选 | `time_steps=30`、`model_dt=0.10`、`batch_size=500`、`DiffDrive`、圆形 critic | smoke 成功；提交后 A→B/B→A 各 3 次均成功、零非地面 contacts | 是 |

## 只改动的一类变量

最终选择的改变属于 MPPI 计算预算/控制周期这一类：

```text
controller_frequency: 20 → 10 Hz
time_steps: 56 → 30
model_dt: 0.05 → 0.10 s
batch_size: 1000 → 500
```

同时保留差速运动模型、障碍 critic、速度上限和 0.45 m inflation；没有通过削弱碰撞
约束来制造成功结果。多边形 footprint 失败是单独的性能候选，没有混入最终配置。

## 冻结参数

正式文件：`experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml`

```text
controller_frequency = 10.0 Hz
time_steps = 30
model_dt = 0.10 s
H_mppi = 3.00 s
batch_size = 500
iteration_count = 1
motion_model = DiffDrive
vx_min / vx_max = -0.12 / 0.28 m/s
wz_max = 0.90 rad/s
visualize = false
CostCritic.consider_footprint = false
ObstaclesCritic.consider_footprint = false
inflation_radius = 0.45 m
```

## 结果与遗留问题

提交后的正式矩阵为 6/6 成功、零非地面 contacts。控制周期 P95 为
0.10024–0.10028 s，说明 10 Hz 预算可稳定执行。

但 6 次中有 3 次触发 `Failed to make progress` 和 spin recovery，MPPI 墙钟中位数约
159.0 s，而 Gate 0 RPP 中位数约 92.3 s。这个问题先作为静态 baseline 的已知特征冻结，
后续动态实验必须统计恢复和急停；如果以后为了公平比较而改动此 baseline，必须新开
版本并重新完成 Gate 1 六次回归，不能覆盖本快照。

## 证据目录

- 正式汇总：`experiments/oracle_mppi/gate1/mppi_static.csv`
- 正式结果：`experiments/oracle_mppi/gate1/case_A_to_B/run_01..03/`
- 正式结果：`experiments/oracle_mppi/gate1/case_B_to_A/run_01..03/`
- 失败/旧版本审计：`experiments/oracle_mppi/gate1/runner_audit/`
- Gate 1 总报告：`experiments/oracle_mppi/reports/GATE1_REPORT_2026-08-28.md`
