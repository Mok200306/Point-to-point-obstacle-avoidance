# Oracle 预测式导航实验

本目录用于执行任务书《Oracle预测式导航生死实验_分阶段执行任务书_v1.docx》（2026-08-27）规定的逐 Gate 实验。Gate 4 已在独立分支 `exp/oracle-g4-critic-2026-08-28` 上完成硬验收；当前工作在独立分支 `exp/oracle-g5-closed-loop-2026-08-29` 上，Gate 5 正式闭环批次已完成但结论为 `BLOCKED`。现有 `main` 分支、历史正式结果和既有 RPP profile 不在本实验中直接修改。

## 研究问题

本阶段先不训练 Transformer，也不把速度外推称为预测。需要先用 Gazebo 中由确定性脚本产生的动态障碍未来真值，对照：

```text
Reactive MPPI       只使用当前障碍信息
Oracle Predictive MPPI  使用脚本可查询的真实未来占据
```

如果完美未来信息都不能稳定改善安全性、冲突处理或流畅性，则应停止 Transformer 主线；如果确有稳定收益，才进入预测模型研究。

## 强制执行规则

1. Gate 必须按 0 → 1 → … 顺序执行；smoke test 通过后才允许重复实验。
2. 每个 Gate 开始时记录 commit、world、launch、YAML、profile、seed、起终点和时间口径。
3. 不覆盖 `results/` 中既有正式证据；失败样本不能删除。
4. Reactive 与 Oracle 必须使用相同的场景、初始状态、速度、footprint、感知和随机种子，唯一变量是未来信息开关。
5. 每次参数只改变一类，并在报告中写清修改前后与归因。
6. Gazebo contacts、轨迹 CSV、参数快照、关键日志和 rosbag 都属于证据。

## 目录

```text
experiments/oracle_mppi/
├── README.md
├── gate0/
│   ├── README.md
│   ├── environment_snapshot.md
│   ├── baseline_rpp_static.csv
│   ├── case_A_to_B/
│   └── case_B_to_A/
├── configs/       # 后续 MPPI / Oracle 冻结配置
├── gate3/          # Oracle 时空占据接口验收与可视化
├── worlds/        # 后续 S1～S4 动态世界快照
├── launch/        # 后续独立 launch
├── scripts/       # Gate 自动化脚本
├── results/       # 后续 Gate 结果（失败样本也保留）
└── reports/       # Gate 报告和最终 GO/NO-GO
```

## 当前状态

| Gate | 状态 | 说明 |
|---|---|---|
| Gate 0 | PASS | `adaptive_goal_line_045` + RPP 静态回归基线已完成 6/6 成功、无非地面 contacts |
| Gate 1 | PASS（硬验收） | `reactive_mppi_static` + 10 Hz Reactive MPPI，A→B/B→A 各 3/3 成功、零非地面 contacts；效率和恢复次数有遗留问题 |
| Gate 2 | PASS（环境与证据链路） | 四类动态场景均已参数化；真实 collision、contacts、Gazebo 真值距离和动态轨迹证据已验证。Reactive MPPI 在 S2/S4 仍有真实动态碰撞，不能写成动态导航 100% 成功 |
| Gate 3 | PASS（接口硬验收） | Oracle 时空占据接口已完成离线和 ROS 2 smoke；尚未接入 Nav2 |
| Gate 4 | PASS（硬验收；teardown caveat） | PredictionCritic、时间对齐、T1–T5、pluginlib 加载和全零 Oracle 3+3 静态回归均已完成；动态收益留给 Gate 5/6 |
| Gate 5 | **BLOCKED** | S1/S2 各方法 5 次正式闭环已完成；Reactive 6/10、Oracle 6/10 通过，包含启动失败、超时和真实动态 contacts；不能进入 Gate 6 |
| Gate 6～8 | 未开始 | 必须先解除 Gate 5 的启动可靠性、超时和 Oracle 因果收益证据缺口 |

## 基线启动

从仓库根目录执行：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/stop.sh'
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /workspaces/rtabmap_tb3_nav && colcon build --symlink-install"'
```

Gate 0 的单次运行协议、记录主题和验收条件见：

- [gate0/README.md](gate0/README.md)
- [gate0/environment_snapshot.md](gate0/environment_snapshot.md)
- [Gate 0 正式报告](reports/GATE0_REPORT_2026-08-27.md)
- [Gate 1 正式报告](reports/GATE1_REPORT_2026-08-28.md)
- [Gate 1 参数调优与候选审计](gate1/GATE1_PARAMETER_TUNING_2026-08-28.md)

Gate 1 的正式矩阵汇总见 [gate1/mppi_static.csv](gate1/mppi_static.csv)。最终使用的
10 Hz 参数为 [nav2_mppi_reactive_10hz_params.yaml](configs/nav2_mppi_reactive_10hz_params.yaml)。
本轮 MPPI 没有使用未来障碍物信息；3/6 次出现 Nav2 progress recovery，虽然全部成功且
无非地面 contacts，但不能把它描述为效率优于 Gate 0 RPP。

## Gate 2 动态场景

Gate 2 复用冻结的静态大场景，在 Gazebo 启动后通过
`gazebo_ros/spawn_entity.py` 载入具有真实 box collision geometry 的
`worlds/oracle_dynamic_obstacle.sdf`，再由 `dynamic_obstacle_controller.py` 按场景
YAML 的确定性 waypoint schedule 驱动。控制器只执行动态障碍轨迹并记录 Gazebo
真值；当前阶段不向 Nav2 发布未来占据，也不使用当前速度外推未来。

四个场景配置位于 `configs/scenarios/`：S1 横穿、S2 对向、S3 斜穿、S4 停-走/变速。
批量 medium 命令如下，脚本顺序执行且失败目录保留：

```bash
./experiments/oracle_mppi/scripts/run_gate2_matrix.sh \
  --difficulty medium --run-prefix medium --start-run 1 \
  --runs-per-scenario 3
python3 experiments/oracle_mppi/scripts/summarize_gate2.py
```

正式结果位于 `gate2/S*/formal_*`，修复后的补跑位于 `postfix2_*`、`recheck_*` 等不可覆盖目录；矩阵状态写入
`gate2/matrix_status_20260828_*.csv`。所有带 `scenario.yaml` 的目录都会递归纳入
`gate2/summary.csv`，包括启动失败、spawn 失败和真实碰撞目录。完整 run 应包含
`dynamic_groundtruth.csv`、`dynamic_summary.yaml`、`metrics.yaml`、动态轨迹图、参数快照和 contacts 证据。

Gate 2 正式报告见 [GATE2_REPORT_2026-08-28.md](reports/GATE2_REPORT_2026-08-28.md)。报告严格区分：

- 动态环境与证据链路已经通过；
- Reactive MPPI 动态避障仍有 S2/S4 碰撞；
- 尚未实现 Oracle publisher、PredictionCritic 或 Transformer。

只有在报告和 `summary.csv` 复核完成后，才允许创建 `oracle-g2-pass` 标签。

## Gate 3 Oracle 未来真值接口

Gate 3 新增 `oracle_dynamic_nav_msgs` 和 `oracle_prediction_publisher`。publisher
从与 Gate 2 相同的确定性 waypoint schedule 查询 `pose_obstacle(t0 + tau)`，将
0.60 m × 0.60 m 动态障碍 footprint 栅格化为时间层；不使用当前速度外推，也不读取
Gazebo 当前状态猜测未来。

首版接口参数为：`frame=odom`、local rolling grid `6×5 m`、`0.05 m` 分辨率、
`dt=0.10 s`、`horizon=3.0 s`、`steps=31`、`publish=10 Hz`。消息类型为
`oracle_dynamic_nav_msgs/msg/PredictedOccupancyGrid`，主题为
`/oracle/predicted_occupancy`。

离线与 live smoke 命令、消息字段、时间语义和回退规则见
[Gate 3 README](gate3/README.md)。正式结论见
[Gate 3 报告](reports/GATE3_REPORT_2026-08-28.md)。Gate 3 通过只表示未来信息
接口在空间、时间和 ROS 2 消息层面正确；Gate 3 本身不包含 PredictionCritic，不能把
本 Gate 写成 Oracle 动态导航性能通过。Critic 的接入与时间对齐在 Gate 4 单独验收。

## Gate 4 PredictionCritic

Gate 4 新增 `src/nav2_mppi_prediction_critic`，以 pluginlib 方式加载
`mppi::critics::PredictionCritic`。它只订阅 Gate 3 的
`/oracle/predicted_occupancy`，并按
`tau_k = (t_eval - t_msg) + k * model_dt` 对 MPPI 候选轨迹逐点采样。消息 frame、
网格尺寸、时间层、数据长度和时间新鲜度均经过检查；无消息、stale、越界或超出预测
时域时回退，不会让 controller 崩溃。

T1–T5 离线测试、真实 Nav2 pluginlib configure/activate 和全零风险 smoke 的命令与
证据规则见 [gate4/README.md](gate4/README.md)。截至当前实现，已证明插件能够加载并
在零风险 Oracle 下工作；正式 3+3 对照已经完成。结果和 caveat 见
[Gate 4 正式报告](reports/GATE4_REPORT_2026-08-28.md)。这还不是动态场景收益结论。

Gate 4 正式矩阵位于 `gate4/zero_risk/`：Reactive 与 Oracle zero-risk 均为 3/3
成功、0/3 非地面 contacts，控制周期 P95 约 0.1003 s。Oracle run 02 有 17 次
stale 计数但按设计安全回退；所有正式运行均最终到达。日志中的
`Failed to make progress` 是两组都出现的 MPPI/recovery 事件，不能省略。整套 launch
联动退出时仍有 planner teardown 工程告警，需在 Gate 5 前做隔离生命周期复核。

## Gate 5：闭环 Oracle 对照（当前为 BLOCKED）

Gate 5 的正式批次位于
`gate5/formal_20260830_01/`，共 20 个计划 run：S1 横穿和 S2 对向各 5 次
Reactive、5 次 Oracle。完整的机器可读汇总为：

- [20 行运行汇总](gate5/formal_20260830_01/gate5_smoke_summary.csv)
- [10 行配对汇总](gate5/formal_20260830_01/gate5_paired_summary.csv)
- [Gate 5 正式报告](reports/GATE5_REPORT_2026-08-30.md)
- [Gate 5 参数扫频审查](reports/GATE5_PARAMETER_SWEEP_2026-08-30.md)
- [Gate 5 阻塞报告](reports/GATE5_BLOCKER_REPORT_2026-08-30.md)

当前硬验收结果：

```text
S1 Reactive: 5/5    S1 Oracle: 3/5
S2 Reactive: 1/5    S2 Oracle: 3/5
总计 Reactive: 6/10    Oracle: 6/10
```

失败原因必须保留在统计分母中：S1 Oracle 有 2 次导航超时；S2 有 3 次
Nav2 启动失败；S2 Reactive 有 2 次真实动态障碍 contacts（其中一次最终到达、
一次超时）。因此“Oracle 已接入且记录到部分风险”成立，但“Oracle 已稳定改善
动态导航”尚未成立，不能进入 Gate 6 正式统计或 Transformer 训练。

正式汇总脚本现在会扫描每个 `reactive_run_N`/`oracle_run_N` 目录的
`experiment.yaml`/`scenario.yaml`，再结合 `matrix_status.csv`。即使运行在 Nav2
启动前结束，也会作为 `STARTUP_FAILURE` 进入 CSV；不会因缺少
`gate5_analysis.yaml` 而被漏计。未来重新生成汇总：

```bash
python3 experiments/oracle_mppi/scripts/summarize_gate5.py \
  --root experiments/oracle_mppi/gate5/formal_20260830_01 \
  --status experiments/oracle_mppi/gate5/formal_20260830_01/matrix_status.csv \
  --output experiments/oracle_mppi/gate5/formal_20260830_01/gate5_smoke_summary.csv \
  --pairs-output experiments/oracle_mppi/gate5/formal_20260830_01/gate5_paired_summary.csv
```

参数扫频 `gate5/cost_sweep_20260829_01/` 只有每个权重一组配对、且使用旧
commit 和诊断场景，只能作为诊断参考，不能据此冻结 `cost_weight=10` 或 `50`。
正式下一步应在当前 commit、正式 S1/S2 场景上进行至少 3 次配对的单变量扫频。

## 证据命名

每次实验使用不可覆盖的目录：

```text
gate0/case_A_to_B/run_01/
gate0/case_A_to_B/run_02/
gate0/case_A_to_B/run_03/
gate0/case_B_to_A/run_01/
gate0/case_B_to_A/run_02/
gate0/case_B_to_A/run_03/
gate2/S1_crossing/medium_01/
gate2/S2_oncoming/medium_01/
gate2/S3_diagonal/medium_01/
gate2/S4_stop_go/medium_01/
```

正式实验开始后，不通过删除目录来“清理”结果；如需重新跑，使用新的 `run_04` 或带日期的标签，并在汇总表中保留所有结果。

Gate 0 的实验目录会保留完整的本地 rosbag 和压缩 Gazebo contacts 原始流；由于
这些二进制流可能超过 GitHub 单文件限制，仓库版本化的是其 `metrics.yaml`、
`experiment.yaml`、CSV、图、参数快照和 `rosbag_info.txt`。完整原始流和 verbose
日志仍与工作区中的同名 run 目录一起保存，不能据此把未上传的原始流误认为未采集。
