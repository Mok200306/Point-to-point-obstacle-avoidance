# Oracle 预测式导航实验

本目录用于执行任务书《Oracle预测式导航生死实验_分阶段执行任务书_v1.docx》（2026-08-27）规定的逐 Gate 实验。实验在分支 `exp/oracle-mppi-2026-08-27` 上进行，现有 `main` 分支、历史正式结果和既有 RPP profile 不在本实验中直接修改。

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
| Gate 1 | 未开始 | MPPI 静态基线 |
| Gate 2 | 未开始 | S1 横穿、S2 对向、S3 斜穿、S4 停-走/变速 |
| Gate 3 | 未开始 | Oracle 时空占据接口 |
| Gate 4 | 未开始 | PredictionCritic 与时间对齐 |
| Gate 5～8 | 未开始 | 闭环、统计、消融、最终复现包 |

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

## 证据命名

每次实验使用不可覆盖的目录：

```text
gate0/case_A_to_B/run_01/
gate0/case_A_to_B/run_02/
gate0/case_A_to_B/run_03/
gate0/case_B_to_A/run_01/
gate0/case_B_to_A/run_02/
gate0/case_B_to_A/run_03/
```

正式实验开始后，不通过删除目录来“清理”结果；如需重新跑，使用新的 `run_04` 或带日期的标签，并在汇总表中保留所有结果。

Gate 0 的实验目录会保留完整的本地 rosbag 和压缩 Gazebo contacts 原始流；由于
这些二进制流可能超过 GitHub 单文件限制，仓库版本化的是其 `metrics.yaml`、
`experiment.yaml`、CSV、图、参数快照和 `rosbag_info.txt`。完整原始流和 verbose
日志仍与工作区中的同名 run 目录一起保存，不能据此把未上传的原始流误认为未采集。
