# Gate 4 报告：PredictionCritic 插件与时间对齐

日期：2026-08-28
任务书：`/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`
实验分支：`exp/oracle-g4-critic-2026-08-28`
本 Gate 提交：`73872bc5db65c56ad4f7aefa3e25fe63b92bf0ac`
父 Gate：`oracle-g3-pass`（`55f5cc8`）

## 1. Gate 结论

**Gate 4：PASS（硬验收通过；存在 launch-wide teardown 说明项）。**

本 Gate 已完成并验证：

1. `PredictionCritic` 可以作为 Nav2 MPPI pluginlib 插件加载；
2. Oracle 消息经过 frame、尺寸、数据长度、时间和原点方向检查后进入 critic；
3. MPPI 第 `k` 个候选轨迹点按同一未来时间层采样，而不是把未来障碍压成静态障碍；
4. T1–T5 离线测试全部通过，并证明同一空间位置在不同时间的 cost 不同；
5. 开启真实 Oracle publisher 和 PredictionCritic 的全零风险静态回归完成 3 次；
6. 对照 Reactive MPPI 完成 3 次；两组均 3/3 到达、Gazebo 非地面 contacts 为 0；
7. Oracle 三次均完成消息字段验证，插件日志显示 `status=active`、`max_risk=0`、
   `max_cost=0`；
8. 控制周期 P95 约为 `0.1003 s`，没有发现 controller crash、NaN 或插件加载错误。

这里的 PASS 只表示 Gate 4 的“插件、时间对齐、零风险回退和静态回归”硬验收通过。
全零 Oracle 不包含动态冲突，因此**不能据此声称 Oracle 已改善动态导航**；动态行为
差异属于 Gate 5/6。正式实验中多次出现 `Failed to make progress`，但均完成恢复并
最终到达，已作为恢复事件保留，不能写成全程无错误。

## 2. 本 Gate 的研究问题与范围

任务书要求先回答：未来占据信息能否正确接入 MPPI，并在时间上与候选轨迹对齐。
本 Gate 不训练 Transformer，不使用当前速度外推未来，也不改变 Gate 1 Reactive
MPPI 的参数来制造对照差异。

对照关系如下：

```text
Reactive MPPI              当前 RGB-D/RTAB-Map/Nav2 信息，无 Oracle topic
Oracle zero-risk MPPI      相同场景和当前信息 + Oracle publisher + PredictionCritic
                           Oracle 障碍代理固定在 (100, 100)，不进入局部窗口，因此风险全为 0
```

两组使用同一个 world、起点、终点、MPPI 动力学、速度、costmap、footprint 和记录
脚本。唯一预期差异是 Oracle 组额外启动 publisher 并将 `PredictionCritic` 加入
critics 列表。

## 3. 实现内容

### 3.1 PredictionCritic 插件

新增包：`src/nav2_mppi_prediction_critic/`

```text
src/nav2_mppi_prediction_critic/
├── CMakeLists.txt
├── package.xml
├── prediction_critics.xml
├── include/nav2_mppi_prediction_critic/
│   ├── prediction_critic.hpp
│   └── prediction_grid_sampler.hpp
├── src/prediction_critic.cpp
└── test/
    ├── test_prediction_grid_sampler.cpp
    └── prediction_critic_offline_test.cpp
```

插件类型为：

```text
mppi::critics::PredictionCritic
```

插件只在 Oracle profile 中加入 `PredictionCritic`；Reactive 参数文件保持原有
critics 列表不变。

### 3.2 冻结接口

| 参数 | 冻结值 |
|---|---:|
| Oracle topic | `/oracle/predicted_occupancy` |
| frame | `odom` |
| grid | `120 × 100` |
| resolution | `0.05 m/cell` |
| physical window | `6.0 m × 5.0 m` |
| prediction `dt` | `0.10 s` |
| layers | `31` |
| prediction horizon | `3.0 s` |
| temporal interpolation | `linear` |
| `cost_weight` | `50.0` |
| `cost_power` | `1` |
| stale threshold | `0.30 s` |
| clock skew tolerance | `0.05 s` |
| out of bounds | ignore and count |
| out of horizon | ignore and count |

Oracle 消息数据按 `[steps, height, width]` row-major 保存。消息回调会拒绝 frame 不符、
数据长度不符、非正分辨率/时间步长或非零 yaw 原点，避免错误消息进入 MPPI 评分。

## 4. 时间对齐和评分公式

设：

```text
t_msg  = Oracle 消息 header.stamp
t_eval = MPPI 当前候选轨迹评价时间
age    = t_eval - t_msg
k      = 候选轨迹时间索引
model_dt = MPPI 模型时间步长
```

实现使用：

```text
tau_k = (t_eval - t_msg) + k * model_dt
```

当 `tau_k` 落在两个 Oracle 层之间时进行线性插值。每条候选轨迹的风险项为：

```text
J_pred = (cost_weight * sum_k R(x_k, y_k, tau_k)) ^ cost_power
```

若消息过旧、时间反向超过容差、采样点越界或超出预测时域，critic 安全回退并记录
计数；不会用错误的时间层继续评分，也不会因为边界点或 NaN 使 controller 崩溃。

## 5. T1–T5 离线测试

机器可读结果：[`critic_debug.csv`](../gate4/critic_debug.csv)。交付前重测结果见
[`critic_debug_recheck.csv`](../gate4/critic_debug_recheck.csv)。两次离线程序均输出
`all_pass=true`，共 8 行，其中最后一行是任务书要求的 cost separation 证明。

| 测试 | 关键构造 | 结果 |
|---|---|---|
| T1 | 所有 Oracle 层为 0 | cost=0，PASS |
| T2 | 同一空间位置，冲突层在 `t=1.0 s` | `t=0.2` 风险低于 `t=1.0`，PASS |
| T2 插值 | `t=0.5 s` 位于两层之间 | 风险=0.5，PASS |
| T3 | 消息 age=0.2 s，轨迹点再前进 0.8 s | 对齐到 `tau=1.0 s`，PASS |
| T4 | 采样时间超出未来 horizon | 安全回退，不崩溃，PASS |
| T5 | 非有限/越界空间采样 | 不越界、不产生 NaN，PASS |
| cost separation | 冲突轨迹与冲突前轨迹空间相同 | 冲突 cost 更高，PASS |

具体数值包括：冲突时风险 `1.0`、cost `50`；插值风险 `0.5`、cost `25`；
所有行的 `passed=true`。

## 6. 构建、pluginlib 和实时 smoke

构建与单元测试：

```bash
cd /home/w417/RTAB-Map
docker compose up -d
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   cd /workspaces/rtabmap_tb3_nav && \
   colcon build --symlink-install && \
   colcon test --packages-select nav2_mppi_prediction_critic \
     --event-handlers console_direct+ && \
   colcon test-result --verbose'
```

离线测试：

```bash
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   /workspaces/rtabmap_tb3_nav/install/nav2_mppi_prediction_critic/lib/nav2_mppi_prediction_critic/prediction_critic_offline_test \
   --output /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/gate4/critic_debug.csv'
```

三次正式 Oracle run 的 launch 日志均包含：

```text
PredictionCritic enabled: topic=/oracle/predicted_occupancy
PredictionCritic status=active ... max_risk=0.000 max_cost=0.000
```

三次 Oracle 消息验证均为 exit `0`，实际字段为：

```text
frame=odom, grid=120x100@0.050m, dt=0.100,
steps=31, data_len=372000, source=oracle
```

形式化运行时 `/controller_server`、`/planner_server`、`/bt_navigator` 和
`/collision_monitor` 都记录为 `active [3]`。Oracle run 01/03 的 stale 计数为 0，
run 02 的内部 stale 计数为 17；这些周期均安全回退，导航没有失败。

第一次 smoke 的验证器没有 source 新消息包的 install 环境，产生：

```text
ModuleNotFoundError: No module named 'oracle_dynamic_nav_msgs'
```

该失败目录 `gate4/smoke_oracle_s0_run_01/` 保留作为复现错误证据。补充 source
后的 `smoke_oracle_s0_run_02/` 验证通过。正式命令必须同时 source
`/opt/ros/humble/setup.bash` 和工作区 `install/setup.bash`。

## 7. 正式 3+3 零风险回归

机器可读汇总：[`zero_risk_summary.csv`](../gate4/zero_risk/zero_risk_summary.csv)。
矩阵状态：[`matrix_status.csv`](../gate4/zero_risk/matrix_status.csv)。

固定条件：

```text
world: src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
start: (-8.5, 0.0)
goal:  ( 8.5, 0.0)
online=true, localization=false, reset_db=true, use_sim_time=true
controller_frequency=10 Hz, model_dt=0.10 s, time_steps=30, batch_size=500
motion_model=DiffDrive, vx_min=-0.12, vx_max=0.28, wz_max=0.90
```

| run | 方法 | status | 仿真时间 (s) | 墙钟时间 (s) | Gazebo 路径 (m) | 末端误差 (m) | 非地面 contact |
|---|---|---:|---:|---:|---:|---:|---|
| reactive_run_01 | Reactive | 4 | 161.0 | 205.510 | 18.377 | 0.0260 | 否 |
| reactive_run_02 | Reactive | 4 | 160.7 | 204.537 | 18.448 | 0.0262 | 否 |
| reactive_run_03 | Reactive | 4 | 164.8 | 210.191 | 18.324 | 0.0340 | 否 |
| oracle_run_01 | Oracle zero-risk | 4 | 164.4 | 209.908 | 18.347 | 0.0276 | 否 |
| oracle_run_02 | Oracle zero-risk | 4 | 179.5 | 228.251 | 17.778 | 0.0718 | 否 |
| oracle_run_03 | Oracle zero-risk | 4 | 166.5 | 213.082 | 17.800 | 0.0331 | 否 |

### 7.1 组级统计

| 方法 | 成功率 | 仿真时间均值 (s) | 墙钟时间均值 (s) | 路径均值 (m) | 末端误差均值 (m) | 控制 P95 周期均值 (s) |
|---|---:|---:|---:|---:|---:|---:|
| Reactive | 3/3 | 162.167 | 206.746 | 18.383 | 0.0287 | 0.100304 |
| Oracle zero-risk | 3/3 | 170.133 | 217.080 | 17.975 | 0.0442 | 0.100318 |

相对 Reactive，Oracle zero-risk 的平均仿真时间增加约 `4.91%`，墙钟时间增加约
`5.00%`，路径长度减少约 `2.22%`。末端误差绝对值仍在 `0.028–0.072 m`，但均值
增加约 `0.016 m`；该差异不能解释为动态导航收益或退化，因为两组都使用全零未来
风险，且只有 3+3 次样本。

### 7.2 安全与稳定性证据

- 6/6 Nav2 action status 为 `4 (SUCCEEDED)`；
- 6/6 `gazebo_non_ground_contact=false`；
- 6 个运行目录均有 `metrics.yaml`、`experiment.yaml`、轨迹 CSV、Gazebo 轨迹 CSV、
  参数快照和 `trajectory_comparison.png`；
- Reactive 和 Oracle 每次均观察到 4 次 `Failed to make progress` 日志，随后执行
  spin/recovery 并最终到达；这是当前 MPPI 静态基线的既有恢复行为，不是 Oracle 独有；
- 未发现 `NaN`、controller crash、PredictionCritic rejected message 或插件加载错误；
- Oracle run 02 的 17 次 stale 计数没有引起 controller failure，证明安全回退路径有效。

轨迹图：

| 方法 | run 01 | run 02 | run 03 |
|---|---|---|---|
| Reactive | [图](../gate4/zero_risk/reactive_run_01/trajectory_comparison.png) | [图](../gate4/zero_risk/reactive_run_02/trajectory_comparison.png) | [图](../gate4/zero_risk/reactive_run_03/trajectory_comparison.png) |
| Oracle zero-risk | [图](../gate4/zero_risk/oracle_run_01/trajectory_comparison.png) | [图](../gate4/zero_risk/oracle_run_02/trajectory_comparison.png) | [图](../gate4/zero_risk/oracle_run_03/trajectory_comparison.png) |

## 8. 生命周期 teardown 说明项

正式导航运行时四个核心生命周期节点均为 active，PredictionCritic 在 controller
configure 阶段完成初始化并持续参与 score。独立生命周期检查中，
`/controller_server` 的 deactivate 和 cleanup 已成功，状态能够回到
`unconfigured [1]`。

但通过 Nav2 lifecycle manager 联动关闭整套 launch 时，planner 在 teardown 阶段
出现额外的 teardown 错误。当前证据没有显示该错误来自 PredictionCritic：

- 错误发生在整套 launch 的联动退出阶段，而非插件 configure/score 阶段；
- 正式 3+3 运行没有 controller crash、NaN 或导航失败；
- 所有正式 run 在结束前均完成 goal，runtime lifecycle 快照均为 active。

因此本报告将 Gate 4 判为“硬验收 PASS，整套 Nav2 退出过程仍有工程 caveat”。在进入
Gate 5 前应补充一次隔离的 lifecycle deactivate/cleanup 回归，分别记录
`controller_server`、`planner_server` 和 `collision_monitor`，避免把 launch teardown
副作用混入 PredictionCritic 结论。

## 9. Gate 4 硬验收清单

| 任务书要求 | 判定 | 证据 |
|---|---|---|
| T1–T5 全部通过 | PASS | `gate4/critic_debug.csv` |
| 同位置不同时间 cost 不同 | PASS | `cost_separation` 行、T2 行 |
| pluginlib 加载 | PASS | 正式 Oracle `launch.log` |
| configure/activate | PASS | 正式 Oracle `launch.log`、runtime lifecycle |
| deactivate/cleanup 路径 | PASS with caveat | 独立生命周期检查；launch-wide teardown 仍需隔离复核 |
| 全零 Oracle 静态回归至少 3 次 | PASS | `zero_risk/oracle_run_01..03` |
| Reactive 对照至少 3 次 | PASS | `zero_risk/reactive_run_01..03` |
| 全零 Oracle 不改变安全结果 | PASS | 两组均 3/3、0/3 contacts |
| 控制周期稳定 | PASS | 各 run `control_frequency.yaml`，P95 约 0.1003 s |
| 动态 Oracle 是否改善导航 | 未判定 | 留给 Gate 5/6 |

## 10. 复现命令

### 10.1 构建和离线验收

见 [`gate4/README.md`](../gate4/README.md)。

### 10.2 新目录执行 3+3 回归

不要把新实验写入已有 `zero_risk/`，以免覆盖矩阵索引。使用新的仓库内目录：

```bash
cd /home/w417/RTAB-Map
new_root="experiments/oracle_mppi/gate4/reproduction_$(date +%Y%m%d_%H%M%S)"
./experiments/oracle_mppi/scripts/run_gate4_zero_risk_matrix.sh \
  --runs 3 \
  --root "$new_root"
```

随后可以直接汇总：

```bash
python3 experiments/oracle_mppi/scripts/summarize_gate4.py \
  --root "$new_root" \
  --output "$new_root/zero_risk_summary.csv"
```

脚本对非空 run 目录拒绝覆盖；每次新 root 都会保留独立参数、轨迹、metrics 和
状态 CSV。Git 忽略 `gazebo_contacts.log.gz` 和 rosbag 数据库等大体积原始流，但这些
文件在本机正式 run 目录中保留；Git 提交包含对应的 metrics、contact 判定、CSV、图和
参数快照，足以复核汇总并重跑实验。

## 11. Gate 5 入口和禁止事项

当前允许进入 Gate 5 的原因是 Gate 4 接口硬验收已经通过。下一步只能在不改动
Reactive 基线的前提下：

1. 先做 S1 横穿 medium 的小批量闭环 smoke；
2. 再做 S2 对向 medium 的小批量闭环 smoke；
3. Reactive 与 Oracle 使用相同动态脚本、起点、终点和确定性条件；
4. 记录提前减速、最小 clearance、急停、恢复、碰撞和时间；
5. 对 `PredictionCritic.cost_weight` 做小范围单变量 sweep 后冻结；
6. 只有观察到可复核的提前控制变化，才进入 Gate 6 正式统计。

本报告不支持以下结论：

- Oracle 已经比 Reactive 更安全或更快；
- 已经证明 Transformer 值得训练；
- 已经完成 4 类动态场景的预测式导航；
- 真实 D435i 可以直接复用而无需重新校准。
