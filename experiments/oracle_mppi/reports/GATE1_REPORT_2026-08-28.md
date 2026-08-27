# Gate 1 正式报告：Reactive MPPI 静态基线

日期：2026-08-28  
实验分支：`exp/oracle-mppi-2026-08-27`  
Gate 1 执行提交：`a028838250104afd84a6a449910e9aa4175f1c28`  
任务书：`/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`

## 1. Gate 状态

**Gate 1：PASS（硬验收通过；质量目标有明确遗留问题）。**

这表示当前配置已经可以作为“没有未来信息的 Reactive MPPI”静态对照基线，允许进入
Gate 2 动态场景准备。它不表示 MPPI 已经比 RPP 更快，也不表示 Oracle 或
PredictionCritic 已经实现。

| 任务书硬验收 | 结果 |
|---|---|
| A→B 静态回归 | 3/3 成功 |
| B→A 静态回归 | 3/3 成功 |
| 总成功率 | 6/6 = 100% |
| Nav2 action status | 6 次均为 `4`（SUCCEEDED） |
| Gazebo 非地面 contacts | 6 次均为 `false` |
| MPPI 插件 | 6 次均完成加载、configure、activate |
| crash / NaN / 插件加载错误 | 未发现 |
| 实测控制周期 | P95 为 0.10024–0.10028 s，约为 10 Hz 配置周期的 1.0024–1.0028 倍 |
| 参数、轨迹、cmd_vel、rosbag 元数据 | 6 次均完整 |

## 2. 本轮验证的内容与边界

本轮只验证静态场景中的 Reactive MPPI：

```text
模拟 RGB-D → RTAB-Map online SLAM → /map、map→odom
          → map_padder → /nav_map → global/local costmap
          → GoalLineSmacPlanner → MPPIController → /cmd_vel
          → velocity_smoother → collision_monitor → TurtleBot3 Waffle
```

MPPI 只读取当前 costmap 和当前全局路径，不读取未来障碍物位置；当前没有动态障碍、
Oracle future occupancy、PredictionCritic、Transformer 或真实 D435i。`main` 分支和
Gate 0 的 RPP 正式结果没有被替换。

固定条件：

```text
world: src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
profile: reactive_mppi_static
online: true
localization: false
reset_db: true
use_sim_time: true
robot: TurtleBot3 Waffle / DiffDrive
camera: Gazebo simulated RGB-D, no real /scan
inflation_radius: 0.45 m
planner: rtabmap_tb3_nav/GoalLineSmacPlanner + SmacPlanner2D
```

## 3. 冻结的 MPPI 参数

配置文件：[`nav2_mppi_reactive_10hz_params.yaml`](../configs/nav2_mppi_reactive_10hz_params.yaml)

| 参数 | 冻结值 |
|---|---:|
| `controller_frequency` | 10.0 Hz |
| `time_steps` | 30 |
| `model_dt` | 0.10 s |
| `H_mppi = time_steps × model_dt` | 3.00 s |
| `batch_size` | 500 |
| `iteration_count` | 1 |
| `motion_model` | `DiffDrive` |
| `vx_min / vx_max` | -0.12 / 0.28 m/s |
| `wz_max` | 0.90 rad/s |
| `visualize` | false |
| `CostCritic.consider_footprint` | false |
| `ObstaclesCritic.consider_footprint` | false |
| `inflation_radius` | 0.45 m |

正式 critics 顺序为：

```text
ConstraintCritic, CostCritic, ObstaclesCritic, GoalCritic,
GoalAngleCritic, PathAlignCritic, PathFollowCritic,
PathAngleCritic, PreferForwardCritic
```

两个 MPPI cost critic 在本轮采用圆形近似以满足本机 CPU 实时性；Nav2 costmap、
MPPI 的障碍代价、底盘碰撞检查和独立 `collision_monitor` 仍然工作。多边形 footprint
critic 的性能影响被单独记录，未混入本基线结论。

## 4. 六次正式结果

机器可读汇总：[`mppi_static.csv`](../gate1/mppi_static.csv)。每行对应一个独立结果目录。

| 用例 | status | 墙钟时间 (s) | 仿真时间 (s) | Gazebo 轨迹 (m) | 末端 map 误差 (m) | contacts | 控制 P95 周期 (s) |
|---|---:|---:|---:|---:|---:|---|---:|
| A→B run_01 | 4 | 93.354 | 75.7 | 18.165 | 0.355 | false | 0.100260 |
| A→B run_02 | 4 | 206.983 | 166.0 | 18.374 | 0.052 | false | 0.100271 |
| A→B run_03 | 4 | 217.686 | 175.9 | 18.384 | 0.071 | false | 0.100279 |
| B→A run_01 | 4 | 237.412 | 171.8 | 18.926 | 0.063 | false | 0.100244 |
| B→A run_02 | 4 | 111.076 | 90.0 | 19.232 | 0.211 | false | 0.100281 |
| B→A run_03 | 4 | 97.738 | 78.4 | 18.288 | 0.379 | false | 0.100250 |

### 4.1 统计摘要

| 指标 | MPPI 六次平均 | MPPI 中位数 | 最小–最大 |
|---|---:|---:|---:|
| 墙钟时间 (s) | 160.708 | 159.030 | 93.354–237.412 |
| 仿真时间 (s) | 126.300 | 128.000 | 75.7–175.9 |
| Gazebo 轨迹 (m) | 18.562 | 18.379 | 18.165–19.232 |
| 末端 map 误差 (m) | 0.188 | 0.141 | 0.052–0.379 |

按方向：

| 方向 | 成功率 | 墙钟平均 (s) | 仿真平均 (s) | 轨迹平均 (m) | 末端误差平均 (m) |
|---|---:|---:|---:|---:|---:|
| A→B | 3/3 | 172.674 | 139.2 | 18.308 | 0.159 |
| B→A | 3/3 | 148.742 | 113.4 | 18.815 | 0.218 |

## 5. 与 Gate 0 RPP 基线的比较

Gate 0 数据来自同一静态世界、同一在线 RGB-D/RTAB-Map 链路和 `adaptive_goal_line_045`
profile；其六次墙钟平均为 91.788 s、中位数为 92.326 s，Gazebo 轨迹平均为
17.751 m。

因此当前 Reactive MPPI 的：

- 墙钟平均约 160.708 s，较 RPP 平均增加约 75%；
- 墙钟中位数约 159.030 s，较 RPP 中位数增加约 72%；
- Gazebo 轨迹平均约 18.562 m，较 RPP 增加约 4.6%；
- 成功率和物理安全证据保持相同的 100% / 零非地面 contacts。

这说明当前 MPPI 是“可运行的安全静态 Reactive baseline”，但不是效率基线。任务书的
“中位导航时间不宜恶化超过 25%”属于质量目标，本轮未达到，必须在后续研究报告中诚实
保留，不能把 Gate 1 描述为性能提升。

## 6. 恢复与性能审计

六次中有 3 次出现 `controller_server: Failed to make progress`，随后由行为树执行
恢复并最终到达：

| run | 观察到的行为 |
|---|---|
| A→B run_02 | 4 次 progress failure，出现 spin recovery |
| A→B run_03 | 4 次 progress failure，出现 spin recovery |
| B→A run_01 | 4 次 progress failure，出现 spin recovery |

这 3 次没有造成 action 失败、碰撞或 controller crash，但解释了时间分布的双峰和
明显变慢。其余 3 次没有该恢复序列。控制频率统计仍稳定，说明主要问题不是 MPPI
周期失控，而是在线地图更新、局部可行性/进度判定与恢复行为之间的交互。

后续动态 Gate 必须记录 `Failed to make progress`、spin/backup 次数和停滞时长；在
没有单独实验前，不能把它们归因于 Oracle 或未来信息。

## 7. 失败候选与版本审计

失败或无效候选均未删除，保存在 `experiments/oracle_mppi/gate1/` 及其
`runner_audit/` 中：

| 目录/类别 | 判定 | 原因 |
|---|---|---|
| `smoke_A_to_B`、`smoke_A_to_B_rerun` | 启动失败样本 | 容器内使用了旧安装 launch，属于复现/构建问题 |
| `smoke_A_to_B_15hz` | 无效候选 | 生成的 YAML 缺少 `global_costmap`，未纳入正式统计 |
| `smoke_A_to_B_footprint` | 性能失败样本 | 多边形 footprint critic 计算负载过高，运行中止/卡住 |
| 早期 20 Hz、`batch_size=1000` 候选 | 性能失败样本 | 控制周期超时并在终点前恢复循环 |
| 本次旧工作树 10 Hz 六次 | 归档样本 | 结果有效但运行前代码未提交，已移入 `runner_audit` 做 provenance 修复 |

这些目录是实验审计资产，不能作为正式成功率统计，也不能删除来改善结果。

## 8. 轨迹图索引

每个正式 run 都包含左右双栏 `trajectory_comparison.png`：左侧为 Gazebo 世界真值与
轨迹，右侧为 map/costmap 视角与轨迹。

| 用例 | 轨迹图 |
|---|---|
| A→B run_01 | [trajectory_comparison.png](../gate1/case_A_to_B/run_01/trajectory_comparison.png) |
| A→B run_02 | [trajectory_comparison.png](../gate1/case_A_to_B/run_02/trajectory_comparison.png) |
| A→B run_03 | [trajectory_comparison.png](../gate1/case_A_to_B/run_03/trajectory_comparison.png) |
| B→A run_01 | [trajectory_comparison.png](../gate1/case_B_to_A/run_01/trajectory_comparison.png) |
| B→A run_02 | [trajectory_comparison.png](../gate1/case_B_to_A/run_02/trajectory_comparison.png) |
| B→A run_03 | [trajectory_comparison.png](../gate1/case_B_to_A/run_03/trajectory_comparison.png) |

每个目录还包括：

```text
metrics.yaml
experiment.yaml
<实际使用的 MPPI YAML 快照>
trajectory.csv
gazebo_trajectory.csv
cmd_vel.csv
control_frequency.yaml
world.sdf
rosbag/metadata.yaml
gazebo_contacts.log.gz（本机原始流）
```

## 9. 复现命令

### 9.1 构建

```bash
cd /home/w417/RTAB-Map
git switch exp/oracle-mppi-2026-08-27
docker compose up -d
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   cd /workspaces/rtabmap_tb3_nav && \
   colcon build --symlink-install'
```

### 9.2 单次新目录复现

```bash
./experiments/oracle_mppi/scripts/run_gate1_leg.sh \
  --start-x -8.5 --start-y 0.0 \
  --x 8.5 --y 0.0 --yaw 0.0 \
  --profile reactive_mppi_static \
  --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
  --expected-control-period 0.1 \
  --settle-seconds 5.0 --startup-timeout 90 --contact-timeout 420 \
  --label experiments/oracle_mppi/gate1/reproduction_A_to_B_$(date +%Y%m%d_%H%M%S)
```

### 9.3 完整六次矩阵

```bash
./experiments/oracle_mppi/scripts/run_gate1_matrix.sh \
  --profile reactive_mppi_static \
  --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
  --expected-control-period 0.1 \
  --settle-seconds 5.0 --startup-timeout 90 --contact-timeout 420
```

若要明确重新跑已有 run，必须使用 `--rerun-all`；它会将旧目录移动到带时间戳的
`gate1/runner_audit/`，不会覆盖或删除原证据。

## 10. 进入 Gate 2 的边界

允许进入 Gate 2，因为 Reactive MPPI 已满足 Gate 1 硬验收。进入 Gate 2 后必须：

1. 保持本 YAML、速度、footprint、costmap 和 `reactive_mppi_static` 不变；
2. 新建具有真实 collision geometry 的动态障碍场景 S1–S4；
3. 先验证动态真值轨迹、contacts 和最小距离，再实现 Oracle publisher；
4. Reactive 与 Oracle 使用完全相同的当前条件，唯一增加未来信息；
5. 把本轮恢复次数和时间方差作为 baseline 指标，不得事后删掉慢样本。

本报告的结论只到 Gate 1 为止，不允许据此写出 Oracle 有收益或可以进入 Transformer
训练的结论。
