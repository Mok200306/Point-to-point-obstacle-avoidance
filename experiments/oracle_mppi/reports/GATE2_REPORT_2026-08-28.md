# Gate 2 报告：动态场景与碰撞真值

日期：2026-08-28
任务书：/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx
实验分支：exp/oracle-g2-dynamic-2026-08-28
本轮代码基线：bfab28f（动态障碍 spawn 冷启动恢复）
主分支：main@6947245（本轮未修改）

## 1. Gate 结论

### Gate 2：PASS（动态环境与证据链路）

本 Gate 已证明：

1. S1 横穿、S2 对向、S3 斜穿、S4 停-走/变速四类动态场景已经参数化，可以由脚本自动启动、驱动和结束。
2. 动态障碍是带真实 box collision geometry 的 Gazebo 实体，而不是只有 visual 的假障碍；真实机器人—动态障碍接触可以在 Gazebo contacts 中捕获。
3. 完整运行可以从 Gazebo model states 计算每个时刻的机器人—动态障碍几何 clearance，并保存计划轨迹和 Gazebo 实际轨迹。
4. S1、S2 medium 场景产生了可观测的动态交互；S2 中还出现了 Reactive MPPI 的真实机器人—动态障碍碰撞。

这里的 PASS 只针对任务书 Gate 2 的“场景公平、可碰撞、可量化、证据可复核”要求，不等于 Reactive MPPI 已经通过动态避障。正式结果中 S2 有 2/3 次动态碰撞，S4 有 1/3 次动态碰撞；失败样本是后续 Oracle 对照的必要基线，已全部保留。

当前状态应写成：

    动态场景、真实碰撞、距离计算和证据采集链路：PASS
    Reactive MPPI 动态导航性能：S2/S4 仍有失败，待 Gate 5/6 对照
    Oracle publisher：未实现
    PredictionCritic：未实现
    Transformer：未实现

因此允许进入 Gate 3，但不能把当前结果写成“动态导航 100% 成功”，也不能据此提前决定训练 Transformer。

## 2. 任务书硬验收逐项核对

| Gate 2 硬验收 | 证据 | 判定 |
|---|---|---|
| 4 类场景能够自动启动、自动结束，并得到可控动态轨迹 | configs/scenarios/s1_crossing.yaml 至 s4_stop_go.yaml；正式矩阵和补跑状态表；各完整 run 的 dynamic_groundtruth.csv | PASS |
| 动态模型具有有效 collision geometry | worlds/oracle_dynamic_obstacle.sdf 的实体 box collision | PASS |
| 人工/实际碰撞可以被 contacts 捕获 | S1 smoke_04、S2 formal_02/formal_03、S4 formal_02 的 metrics.yaml 与本地 gazebo_contacts.log.gz | PASS |
| 可从 Gazebo 真值计算每时刻最小几何距离 | 各完整 run 的 dynamic_groundtruth.csv 和 dynamic_summary.yaml | PASS |
| S1/S2 medium 存在可观察动态交互 | S1 formal_02 最小 clearance 0.278 m；S2 formal_01 最小 clearance 0.527 m，formal_02/03 发生真实碰撞 | PASS |

“每个场景 3 次 smoke 且轨迹标准差较小”在任务书中属于目标项，不是 Gate 2 硬验收。早期 smoke 的启动异常和补跑均保留，正式 medium 数据用于主判定。

## 3. 实验边界与公平性

本 Gate 使用 Gate 1 冻结的 Reactive MPPI，不发布未来障碍物信息：

    profile: reactive_mppi_static
    nav2 params: experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml
    controller_frequency: 10 Hz
    time_steps: 30
    model_dt: 0.10 s
    MPPI horizon: 3.00 s
    batch_size: 500
    motion_model: DiffDrive
    vx_min / vx_max: -0.12 / 0.28 m/s
    wz_max: 0.90 rad/s
    inflation_radius: 0.45 m
    online: true
    localization: false
    reset_db: true
    use_sim_time: true

所有场景复用同一个静态大场景和目标：

    world: src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world
    start: (-8.5, 0.0)
    goal:  ( 8.5, 0.0)
    dynamic model: oracle_dynamic_obstacle

动态障碍控制器只执行场景 YAML 中的确定性 waypoint schedule 并记录 Gazebo 真值。它：

- 不向 Nav2 发布未来占据；
- 不使用当前速度外推未来；
- 不向 Reactive MPPI 注入未来信息；
- 通过 /gazebo/set_entity_state 驱动真实可碰撞模型；
- 使用 ROS 2 仿真时间记录计划位置、实际位置、机器人位置和 clearance。

后续 Reactive 与 Oracle 对照必须复用相同的 world、场景 YAML、起终点、动态轨迹、速度和确定性条件，唯一增加未来信息开关。

## 4. 动态模型和距离证据

worlds/oracle_dynamic_obstacle.sdf 中的动态代理是 0.60 m × 0.60 m × 0.90 m 的非静态 box：

    <model name="oracle_dynamic_obstacle">
      <static>false</static>
      <collision name="collision">
        <geometry><box><size>0.60 0.60 0.90</size></box></geometry>
      </collision>
    </model>

控制器将机器人近似为 0.66 m × 0.54 m 旋转矩形，将障碍近似为 0.60 m × 0.60 m 旋转矩形，并用二维 polygon intersection / edge distance 计算边界 clearance：

- 多边形相交或重叠：clearance = 0.0 m；
- 不相交：取两个多边形边界的最小距离；
- Gazebo contacts 是物理接触证据，几何 clearance 是连续距离证据，两者分别记录。

计划轨迹与 Gazebo 实际轨迹的最大位置误差由 /gazebo/model_states 计算。正式完整运行中的最大误差约为 0.009–0.021 m，说明驱动轨迹可复核；该误差仍保留在数据中，不写成零误差。

## 5. 四类场景

| 场景 | 动态行为 | 验证问题 | medium schedule 摘要 |
|---|---|---|---|
| S1 crossing | 障碍在北侧绕行通道横向移动并停留 | 横穿冲突能否产生减速、绕行或近距离交互 | t=25→45 s：(-4.0,0.9) 到 (-7.0,0.9) |
| S2 oncoming | 障碍沿绕行通道从东向西迎面运动 | 对向接近时是否提前侧向或调整速度 | t=0→70 s：(7.5,0.6) 到 (-7.5,0.6) |
| S3 diagonal | 障碍从西北方向斜向进入未来通道 | 避免只验证正交横穿 | t=10→42→58 s 分段斜向运动 |
| S4 stop_go | 障碍分段移动并短暂停止/变速 | 停走变化是否造成时间冲突 | t=12→20→28→38 s 分段南北移动 |

每个 YAML 都提供 easy / medium / hard 三档 time_scale 和 start_delay_s。正式主结果使用 medium，轨迹由 YAML 固定，没有运行内随机化。

## 6. 正式 medium 结果

正式矩阵在 commit 079b0e9 上开始执行。S4 formal_03 发生启动生命周期竞态，随后通过不改变导航参数和场景轨迹的 retry 修复，并使用新目录补跑。之后又加入动态障碍 spawn 冷启动恢复；没有覆盖原目录。

### 6.1 S1 横穿：3/3 到达，0/3 动态碰撞

| run | status | 成功 | 仿真时间 (s) | Gazebo 路径 (m) | 末端误差 (m) | 最小 clearance (m) | 脚本—Gazebo误差 (m) | 动态 contact |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| formal_01 | 4 | 是 | 163.3 | 18.596 | 0.036 | 1.756 | 0.015 | 否 |
| formal_02 | 4 | 是 | 158.7 | 18.230 | 0.031 | 0.278 | 0.015 | 否 |
| formal_03 | 4 | 是 | 164.5 | 18.313 | 0.034 | 1.523 | 0.015 | 否 |

formal_02 接近动态障碍但未接触，证明 S1 不是双方永远错开。

### 6.2 S2 对向：1/3 无碰撞，2/3 真实动态碰撞

| run | status | 成功 | 仿真时间 (s) | Gazebo 路径 (m) | 末端误差 (m) | 动态 clearance (m) | 动态 contact |
|---|---:|---:|---:|---:|---:|---:|---|
| formal_01 | 4 | 是 | 159.6 | 18.177 | 0.052 | 0.527 | 否 |
| formal_02 | 6 | 否 | 291.0 | 33.621 | 0.252 | 0.000 | 是 |
| formal_03 | 6 | 否 | 182.0 | 26.420 | 17.937 | 0.000 | 是 |

S2 是最重要的 Reactive 动态失败证据。contacts 中出现 waffle::base_link、左右轮与 oracle_dynamic_obstacle::link::collision 的碰撞对；原始 contacts 流在本地 .gz 文件中保留。

### 6.3 S3 斜穿：3/3 到达，0/3 动态碰撞

| run | status | 成功 | 仿真时间 (s) | Gazebo 路径 (m) | 末端误差 (m) | 最小 clearance (m) | 脚本—Gazebo误差 (m) | 动态 contact |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| formal_01 | 4 | 是 | 161.7 | 18.429 | 0.049 | 1.125 | 0.009 | 否 |
| formal_02 | 4 | 是 | 160.9 | 17.712 | 0.028 | 1.434 | 0.009 | 否 |
| formal_03 | 4 | 是 | 158.7 | 17.892 | 0.029 | 1.390 | 0.009 | 否 |

S3 证明动态刺激不局限于水平横穿，并且动态真值记录链路稳定。

### 6.4 S4 停-走/变速

S4 最初的 formal_03 没有进入导航阶段：三个主要 Nav2 节点 active，但 collision_monitor 仍为 inactive [2]。这是启动生命周期竞态，不是导航结果；目录保留。修复后新增 formal_04/formal_05。

| run | status | 成功 | 仿真时间 (s) | Gazebo 路径 (m) | 末端误差 (m) | 最小 clearance (m) | 动态 contact |
|---|---:|---:|---:|---:|---:|---:|---|
| formal_01 | 4 | 是 | 163.8 | 18.385 | 0.063 | 0.799 | 否 |
| formal_02 | 4 | 是 | 338.0 | 24.544 | 0.024 | 0.000 | 是 |
| formal_03 | — | — | — | — | — | — | 启动失败 |
| formal_04 | 4 | 是 | 167.5 | 18.327 | 0.039 | 0.576 | 否 |
| formal_05 | 4 | 是 | 158.9 | 17.602 | 0.028 | 0.392 | 否 |

formal_02 的动态碰撞应作为后续预测式导航的困难基线样本。

## 7. 补跑和工程修复

### 7.1 collision_monitor 生命周期竞态

提交：f01235e fix: retry Gate 2 collision monitor activation

运行器现在会：

1. 检查 /controller_server、/planner_server、/collision_monitor lifecycle；
2. 冷启动时若前两个 active 而 collision monitor 长时间 inactive，只执行一次 configure/activate retry；
3. 仍要求三个节点都为 active [3] 才发送 Goal；
4. 将重试前后状态写入 startup_recovery.txt。

这不是绕过安全节点，也没有改变 MPPI、costmap、世界或障碍轨迹。

### 7.2 动态障碍 spawn 冷启动竞态

提交：bfab28f fix: retry dynamic obstacle spawn on cold start

运行器现在会：

1. 调用 /spawn_entity；
2. 若客户端退出非零，先检查 /gazebo/model_states 中实体是否已经存在；
3. 若实体已存在，接受“客户端超时但模型已生成”；
4. 若实体不存在，只重试一次；
5. 在 experiment.yaml 和 spawn_obstacle.log 记录恢复标志和尝试次数。

该修复没有改变动态模型、导航参数或轨迹 schedule。

### 7.3 补跑状态

| 场景 | 补跑目录 | 结果 |
|---|---|---|
| S1 | recheck_01、postfix2_01 | 自动完成，无动态碰撞 |
| S2 | recheck_01、postfix2_01 | 到达但均发生真实动态碰撞；问题属于 Reactive 动态避障 |
| S3 | recheck_01、recheck_02 | 前者 spawn 失败并保留，后者在 bfab28f 后成功且无动态碰撞 |
| S4 | recheck_01、postfix2_01 | 前者无碰撞，后者发生真实动态碰撞 |

## 8. 汇总和证据目录

机器可读总表：experiments/oracle_mppi/gate2/summary.csv。脚本递归纳入所有带 scenario.yaml 的运行目录，包括不完整运行；不按成功结果筛选。

矩阵状态：

- experiments/oracle_mppi/gate2/matrix_status_20260828_formal.csv
- experiments/oracle_mppi/gate2/matrix_status_20260828_postfix.csv
- experiments/oracle_mppi/gate2/matrix_status_20260828_postfix2.csv

当前总表包含 33 个运行目录，其中 29 个完整证据目录、4 个不完整目录：

- S1_crossing/smoke_01：早期目录缺少动态 ground truth；
- S1_crossing/smoke_02：metrics/动态记录不完整；
- S3_diagonal/recheck_01：动态障碍 spawn 失败；
- S4_stop_go/formal_03：collision_monitor lifecycle 启动失败。

不完整目录仍在 summary.csv 中，并标记 evidence_complete=False 或对应 failure_reason。

完整运行目录的主要证据为：

    scenario.yaml
    experiment.yaml
    metrics.yaml
    dynamic_groundtruth.csv
    dynamic_summary.yaml
    dynamic_trajectory_comparison.png
    trajectory.csv
    gazebo_trajectory.csv
    trajectory_comparison.png
    cmd_vel.csv
    world.sdf
    oracle_dynamic_obstacle.sdf
    nav2_mppi_reactive_10hz_params.yaml
    reproduce_command.sh
    rosbag_info.txt
    gazebo_contacts.log.gz（本地原始流，按 .gitignore 不提交）
    rosbag/（本地原始 bag，按 .gitignore 不提交）

## 9. 轨迹图

每个完整 run 的 dynamic_trajectory_comparison.png 为 Gate 2 双栏证据图：

- 左图：Gazebo 风格俯视图，绘制静态障碍、机器人真值轨迹、动态障碍计划轨迹和实际轨迹，并标出起点、终点和最小 clearance 时刻；
- 右图：动态障碍与机器人边界 clearance 随仿真时间变化，标出 0 m 接触线和 0.50 m 参考线。

它不是 RViz 截图；Gate 2 的关键证据是 Gazebo 真值、动态 CSV 和 contacts。

代表图：

- experiments/oracle_mppi/gate2/S1_crossing/formal_02/dynamic_trajectory_comparison.png
- experiments/oracle_mppi/gate2/S2_oncoming/formal_02/dynamic_trajectory_comparison.png（碰撞）
- experiments/oracle_mppi/gate2/S3_diagonal/formal_02/dynamic_trajectory_comparison.png
- experiments/oracle_mppi/gate2/S4_stop_go/formal_02/dynamic_trajectory_comparison.png（碰撞）

## 10. 复现命令

### 10.1 环境和构建

    cd /home/w417/RTAB-Map
    git switch exp/oracle-g2-dynamic-2026-08-28
    docker compose up -d
    docker compose exec -T ros2 bash -lc \
      'source /opt/ros/humble/setup.bash && \
       cd /workspaces/rtabmap_tb3_nav && \
       colcon build --symlink-install'

### 10.2 单场景新目录

脚本拒绝覆盖非空目录，必须使用新的 label：

    ./experiments/oracle_mppi/scripts/run_gate2_scene.sh \
      --scenario experiments/oracle_mppi/configs/scenarios/s1_crossing.yaml \
      --difficulty medium \
      --profile reactive_mppi_static \
      --nav2-params experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml \
      --label experiments/oracle_mppi/gate2/S1_crossing/reproduction_$(date +%Y%m%d_%H%M%S)

### 10.3 四场景矩阵

    ./experiments/oracle_mppi/scripts/run_gate2_matrix.sh \
      --difficulty medium \
      --run-prefix formal \
      --start-run 1 \
      --runs-per-scenario 3 \
      --status-output experiments/oracle_mppi/gate2/matrix_status_$(date +%Y%m%d_%H%M%S).csv

### 10.4 只重新生成已有图和汇总

    for d in $(find experiments/oracle_mppi/gate2 \
      -mindepth 3 -maxdepth 3 -type f \
      -name dynamic_groundtruth.csv -printf '%h\n' | sort -u); do
      [ -f "$d/dynamic_trajectory_comparison.png" ] || \
        python3 experiments/oracle_mppi/scripts/plot_gate2_dynamic_run.py --run "$d" || true
    done

    python3 experiments/oracle_mppi/scripts/summarize_gate2.py \
      --root experiments/oracle_mppi/gate2 \
      --output experiments/oracle_mppi/gate2/summary.csv

## 11. 当前不能宣称的内容

以下内容必须留到后续 Gate：

- 没有 predicted_occupancy topic；
- 没有 Oracle publisher；
- 没有 PredictionCritic；
- 没有时间对齐 cost；
- 没有 Reactive vs Oracle paired 对照；
- 没有 Gate 5 的 10+10 smoke runs；
- 没有 Gate 6 的至少 80 个正式 runs；
- 没有最终 GO/NO-GO 研究结论；
- 不得据此决定训练 Transformer。

## 12. 进入 Gate 3

Gate 2 允许进入 Gate 3。下一步只实现任务书规定的 Oracle 真值接口，不修改冻结的 Reactive baseline：

1. 新建 oracle_dynamic_nav_msgs 消息包，冻结 frame、resolution、origin、dt、steps 和 source 字段；
2. 新建 Oracle publisher，从同一份 waypoint schedule 查询任意未来时刻，不从当前速度外推；
3. 对 τ=0、0.5、1.0、1.5 s 做离线/在线空间对齐测试；
4. 验证 future grid 与 Gazebo 计划位置误差不超过一个栅格；
5. 关闭 publisher 时确认系统仍退化为当前 Reactive MPPI；
6. Gate 3 全部硬验收通过后，才允许实现 PredictionCritic。

若 Gate 3 任意硬验收失败，不创建 Gate 3 标签，也不进入 Gate 4。

## 13. Git provenance

本轮相关提交：

    079b0e9 exp: prepare Gate 2 dynamic scenario matrix
    f01235e fix: retry Gate 2 collision monitor activation
    bfab28f fix: retry dynamic obstacle spawn on cold start

大型 gazebo_contacts.log.gz 和 rosbag 按 .gitignore 保留在本机；metrics.yaml、experiment.yaml、CSV、图、参数快照和 rosbag_info.txt 用于 Git 复核。失败目录不删除、不覆盖。
