# Gate 3：Oracle 未来真值与时空占据发布

日期：2026-08-28
任务书：`/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`
实验分支：`exp/oracle-g3-publisher-2026-08-28`

## 1. 本 Gate 的边界

Gate 3 只建立并验收 Oracle 的未来动态障碍信息接口，不修改 Reactive MPPI 的
控制器、planner、costmap 或 collision monitor，也不实现 PredictionCritic。

Oracle 的含义是：动态障碍的未来位置直接由 Gate 2 使用的确定性 waypoint
schedule 查询得到。它不是根据当前速度外推，也不是读取 Gazebo 当前状态后猜测
未来，更不是 Transformer 预测。这样可以把“未来信息是否有价值”和“预测器是否
准确”两个问题分开。

通过 Gate 3 后，Gate 4 才允许编写 PredictionCritic。Gate 3 失败时应先修复
frame、origin、axis、resolution 或时间语义，不能通过修改 MPPI 参数掩盖接口问题。

## 2. 新增组件

```text
src/oracle_dynamic_nav_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/PredictedOccupancyGrid.msg

src/oracle_prediction_publisher/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/oracle_prediction_publisher
└── oracle_prediction_publisher/
    ├── __init__.py
    ├── grid.py
    ├── trajectory.py
    └── publisher.py
```

消息定义为：

```text
std_msgs/Header header
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
float32 dt
uint32 steps
float32[] data
string source
float32 footprint_half_size_m
float32 risk_padding_m
bool conservative_cell
```

`data` 按 `[steps, height, width]` 的连续 row-major 顺序展开；每个栅格值为
`0.0` 或 `1.0` 的占据/风险标志。`source` 固定为 `oracle`。消息的
`header.stamp` 是本条消息的发布参考时刻 `t_msg`，第 `k` 层代表
`t_msg + k * dt` 附近的未来障碍位置。

当前首版冻结接口参数：

| 参数 | 值 | 说明 |
|---|---:|---|
| frame | `odom` | Gate 2 的世界坐标与 reset 后 odom 基准重合 |
| local grid | 6.0 m × 5.0 m | 从 Nav2 配置读取，不硬编码 cell 数 |
| resolution | 0.05 m | 从 `local_costmap` 配置读取 |
| cells | 120 × 100 | 由物理尺寸 / 分辨率换算 |
| `prediction_dt` | 0.10 s | 预测层时间间隔 |
| `prediction_horizon_s` | 3.0 s | 与 Gate 1 MPPI horizon 对齐 |
| `steps` | 31 | `round(3.0 / 0.10) + 1` |
| `risk_padding_m` | 0.0 m | 与真实 footprint 风险扩张分离，首版不额外扩张 |
| `conservative_cell` | `true` | 栅格化时加入半栅格保守边界 |
| publish period | 0.10 s | 10 Hz |

动态障碍 footprint 当前按 `0.60 m × 0.60 m` box 处理，半尺寸为 0.30 m。
这只是 Gate 3 的动态模型 footprint，不代表已经把它接入 Nav2 代价函数。

## 3. 离线验证

先在仓库根目录执行：

```bash
cd /home/w417/RTAB-Map
python3 experiments/oracle_mppi/scripts/test_gate3_grid_alignment.py \
  --difficulty medium \
  --costmap-name global_costmap \
  --output experiments/oracle_mppi/gate3/oracle_grid_alignment_test.csv
```

测试会遍历 `configs/scenarios/s*.yaml` 的 S1～S4，在
`tau = 0, 0.5, 1.0, 1.5 s` 查询 waypoint schedule，将 box 栅格化后计算占据
栅格中心与 schedule 中心的距离。通过阈值为一个栅格对角线
`resolution * sqrt(2)`；所有场景和所有时间点都必须通过。

验证滚动 local costmap 的几何实现时，使用随 schedule 中心移动的测试原点：

```bash
python3 experiments/oracle_mppi/scripts/test_gate3_grid_alignment.py \
  --difficulty medium \
  --costmap-name local_costmap \
  --origin-mode schedule_center \
  --output experiments/oracle_mppi/gate3/oracle_grid_alignment_local_test.csv
```

如果给 local costmap 传入一个固定的全局原点，障碍物在窗口之外时得到空栅格是
预期行为，不是坐标错位；这类诊断输出放在 `gate3/diagnostics/`，不作为硬验收。

生成一张可视化图：

```bash
python3 experiments/oracle_mppi/scripts/plot_gate3_oracle_grid.py \
  --scenario experiments/oracle_mppi/configs/scenarios/s2_oncoming.yaml \
  --difficulty medium \
  --dt 0.5 \
  --horizon-s 1.5 \
  --output experiments/oracle_mppi/gate3/oracle_grid_layers_s2.png
```

图中每一列是一个未来时间层，红色栅格是 Oracle 占据，黑色叉号是同一时间的
schedule 中心。它用于检查 frame、origin、轴向和栅格分辨率；不是导航轨迹图。

## 4. ROS 2 publisher smoke

宿主机启动容器并构建两个新包：

```bash
cd /home/w417/RTAB-Map
sg docker -c './scripts/start.sh'
sg docker -c 'docker compose exec -T ros2 bash -lc \
  "source /opt/ros/humble/setup.bash && \
   cd /workspaces/rtabmap_tb3_nav && \
   colcon build --symlink-install"'
```

在容器终端 source 工作区：

```bash
docker compose exec ros2 bash
source /opt/ros/humble/setup.bash
source /workspaces/rtabmap_tb3_nav/install/setup.bash
```

如果已有 Gazebo/RTAB-Map/Nav2 运行实例，另开一个容器终端发布 S1：

```bash
ros2 run oracle_prediction_publisher oracle_prediction_publisher \
  --ros-args \
  --params-file /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml \
  -p scenario_file:=/workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/configs/scenarios/s1_crossing.yaml \
  -p nav2_params_file:=/workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/configs/nav2_mppi_reactive_10hz_params.yaml
```

检查主题和消息：

```bash
ros2 topic list | grep -E '^/oracle/predicted_occupancy$'
ros2 topic hz /oracle/predicted_occupancy
ros2 topic echo /oracle/predicted_occupancy --once
```

也可以使用只读字段检查器：

```bash
python3 /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/scripts/validate_gate3_ros_message.py \
  --topic /oracle/predicted_occupancy
```

应检查：

1. `header.frame_id == odom`；
2. `header.stamp` 使用 ROS 仿真时钟，且 `/clock` 存在；
3. `resolution == 0.05`、`width == 120`、`height == 100`；
4. `dt == 0.10`、`steps == 31`；
5. `len(data) == steps * width * height`；
6. `source == oracle`；
7. `origin` 随 rolling local grid 的 robot-centered 原点变化；
8. publisher 关闭后，Reactive MPPI 的 topic、参数和控制链路不受影响。

本阶段不要求把该 topic 显示成 Nav2 的标准 `nav_msgs/OccupancyGrid`，也不要求
RViz 自动显示它。后续如需 RViz 显示，应另写只读可视化适配器，不能改变接口的
时间层语义。

## 5. 时间语义

publisher 的每条消息建立自己的 `header.stamp = t_msg`。第 `k` 层对应：

```text
t_layer(k) = t_msg + k * dt
```

scenario schedule 的绝对参考只用于确定障碍在仿真时间轴上的位置；它不替代消息
时间戳。Gate 4 的 PredictionCritic 评价候选轨迹时必须使用：

```text
tau_k = (t_eval - t_msg) + k * model_dt
```

因此 Gate 3 不允许把一条当前静态栅格复制到所有未来层，也不允许用 wall time
与 sim time 混算。

## 6. 关闭与回退

publisher 是独立节点。按 `Ctrl-C` 关闭它不会修改：

- `nav2_mppi_reactive_10hz_params.yaml`；
- `controller_server` / `planner_server`；
- RTAB-Map、RGB-D costmap 和 collision monitor；
- Gate 0～Gate 2 的任何结果目录。

因此关闭 publisher 后系统仍应回到 Gate 1 的 Reactive MPPI 行为。Gate 3 本身
不提供“Oracle 导航成功率”，也不改变 Gate 2 中已记录的 Reactive 动态碰撞结论。

## 7. 证据文件

通过后本目录至少应包含：

```text
gate3/
├── README.md
├── oracle_grid_alignment_test.csv
└── oracle_grid_layers_s2.png
```

正式结论写入：

```text
experiments/oracle_mppi/reports/GATE3_REPORT_2026-08-28.md
```

只有离线四场景 × 四个时间点全部通过、ROS 2 publisher smoke 的消息字段和时间
语义全部通过，且关闭 publisher 后 Reactive 回退检查通过，才允许提交本分支并
创建 `oracle-g3-pass` 标签。
