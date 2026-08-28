# Gate 3 报告：Oracle 未来真值与时空占据发布

日期：2026-08-28
任务书：`/home/w417/文档/Oracle预测式导航生死实验_分阶段执行任务书_v1.docx`
实验分支：`exp/oracle-g3-publisher-2026-08-28`
Gate 2 父提交：`e90ef85`（`oracle-g2-pass`）
主分支：`main@6947245`，本轮未修改

## 1. Gate 结论

### PASS：Oracle 接口硬验收

本 Gate 已证明：

1. S1～S4 的确定性动态障碍 schedule 可以在指定未来时刻查询；
2. 动态 box footprint 可以按 Nav2 costmap 的物理范围和分辨率栅格化；
3. global 固定包络和 local rolling-window 两种空间语义均通过离线对齐；
4. ROS 2 publisher 发布了完整的时间、frame、栅格尺寸和数据长度；
5. 关闭 publisher 后 RTAB-Map、Nav2 Reactive MPPI 和 collision monitor 仍保持运行。

此 PASS 只表示“Oracle 未来信息接口正确、可复现、可独立启停”。本 Gate 没有将
Oracle 接入 MPPI，也没有实现 PredictionCritic，因此不能写成 Oracle 动态导航
性能已经通过。Gate 2 中 Reactive MPPI 在 S2/S4 的真实动态碰撞结论保持不变。

## 2. 实现范围

新增消息包和 publisher：

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

实验配置和检查脚本：

```text
experiments/oracle_mppi/configs/oracle_publisher_gate3.yaml
experiments/oracle_mppi/scripts/test_gate3_grid_alignment.py
experiments/oracle_mppi/scripts/plot_gate3_oracle_grid.py
experiments/oracle_mppi/scripts/validate_gate3_ros_message.py
```

消息类型为 `oracle_dynamic_nav_msgs/msg/PredictedOccupancyGrid`，主题为
`/oracle/predicted_occupancy`：

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

`data` 按 `[steps, height, width]` row-major 展开。每层是动态障碍真实 footprint
的二值占据；`source=oracle`。人为风险扩张单独由 `risk_padding_m` 表达，首版为
0.0 m，没有把真实占据和安全核混合。

## 3. 冻结接口参数

| 参数 | 值 |
|---|---:|
| message frame | `odom` |
| local physical size | 6.0 m × 5.0 m |
| local cells | 120 × 100 |
| resolution | 0.05 m |
| prediction dt | 0.10 s |
| prediction horizon | 3.0 s |
| prediction steps | 31 |
| publish period | 0.10 s |
| dynamic footprint | 0.60 m × 0.60 m |
| risk padding | 0.0 m |
| source | deterministic waypoint schedule |

publisher 使用真实机器人位置计算 rolling-window 原点：

```text
origin_x = robot_x - grid_width_m / 2
origin_y = robot_y - grid_height_m / 2
```

Gate 2 的 schedule 与 reset 后 odom 坐标基准一致，所以首版直接使用 `odom`。
真实机器人或 `map` 适配必须提供并验证显式 TF，不能默认坐标系恒等。

## 4. 离线空间对齐

### 4.1 global 固定包络

命令：

```bash
cd /home/w417/RTAB-Map
python3 experiments/oracle_mppi/scripts/test_gate3_grid_alignment.py \
  --difficulty medium \
  --costmap-name global_costmap \
  --output experiments/oracle_mppi/gate3/oracle_grid_alignment_test.csv
```

结果：

```text
rows=16
scenarios=S1_crossing,S2_oncoming,S3_diagonal,S4_stop_go
tau=0.0,0.5,1.0,1.5 s
grid=480x340 @ 0.050 m
max_centroid_error_m=1.11e-15
passed=True
```

16 个检查点全部通过。global costmap 的固定包络为 24 m × 17 m，原点为
`(-12.0, -8.5)`，来自当前 Reactive MPPI 配置。

### 4.2 local rolling-window

命令：

```bash
python3 experiments/oracle_mppi/scripts/test_gate3_grid_alignment.py \
  --difficulty medium \
  --costmap-name local_costmap \
  --origin-mode schedule_center \
  --output experiments/oracle_mppi/gate3/oracle_grid_alignment_local_test.csv
```

结果：

```text
rows=16
grid=120x100 @ 0.050 m
max_centroid_error_m=0.000000
passed=True
```

这里的 `schedule_center` 仅用于把障碍放进窗口，验证 local 栅格的轴向、分辨率
和栅格化几何；live publisher 使用机器人位置作为原点，不使用障碍物位置移动
窗口。若把 local costmap 错当作固定 global 原点，障碍在窗口外而输出空栅格是
正确裁剪，不是坐标错位。对应诊断保存于 `gate3/diagnostics/`。

### 4.3 easy/medium/hard 附加结果

| difficulty | global | local rolling-window |
|---|---:|---:|
| easy | 16/16 PASS | 16/16 PASS |
| medium | 16/16 PASS | 16/16 PASS |
| hard | 16/16 PASS | 16/16 PASS |

这些 CSV 证明对齐逻辑不依赖 medium 的 start delay 或 time scale。

## 5. 可视化证据

每个场景都生成了未来层图：红色栅格为 Oracle 占据，黑色叉号为对应 schedule
中心。

- [S1 图](../gate3/oracle_grid_layers_s1.png)
- [S2 图](../gate3/oracle_grid_layers_s2.png)
- [S3 图](../gate3/oracle_grid_layers_s3.png)
- [S4 图](../gate3/oracle_grid_layers_s4.png)

这些图用于验证空间落点、原点、轴向和分辨率，不是机器人导航轨迹，也没有向
Nav2 costmap 写入数据。

## 6. ROS 2 live smoke

构建命令：

```bash
cd /home/w417/RTAB-Map
docker compose up -d
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   cd /workspaces/rtabmap_tb3_nav && \
   colcon build --symlink-install'
```

在 headless Gazebo/RTAB-Map/Nav2 实例中启动独立 S4 publisher 后，使用只读检查器：

```bash
docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && \
   source /workspaces/rtabmap_tb3_nav/install/setup.bash && \
   python3 /workspaces/rtabmap_tb3_nav/experiments/oracle_mppi/scripts/validate_gate3_ros_message.py \
     --topic /oracle/predicted_occupancy \
     --expected-frame odom --expected-source oracle \
     --expected-resolution 0.05 --expected-width 120 --expected-height 100 \
     --expected-dt 0.10 --expected-steps 31 \
     --min-occupied-cells 1 --timeout-s 10'
```

现场结果：

```text
frame=odom stamp=2160.500000000 grid=120x100@0.050m dt=0.100
steps=31 data_len=372000 occupied_cells=5408 source=oracle
origin=(-11.437, -2.498)
PASS: Gate 3 message interface fields are valid
```

消息字段、仿真时间和 payload 长度均正确；publisher 只发布未来信息，不发送目标
或速度指令。完整记录见 `gate3/ros_smoke_validation.txt`。

## 7. Reactive 回退

关闭独立 publisher 后现场检查：

```text
oracle node: absent
/oracle/predicted_occupancy publisher count: 0
/collision_monitor: active [3]
/controller_server: active [3]
/planner_server: active [3]
/rtabmap, /map, /odom, /cmd_vel, /local_costmap/costmap: still present
PASS
```

因此 publisher 可独立启停，关闭后系统保持 Gate 1 Reactive MPPI 链路。记录见
`gate3/reactive_fallback_validation.txt`。

## 8. 实施期间发现的问题与修复

### 8.1 ROS 2 日志 API

初版使用 ROS 1 风格的 `logger.info('value=%s', value)`，Humble 的
`RcutilsLogger.info()` 不接受位置参数，节点会在首次发布前退出。现已全部改为
预格式化字符串，并统一使用 ROS 2 warning API。

### 8.2 install 环境钩子

一个残留的生成 `.catkin` 标记使新消息包没有进入重新 source 后的
`AMENT_PREFIX_PATH`。该标记仅属于 build/install 生成物，已移到带时间戳的 stale
缓存目录留档；源代码和 Gate 结果没有删除。随后补充 `ament_cmake` build type
导出和消息运行依赖，干净包级缓存重建通过。

fresh shell 已验证：

```text
ros2 pkg prefix oracle_dynamic_nav_msgs
/workspaces/rtabmap_tb3_nav/install/oracle_dynamic_nav_msgs
```

## 9. 交付物

- `src/oracle_dynamic_nav_msgs`：消息接口包；
- `src/oracle_prediction_publisher`：确定性未来占据发布器；
- `configs/oracle_publisher_gate3.yaml`：冻结配置；
- `scripts/test_gate3_grid_alignment.py`：离线对齐测试；
- `scripts/plot_gate3_oracle_grid.py`：未来层可视化；
- `scripts/validate_gate3_ros_message.py`：live 字段检查器；
- `gate3/oracle_grid_alignment_test.csv`：medium global 数据；
- `gate3/oracle_grid_alignment_local_test.csv`：medium local 数据；
- `gate3/oracle_grid_layers_s1.png`～`s4.png`：四场景图；
- `gate3/*validation.txt`：构建、live、回退证据；
- 本报告和 `gate3/README.md`。

## 10. 下一步 Gate 4

现在才允许实现可启停的 PredictionCritic。MPPI 候选轨迹第 `k` 个点必须访问
同一未来时刻的 Oracle 层：

```text
tau_k = (t_eval - t_msg) + k * model_dt
```

Gate 4 先做离线单元测试，证明同一空间位置在不同未来时间层的 cost 不同，再接入
MPPI。Reactive 与 Oracle 的唯一变量应是 PredictionCritic/future topic 开关；
Gate 1 的参数、场景、速度、footprint、随机种子和记录方法必须保持一致。

在 Gate 4 通过前，不训练 Transformer、不修改 Reactive profile、不把单独运行
publisher 写成 Oracle 导航成功，也不覆盖 Gate 2 失败碰撞目录。
