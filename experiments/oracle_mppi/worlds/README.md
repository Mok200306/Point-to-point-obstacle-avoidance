# Gate 2 动态场景

Gate 2 复用已冻结的静态大场景
`src/rtabmap_tb3_nav/worlds/indoor_obstacle_course_large.world`，不修改它。
每次实验启动后，通过 `gazebo_ros/spawn_entity.py` 载入
`oracle_dynamic_obstacle.sdf`；该模型有独立的 box collision geometry，随后由
`dynamic_obstacle_controller.py` 依据场景 YAML 的确定性 waypoint schedule 更新位姿。

四个场景描述位于 `experiments/oracle_mppi/configs/scenarios/`：

| 场景 | 类型 | medium 设计 |
|---|---|---|
| S1 | 横穿 | 从机器人走廊侧面横向穿过 |
| S2 | 对向 | 在下侧走廊沿 x 轴反向接近 |
| S3 | 斜穿 | 从上侧斜向进入未来路线 |
| S4 | 停-走/变速 | 横穿过程中短暂停止后继续 |

每个 YAML 都带 `easy / medium / hard` 时间缩放与起始延迟；正式 Gate 2 smoke 使用
`medium`。动态障碍位置的脚本真值和 Gazebo 实际状态都写入每个 run 的
`dynamic_groundtruth.csv`，不会使用 `/gazebo/model_states` 当前速度做未来外推。
