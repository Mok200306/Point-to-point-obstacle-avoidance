# WATER SDK 完整公开 API 签名总表

> 本文件由实际 `WaterChassis` 源码提取，用来确认没有漏掉公开方法。详细用途见 `WATER底盘SDK使用手册.md`。

构造函数补充参数：`stop_on_close=True`。ROS bridge 的只读诊断模式会显式传入
`False`，避免关闭诊断节点时抢占其他控制器；真正拥有运动控制权的实例应保持默认
值 `True`，由 `close()` 正常停车。

## `get_config()`

```python
get_config(self) -> dict[str, Any]
```

返回当前已加载配置的一份副本。修改返回值不会直接修改 SDK 内部配置。

## `set_feedback()`

```python
set_feedback(self, enabled: bool, callback: FeedbackCallback | None=None) -> None
```

打开或关闭后续阻塞式动作的默认实时反馈，也可同时设置反馈回调函数。

## `close()`

```python
close(self) -> None
```

先执行正常停车，再关闭客户端；仅终止由当前对象自动启动的 Gateway。

## `get_state()`

```python
get_state(self, *, refresh: bool=False) -> dict[str, Any]
```

获取完整底盘状态。``refresh=True`` 时先主动刷新一次底盘状态。

## `get_health()`

```python
get_health(self) -> dict[str, Any]
```

获取 SDK Gateway 与真实底盘的连接健康状态。

## `get_capabilities()`

```python
get_capabilities(self) -> dict[str, Any]
```

获取当前 SDK 暴露的控制模式、速度限制和状态能力。

## `get_pose()`

```python
get_pose(self, *, refresh: bool=False) -> dict[str, Any]
```

获取地图坐标系位姿：x_m、y_m、yaw_rad、floor。

## `get_velocity()`

```python
get_velocity(self) -> dict[str, Any]
```

获取当前线速度和角速度。

## `get_power()`

```python
get_power(self) -> dict[str, Any]
```

获取电量、电压、电流和充电状态。

## `get_safety()`

```python
get_safety(self) -> dict[str, Any]
```

获取急停、故障和 error_code 等安全状态。

## `get_motion_state()`

```python
get_motion_state(self) -> dict[str, Any]
```

获取当前运动模式、厂家导航状态和 DIRECT 动作进度。

## `get_feedback_state()`

```python
get_feedback_state(self) -> dict[str, Any] | None
```

返回当前或最近一次 DIRECT 动作的进度记录。

## `is_ready()`

```python
is_ready(self) -> bool
```

底盘在线、位姿有效且未急停/故障时返回 True。

## `set_velocity()`

```python
set_velocity(self, linear_mps: float, angular_rps: float, *, replace_current: bool=False) -> dict[str, Any]
```

发送一帧线速度/角速度命令。

## `drive_for()`

```python
drive_for(self, linear_mps: float, angular_rps: float, duration_s: float, *, rate_hz: float | None=None, replace_current: bool=False, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

按固定 v/ω 持续运动指定秒数，结束后自动停车。适合简单测试动作。

## `spin_for()`

```python
spin_for(self, angular_rps: float=0.25, duration_s: float=3.0, *, rate_hz: float | None=None, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

原地按固定角速度旋转指定秒数；正值左转，负值右转。

## `rotate_by()`

```python
rotate_by(self, angle_deg: float, *, max_angular_rps: float | None=None, tolerance_deg: float | None=None, timeout_s: float | None=None, rate_hz: float | None=None, replace_current: bool=True, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

基于位姿反馈相对旋转指定角度（度）；正值左转，负值右转。

## `rotate_to()`

```python
rotate_to(self, yaw_deg: float, **kwargs: Any) -> dict[str, Any]
```

旋转到地图坐标系的绝对 yaw 角（度）。

## `drive_distance()`

```python
drive_distance(self, distance_m: float, *, speed_mps: float | None=None, tolerance_m: float | None=None, heading_hold: bool | None=None, max_heading_correction_rps: float | None=None, timeout_s: float | None=None, rate_hz: float | None=None, replace_current: bool=True, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

基于位姿反馈闭环行驶指定距离；正值前进，负值后退。

## `move_relative_direct()`

```python
move_relative_direct(self, heading_deg: float, distance_m: float, *, linear_speed_mps: float | None=None, angular_speed_rps: float | None=None, distance_tolerance_m: float | None=None, angle_tolerance_deg: float | None=None, replace_current: bool=True, timeout_s: float | None=None, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

用两个 DIRECT 闭环完成“相对航向角 + 有符号距离”。

## `move_relative()`

```python
move_relative(self, heading_deg: float, distance_m: float, *, mode: MotionMode='direct', **kwargs: Any) -> dict[str, Any]
```

执行“相对航向角 + 距离”。

## `forward()`

```python
forward(self, distance_m: float, **kwargs: Any) -> dict[str, Any]
```

前进指定距离（米）。

## `backward()`

```python
backward(self, distance_m: float, **kwargs: Any) -> dict[str, Any]
```

后退指定距离（米）。

## `turn_left()`

```python
turn_left(self, angle_deg: float=90.0, **kwargs: Any) -> dict[str, Any]
```

原地向左闭环旋转指定角度（度）。

## `turn_right()`

```python
turn_right(self, angle_deg: float=90.0, **kwargs: Any) -> dict[str, Any]
```

原地向右闭环旋转指定角度（度）。

## `navigate_relative()`

```python
navigate_relative(self, heading_deg: float, distance_m: float, *, wait: bool=True, timeout_s: float | None=None, distance_tolerance_m: float | None=None, yaw_tolerance_deg: float | None=None, max_continuous_retries: int | None=None, replace_current: bool=False, final_yaw_deg: float | None=None, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

## `navigate_to()`

```python
navigate_to(self, x_m: float, y_m: float, yaw_deg: float, *, wait: bool=True, timeout_s: float | None=None, distance_tolerance_m: float | None=None, yaw_tolerance_deg: float | None=None, max_continuous_retries: int | None=None, replace_current: bool=False, feedback: bool | None=None, feedback_callback: FeedbackCallback | None=None) -> dict[str, Any]
```

## `get_task()`

```python
get_task(self, task_id: str) -> dict[str, Any]
```

## `wait_task()`

```python
wait_task(self, task_id: str, **kwargs: Any) -> dict[str, Any]
```

## `cancel_navigation()`

```python
cancel_navigation(self) -> dict[str, Any]
```

## `get_robot_info()`

```python
get_robot_info(self) -> dict[str, Any]
```

## `get_diagnosis()`

```python
get_diagnosis(self) -> dict[str, Any]
```

## `get_navigation_params()`

```python
get_navigation_params(self) -> dict[str, Any]
```

## `set_navigation_speed()`

```python
set_navigation_speed(self, *, linear_mps: float | None=None, angular_rps: float | None=None) -> dict[str, Any]
```

## `get_planned_path()`

```python
get_planned_path(self) -> dict[str, Any]
```

## `query_accessible_point()`

```python
query_accessible_point(self, x_m: float, y_m: float) -> dict[str, Any]
```

## `query_obstacle_distance()`

```python
query_obstacle_distance(self, x_m: float, y_m: float) -> dict[str, Any]
```

## `plan_distance()`

```python
plan_distance(self, start_x: float, start_y: float, start_floor: int, goal_x: float, goal_y: float, goal_floor: int) -> float
```

## `get_map_list()`

```python
get_map_list(self) -> dict[str, Any]
```

## `get_current_map()`

```python
get_current_map(self) -> dict[str, Any]
```

## `stop()`

```python
stop(self) -> dict[str, Any]
```

## `estop()`

```python
estop(self) -> dict[str, Any]
```

## `release_estop()`

```python
release_estop(self) -> dict[str, Any]
```

解除软件急停。实体急停仍需在机器人上物理解除。

---

## 可选中文别名

中文版额外提供以下别名；正式多人项目仍推荐英文主接口。

```text
获取配置
设置反馈
获取状态
获取健康状态
获取能力
获取位姿
获取速度
获取电量
获取安全状态
获取运动状态
获取动作反馈
是否就绪
设置速度
定时运动
定时原地转
相对转向
转到绝对航向
行驶距离
相对移动
前进
后退
左转
右转
相对自主导航
导航到
获取任务
等待任务
取消导航
获取机器人信息
获取诊断
获取导航参数
设置导航速度
获取规划路径
查询可达点
查询障碍距离
规划距离
获取地图列表
获取当前地图
停止
急停
解除急停
```
