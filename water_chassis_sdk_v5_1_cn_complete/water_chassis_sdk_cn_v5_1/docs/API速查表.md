# WATER SDK API 速查表

## 最常用

| 功能 | 调用 |
|---|---|
| 完整状态 | `robot.get_state()` |
| 位姿 | `robot.get_pose()` |
| 速度 | `robot.get_velocity()` |
| 电量 | `robot.get_power()` |
| 安全状态 | `robot.get_safety()` |
| 是否可运动 | `robot.is_ready()` |
| 前进 | `robot.forward(0.30)` |
| 后退 | `robot.backward(0.20)` |
| 左转 | `robot.turn_left(30)` |
| 右转 | `robot.turn_right(30)` |
| 航向+距离 | `robot.move_relative(20, 0.50)` |
| 实时速度 | `robot.set_velocity(v, w)` |
| 正常停车 | `robot.stop()` |
| 软件急停 | `robot.estop()` |

## 方向和单位

```text
distance_m：m
linear_mps：m/s
angular_rps：rad/s
angle_deg / heading_deg / yaw_deg：度

v > 0 前进，v < 0 后退
w > 0 左转，w < 0 右转
heading_deg > 0 左，< 0 右，0 正前方
```

## 常用运动

```python
robot.forward(0.30)
robot.backward(0.20)
robot.turn_left(30)
robot.turn_right(45)
robot.rotate_by(-20)
robot.rotate_to(90)
robot.drive_distance(0.50)
robot.move_relative(30, 0.50)
```

## 实时规划器

```python
while True:
    v, w = planner()
    robot.set_velocity(v, w)
    time.sleep(0.1)
```

## WATER 自主导航

```python
robot.navigate_relative(0, 1.0)
robot.navigate_to(1.0, 2.0, 90)
robot.cancel_navigation()
```

## 反馈

```python
with WaterChassis(feedback=True) as robot:
    robot.forward(0.5)
```

回调：

```python
def callback(event):
    print(event["progress"])

with WaterChassis(feedback=False, feedback_callback=callback) as robot:
    robot.forward(0.5)
```
