# WATER 底盘 Python SDK V5.1 中文版使用手册

> 面向视觉、规划和控制程序调用者。目标是：不用理解厂家 Socket 协议，不用手动启动多个程序，直接在自己的 Python 项目里创建一个 `WaterChassis` 对象，然后读取状态、控制运动。

---

## 1. 先说最重要的：正常项目到底怎么写

最推荐的写法就是下面这样：

```python
from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        state = robot.get_state(refresh=True)
        print(state)

        robot.forward(0.30)
        robot.turn_left(30)
        robot.stop()


if __name__ == "__main__":
    main()
```

对普通调用者来说，基本只需要理解这一层：

```text
你的视觉 / 规划 / 控制代码
        ↓
WaterChassis
        ↓
WATER 底盘
```

内部的 FastAPI Gateway、TCP Socket、厂家命令、状态订阅、重连、watchdog 都由 SDK 管理。

---

# 2. `with WaterChassis() as robot:` 是不是 Python 的固定格式？

**它是 Python 的标准语法，但不是“必须这么写”的固定格式。**

这叫做 **上下文管理器（context manager）**。

```python
with WaterChassis() as robot:
    robot.forward(0.3)
```

之所以这套 SDK 可以这样写，是因为 `WaterChassis` 类实现了 Python 规定的两个特殊方法：

```python
__enter__()
__exit__()
```

你可以把它理解成：

```text
进入 with
  ↓
创建并连接 robot
  ↓
执行你的代码
  ↓
离开 with
  ↓
自动调用 robot.close()
  ↓
正常停车 + 关闭客户端 + 清理 SDK 自己启动的 Gateway
```

所以它最大的优点不是“写得短”，而是**安全**：哪怕中间代码抛异常，只要 Python 能正常进入 `__exit__()`，SDK 都会尽量先停车再收尾。

### 不用 `with` 也完全可以

等价写法是：

```python
from water_chassis_sdk import WaterChassis

robot = WaterChassis()

try:
    robot.forward(0.30)
finally:
    robot.close()
```

这里的 `finally` 很重要，因为它能保证无论正常结束还是出现异常，都会执行：

```python
robot.close()
```

### 不推荐这样写

```python
robot = WaterChassis()
robot.forward(0.30)
```

然后程序直接结束，不调用 `close()`。

这样并不是说一定会出问题，但资源清理不够规范，也不利于后续集成。

### 视觉/规划循环应该把 `with` 放在循环外面

正确：

```python
with WaterChassis() as robot:
    while True:
        v, w = planner()
        robot.set_velocity(v, w)
```

不要这样：

```python
while True:
    with WaterChassis() as robot:
        robot.set_velocity(v, w)
```

后者会反复创建和关闭 SDK，不适合实时控制。

---

# 3. 放到别人项目里怎么用

建议目录：

```text
my_project/
│
├─ main.py
├─ vision/
├─ planner/
│
└─ water_chassis_sdk/
   ├─ __init__.py
   ├─ water_chassis.py
   ├─ water_api_client.py
   ├─ water_config.py
   ├─ config.json
   ├─ run_gateway.py
   └─ app/
```

然后你的 `main.py` 直接：

```python
from water_chassis_sdk import WaterChassis
```

不需要自己写 `sys.path`，也不需要手动启动 Gateway。

首次使用前安装依赖：

```powershell
python -m pip install -r requirements.txt
```

如果是在这个压缩包根目录，也可以直接双击：

```text
install_requirements.bat
```

---

# 4. 默认网络配置

SDK 默认连接：

```text
WATER 底盘：192.168.10.10:31001
本地 Gateway：127.0.0.1:8080
```

这些都可以在：

```text
water_chassis_sdk/config.json
```

里修改。

也可以在创建对象时直接覆盖：

```python
with WaterChassis(
    robot_host="192.168.10.10",
    robot_port=31001,
) as robot:
    ...
```

通常不需要改。

---

# 5. 坐标和正负方向约定

高层接口统一使用下面的约定：

```text
线速度 linear_mps > 0 ：前进
线速度 linear_mps < 0 ：后退

角速度 angular_rps > 0 ：左转
角速度 angular_rps < 0 ：右转

相对航向 heading_deg = 0 ：当前正前方
相对航向 heading_deg > 0 ：向左
相对航向 heading_deg < 0 ：向右
```

高层角度接口主要使用 **度（deg）**，例如：

```python
robot.turn_left(30)
```

底层实时角速度使用 **rad/s**，例如：

```python
robot.set_velocity(0.10, 0.20)
```

距离单位统一是 **米（m）**，速度统一是 **m/s**。

---

# 6. 创建底盘对象

最简单：

```python
with WaterChassis() as robot:
    ...
```

完整构造参数：

```python
WaterChassis(
    robot_host=None,
    robot_port=None,
    config_path=None,
    gateway_host=None,
    gateway_port=None,
    connect_timeout_s=None,
    auto_start_gateway=None,
    feedback=None,
    feedback_callback=None,
)
```

常用的只有两个：

```python
with WaterChassis(feedback=True) as robot:
    ...
```

或者指定自己的配置文件：

```python
with WaterChassis(config_path="my_water_config.json") as robot:
    ...
```

---

# 7. 状态读取

## 7.1 获取完整状态：`get_state()`

```python
state = robot.get_state()
```

需要强制主动刷新一次底盘状态：

```python
state = robot.get_state(refresh=True)
```

返回结构大致：

```python
{
    "connection": {...},
    "pose": {...},
    "velocity": {...},
    "motion": {...},
    "safety": {...},
    "power": {...},
    "ready_to_move": True,
    ...
}
```

推荐运动前至少检查：

```python
if not robot.is_ready():
    print("底盘当前不允许运动")
    return
```

---

## 7.2 位姿：`get_pose()`

```python
pose = robot.get_pose(refresh=True)

print(pose["x_m"])
print(pose["y_m"])
print(pose["yaw_rad"])
print(pose["floor"])
```

其中：

- `x_m`：地图坐标 x，单位 m
- `y_m`：地图坐标 y，单位 m
- `yaw_rad`：当前航向角，单位 rad
- `floor`：楼层编号

---

## 7.3 速度：`get_velocity()`

```python
velocity = robot.get_velocity()

v = velocity["linear_mps"]
w = velocity["angular_rps"]
```

---

## 7.4 电量：`get_power()`

```python
power = robot.get_power()
print(power)
```

常见字段可能包括：

```text
percent
battery_voltage_v
battery_current_a
charging
```

实际字段以底盘厂家返回为准。

---

## 7.5 安全状态：`get_safety()`

```python
safety = robot.get_safety()

print(safety["estop"])
print(safety["hard_estop"])
print(safety["soft_estop"])
print(safety["error_code"])
print(safety["fault"])
```

---

## 7.6 当前运动状态：`get_motion_state()`

```python
motion = robot.get_motion_state()
print(motion)
```

这里可以看到：

- Gateway 当前是 `IDLE / DIRECT / NAV`
- 厂家 `move_status`
- 厂家 `running_status`
- 当前 DIRECT 动作进度
- 导航剩余直线距离等

---

## 7.7 是否允许运动：`is_ready()`

```python
if robot.is_ready():
    robot.forward(0.30)
```

`is_ready()` 是最适合上层程序做快速判断的接口。

---

# 8. 最常用动作接口

## 8.1 前进：`forward()`

```python
robot.forward(0.30)
```

表示前进 0.30 m。

默认使用位姿反馈闭环，不是简单按时间估算。

临时覆盖速度：

```python
robot.forward(
    0.30,
    speed_mps=0.08,
)
```

---

## 8.2 后退：`backward()`

```python
robot.backward(0.20)
```

表示后退 0.20 m。

---

## 8.3 左转：`turn_left()`

```python
robot.turn_left(30)
```

表示原地向左闭环旋转 30°。

不传角度时默认 90°：

```python
robot.turn_left()
```

---

## 8.4 右转：`turn_right()`

```python
robot.turn_right(45)
```

表示向右旋转 45°。

---

# 9. 航向角 + 距离：`move_relative()`

这是给规划模块最方便的高层接口之一。

```python
robot.move_relative(
    heading_deg=30,
    distance_m=0.50,
)
```

含义：

```text
相对于当前车头先向左转 30°
        ↓
沿新的朝向前进 0.50 m
```

几个例子：

```python
# 当前正前方 0.5 m
robot.move_relative(0, 0.50)

# 左前方 30°，0.5 m
robot.move_relative(30, 0.50)

# 右前方 20°，0.3 m
robot.move_relative(-20, 0.30)
```

默认模式：

```python
mode="direct"
```

也就是使用我们已经真车验证过的 DIRECT 控制链：

```text
闭环转向
  ↓
闭环距离
```

### DIRECT 模式的重要特点

它适合：

- 视觉伺服
- 局部规划
- 短距离动作
- 上层自己负责避障的场景

**DIRECT 本身不等于厂家地图自主避障。**

如果希望 WATER 自己进行地图规划和避障，请使用后面的 `navigate_*()`。

---

# 10. 更底层的闭环动作

## 10.1 行驶指定距离：`drive_distance()`

```python
robot.drive_distance(0.50)
```

前进 0.50 m。

```python
robot.drive_distance(-0.30)
```

后退 0.30 m。

常见覆盖参数：

```python
robot.drive_distance(
    distance_m=0.50,
    speed_mps=0.08,
    tolerance_m=0.02,
    heading_hold=True,
)
```

---

## 10.2 相对旋转：`rotate_by()`

```python
robot.rotate_by(30)
```

左转 30°。

```python
robot.rotate_by(-30)
```

右转 30°。

---

## 10.3 转到地图绝对航向：`rotate_to()`

```python
robot.rotate_to(90)
```

目标是地图坐标系的绝对 yaw=90°。

这个函数和：

```python
robot.rotate_by(90)
```

不是一个意思。

`rotate_by()` 是相对当前姿态转多少；`rotate_to()` 是转到地图绝对方向。

---

# 11. 实时速度控制：`set_velocity()`

如果视觉/规划算法最终直接输出：

```text
v：线速度
ω：角速度
```

建议使用：

```python
robot.set_velocity(
    linear_mps=0.10,
    angular_rps=0.20,
)
```

其中：

```text
linear_mps > 0 ：前进
linear_mps < 0 ：后退
angular_rps > 0：左转
angular_rps < 0：右转
```

### 这一接口不是“调用一次一直跑”

它是实时控制帧。

规划器应该持续刷新，推荐大约 10 Hz：

```python
import time

with WaterChassis() as robot:
    while True:
        v, w = planner()
        robot.set_velocity(v, w)
        time.sleep(0.1)
```

如果更新中断，Gateway 的 watchdog 会自动发零速度停车。

默认配置：

```json
"command_rate_hz": 10.0,
"watchdog_s": 0.35
```

所以不要用 1 Hz、2 Hz 这种很慢的频率去持续控制。

---

# 12. 固定速度运行一段时间：`drive_for()`

适合测试，不是最推荐的路径规划接口。

```python
robot.drive_for(
    linear_mps=0.08,
    angular_rps=0.15,
    duration_s=3.0,
)
```

表示按固定 v、ω 跑 3 秒，然后自动停车。

原地旋转：

```python
robot.drive_for(
    linear_mps=0.0,
    angular_rps=0.15,
    duration_s=2.0,
)
```

也可以写：

```python
robot.spin_for(
    angular_rps=0.15,
    duration_s=2.0,
)
```

---

# 13. 动作过程反馈

V5.1 默认在阻塞式动作执行过程中输出实时反馈。

例如：

```text
[WATER][距离] | 行驶中 |  65.0% | 已走=0.195m | 剩余=0.105m | 位姿=(...) | 线速度=...
```

### 关闭控制台反馈

临时关闭当前对象：

```python
with WaterChassis(feedback=False) as robot:
    robot.forward(0.50)
```

运行中也可以修改：

```python
robot.set_feedback(False)
```

或者在 `config.json`：

```json
"feedback": {
    "enabled": false
}
```

### 单次动作关闭反馈

```python
robot.forward(
    0.50,
    feedback=False,
)
```

---

# 14. 反馈回调：给 GUI / ROS / 日志使用

不要让其他程序去解析控制台字符串。

应该直接使用回调：

```python
def on_feedback(event):
    print(event["action"])
    print(event["phase"])
    print(event["progress"])
    print(event["pose"])
    print(event["velocity"])


with WaterChassis(
    feedback=False,
    feedback_callback=on_feedback,
) as robot:
    robot.forward(0.50)
```

`event` 常见字段：

```python
{
    "type": "progress",
    "action": "distance",
    "phase": "DRIVING",
    "elapsed_s": 1.2,
    "progress": 0.63,
    "pose": {...},
    "velocity": {...},
    "power": {...},
    "safety": {...},
    "direct_action": {...},
    "navigation_remaining_m": None,
    "state": {...},
}
```

这里的字段名保持英文，是为了让程序稳定调用，不随界面语言变化。

---

# 15. WATER 厂家自主导航

DIRECT 和自主导航是两套不同用途的控制。

### DIRECT

```python
robot.move_relative(30, 0.5)
```

SDK 自己利用位姿反馈闭环控制。

### WATER 自主导航

```python
robot.navigate_relative(30, 1.0)
```

SDK 把目标转换给 WATER，由 WATER 自己进行厂家地图规划/避障。

---

## 15.1 相对目标自主导航：`navigate_relative()`

```python
robot.navigate_relative(
    heading_deg=0,
    distance_m=1.0,
    wait=True,
)
```

`wait=True` 表示这个 Python 函数等任务结束再返回。

`wait=False` 表示立即返回任务信息：

```python
task = robot.navigate_relative(
    heading_deg=0,
    distance_m=2.0,
    wait=False,
)

print(task["task_id"])
```

然后：

```python
result = robot.wait_task(task["task_id"])
```

---

## 15.2 地图绝对目标：`navigate_to()`

```python
robot.navigate_to(
    x_m=1.0,
    y_m=2.0,
    yaw_deg=90,
    wait=True,
)
```

`yaw_deg` 是地图绝对航向角。

---

## 15.3 查询任务：`get_task()`

```python
task = robot.get_task(task_id)
```

---

## 15.4 等待任务：`wait_task()`

```python
result = robot.wait_task(task_id)
```

---

## 15.5 取消厂家导航：`cancel_navigation()`

```python
robot.cancel_navigation()
```

---

# 16. 正常停车、软件急停和实体急停

## 正常停车

```python
robot.stop()
```

这是日常程序最常用的停止方式。

## 软件急停

```python
robot.estop()
```

## 解除软件急停

```python
robot.release_estop()
```

**注意：软件接口不能替代实体急停按钮。**

实体急停仍然需要在机器人上物理解除。

危险情况下优先使用实体急停。

---

# 17. 机器人信息和诊断

## 机器人信息

```python
info = robot.get_robot_info()
```

## 自诊断

```python
diagnosis = robot.get_diagnosis()
```

---

# 18. 导航参数

## 获取厂家导航参数

```python
params = robot.get_navigation_params()
```

## 设置厂家导航速度

```python
robot.set_navigation_speed(
    linear_mps=0.4,
    angular_rps=0.8,
)
```

这影响的是厂家自主导航，不等于 DIRECT 的实时速度上限。

---

# 19. 地图与规划辅助接口

## 获取当前规划路径

```python
path = robot.get_planned_path()
```

## 查询附近可达点

```python
result = robot.query_accessible_point(
    x_m=1.0,
    y_m=2.0,
)
```

## 查询某点到障碍物距离

```python
result = robot.query_obstacle_distance(
    x_m=1.0,
    y_m=2.0,
)
```

## 查询两点规划路径距离

```python
distance = robot.plan_distance(
    start_x=0.0,
    start_y=0.0,
    start_floor=4,
    goal_x=3.0,
    goal_y=4.0,
    goal_floor=4,
)
```

## 地图列表

```python
maps = robot.get_map_list()
```

## 当前地图

```python
current_map = robot.get_current_map()
```

---

# 20. 配置文件 `config.json`

绝大多数需要调的参数都已经集中到这里，不建议普通调用者去改 `app/gateway.py`。

完整配置示例：

```json
{
  "robot": {
    "host": "192.168.10.10",
    "port": 31001,
    "connect_timeout_s": 3.0,
    "request_timeout_s": 3.0,
    "reconnect_delay_s": 1.0
  },
  "gateway": {
    "host": "127.0.0.1",
    "port": 8080,
    "auto_start": true,
    "startup_timeout_s": 8.0,
    "chassis_wait_timeout_s": 12.0
  },
  "state": {
    "status_frequency_hz": 5.0,
    "velocity_frequency_hz": 10.0,
    "power_poll_period_s": 5.0,
    "pose_stale_after_s": 1.0,
    "stream_frequency_hz": 5.0
  },
  "feedback": {
    "enabled": true,
    "interval_s": 0.25,
    "print_to_console": true,
    "show_pose": true,
    "show_velocity": true,
    "show_power": false,
    "show_progress": true
  },
  "direct": {
    "max_linear_mps": 0.35,
    "max_angular_rps": 0.7,
    "watchdog_s": 0.35,
    "command_rate_hz": 10.0,
    "rotation": {
      "max_angular_rps": 0.3,
      "tolerance_deg": 2.0,
      "kp": 1.8,
      "min_angular_rps": 0.08,
      "timeout_factor": 2.5,
      "timeout_extra_s": 2.0
    },
    "distance": {
      "speed_mps": 0.1,
      "tolerance_m": 0.015,
      "heading_hold": true,
      "speed_kp": 0.85,
      "min_speed_mps": 0.035,
      "heading_kp": 1.6,
      "max_heading_correction_rps": 0.2,
      "timeout_factor": 2.8,
      "timeout_extra_s": 3.0
    },
    "relative": {
      "linear_speed_mps": 0.1,
      "angular_speed_rps": 0.3,
      "distance_tolerance_m": 0.015,
      "angle_tolerance_deg": 2.0
    }
  },
  "navigation": {
    "distance_tolerance_m": 0.05,
    "yaw_tolerance_deg": 6.0,
    "task_timeout_s": 60.0,
    "max_continuous_retries": 30
  }
}
```

各参数解释见 `配置参数说明.md`。

---

# 21. 调用参数优先级

例如 config 中：

```json
"speed_mps": 0.10
```

如果直接：

```python
robot.forward(0.5)
```

就使用 0.10 m/s 左右的默认参数。

如果某次临时想慢一点：

```python
robot.forward(
    0.5,
    speed_mps=0.06,
)
```

这次调用使用 `0.06`，不会修改配置文件。

可以理解成：

```text
单次函数参数 > config.json 默认参数
```

---

# 22. `get_config()` 和 `set_feedback()`

获取当前配置副本：

```python
config = robot.get_config()
```

临时关闭反馈：

```python
robot.set_feedback(False)
```

重新开启：

```python
robot.set_feedback(True)
```

---

# 23. 可选中文 API

Python 3 本身允许中文标识符，所以中文版 SDK 额外提供了一些中文别名。

例如：

```python
from water_chassis_sdk import Water底盘

with Water底盘() as robot:
    print(robot.获取状态(refresh=True))
    robot.前进(0.30)
    robot.左转(30)
    robot.停止()
```

这些是合法 Python。

不过正式多人项目仍建议：

```python
WaterChassis
robot.get_state()
robot.forward()
robot.turn_left()
```

原因很简单：

- IDE 和第三方工具兼容更好
- 方便和 C++ / ROS / REST 文档对应
- 团队以后换语言不用重新翻译函数名
- 错误搜索和代码检索更方便

所以**中文别名是辅助，不是推荐主接口**。

---

# 24. 典型视觉/规划项目模板

```python
import time
from water_chassis_sdk import WaterChassis


def perception():
    # 视觉识别
    return {}


def planner(perception_result, state):
    # 示例：输出 v、w
    return 0.08, 0.10


def main():
    with WaterChassis() as robot:
        while True:
            state = robot.get_state()

            if not state.get("ready_to_move"):
                robot.stop()
                time.sleep(0.1)
                continue

            perception_result = perception()
            v, w = planner(perception_result, state)

            robot.set_velocity(v, w)
            time.sleep(0.1)


if __name__ == "__main__":
    main()
```

如果规划输出的是航向角+距离：

```python
heading_deg, distance_m = planner(...)
robot.move_relative(heading_deg, distance_m)
```

---

# 25. 常见问题

## Q1：为什么我只写一个 `main.py` 就能用？

因为 `WaterChassis()` 会自动检查本地 Gateway。如果没有启动，并且 `config.json` 中：

```json
"auto_start": true
```

SDK 会自动后台启动它。

调用者不需要再开第二个终端。

---

## Q2：运行结束时需要自己 `stop()` 吗？

动作结束时很多高层接口本身会停车，而且 `close()` 也会尝试停车。

但业务代码在逻辑上需要明确停车时，仍然建议主动：

```python
robot.stop()
```

尤其是实时 `set_velocity()` 模式。

---

## Q3：`forward()` 是按时间估算距离吗？

不是。

SDK 使用 WATER 返回的地图位姿做距离闭环，并可同时进行航向保持。

---

## Q4：`move_relative()` 会自动避障吗？

默认 `mode="direct"` 不使用厂家地图自主规划。

如果需要厂家自主地图规划/避障：

```python
robot.navigate_relative(...)
```

或：

```python
robot.move_relative(..., mode="navigation")
```

---

## Q5：规划器输出 `v / w` 应该怎么调用？

持续约 10 Hz：

```python
while True:
    robot.set_velocity(v, w)
    time.sleep(0.1)
```

不要只发一帧然后期待一直运动。

---

## Q6：程序卡死后车会一直跑吗？

DIRECT 实时速度模式有 watchdog。默认约 0.35 s 没收到新速度指令会自动发送零速度。

这是一层软件保护，但**不能替代实体急停**。

---

## Q7：能不能把反馈接到 PyQt？

可以。使用 `feedback_callback`，不要解析 `print()` 文本。

---

# 26. 推荐给其他同学的最小 API 集合

如果不想让调用者一下子看太多东西，只告诉他下面这些就够：

```python
# 状态
robot.get_state()
robot.get_pose()
robot.get_velocity()
robot.get_power()
robot.get_safety()
robot.is_ready()

# 常用运动
robot.forward(distance_m)
robot.backward(distance_m)
robot.turn_left(angle_deg)
robot.turn_right(angle_deg)
robot.move_relative(heading_deg, distance_m)

# 实时规划
robot.set_velocity(v, w)

# 停止与安全
robot.stop()
robot.estop()
```

其他 API 用到再查即可。

---

# 27. 完整公开方法索引

下面列出 `WaterChassis` 的全部公开方法，方便 IDE 搜索。

### 生命周期 / 配置

```python
robot.get_config()
robot.set_feedback(enabled, callback=None)
robot.close()
```

### 状态

```python
robot.get_state(refresh=False)
robot.get_health()
robot.get_capabilities()
robot.get_pose(refresh=False)
robot.get_velocity()
robot.get_power()
robot.get_safety()
robot.get_motion_state()
robot.get_feedback_state()
robot.is_ready()
```

### DIRECT 实时/定时控制

```python
robot.set_velocity(linear_mps, angular_rps, replace_current=False)
robot.drive_for(linear_mps, angular_rps, duration_s, ...)
robot.spin_for(angular_rps=0.25, duration_s=3.0, ...)
```

### DIRECT 闭环动作

```python
robot.rotate_by(angle_deg, ...)
robot.rotate_to(yaw_deg, ...)
robot.drive_distance(distance_m, ...)
robot.move_relative_direct(heading_deg, distance_m, ...)
robot.move_relative(heading_deg, distance_m, mode="direct", ...)
robot.forward(distance_m, ...)
robot.backward(distance_m, ...)
robot.turn_left(angle_deg=90, ...)
robot.turn_right(angle_deg=90, ...)
```

### WATER 自主导航

```python
robot.navigate_relative(heading_deg, distance_m, ...)
robot.navigate_to(x_m, y_m, yaw_deg, ...)
robot.move_map(x_m, y_m, yaw_deg, ...)
robot.get_task(task_id)
robot.wait_task(task_id, ...)
robot.cancel_navigation()
```

### 厂家信息 / 地图 / 诊断

```python
robot.get_robot_info()
robot.get_diagnosis()
robot.get_navigation_params()
robot.set_navigation_speed(linear_mps=None, angular_rps=None)
robot.get_planned_path()
robot.query_accessible_point(x_m, y_m)
robot.query_obstacle_distance(x_m, y_m)
robot.plan_distance(start_x, start_y, start_floor, goal_x, goal_y, goal_floor)
robot.get_map_list()
robot.get_current_map()
```

### 安全

```python
robot.stop()
robot.estop()
robot.release_estop()
```

---

# 28. 最后的工程建议

对于你们这种“视觉 + 规划 + 底盘”的项目，我建议实际分工保持简单：

```text
视觉：负责看到了什么
规划：负责决定怎么走
SDK：负责可靠地把控制命令落到底盘，并反馈真实底盘状态
```

规划端如果已经能持续输出 `v / w`，优先用：

```python
robot.set_velocity(v, w)
```

规划端如果只输出短时“航向角 + 距离”，用：

```python
robot.move_relative(heading_deg, distance_m)
```

需要 WATER 厂家地图规划和避障时，再明确使用：

```python
robot.navigate_relative(...)
robot.navigate_to(...)
```

这样三套语义不会混在一起。
