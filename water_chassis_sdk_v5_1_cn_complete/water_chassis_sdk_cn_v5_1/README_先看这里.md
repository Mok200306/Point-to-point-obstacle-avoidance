# WATER 底盘 Python SDK V5.1 中文版

这是一套给视觉、规划、控制程序直接调用的 WATER 底盘 SDK。

## 第一次用，只看这 3 个文件

1. `main.py`：最小调用示例，先看它。
2. `docs/WATER底盘SDK使用手册.md`：完整使用手册。
3. `water_chassis_sdk/config.json`：需要改 IP、速度、容差、反馈频率时改这里。

正常使用时不需要手动启动 Gateway，也不需要自己写 Socket。SDK 会自动连接：

```text
你的 main.py
    ↓
WaterChassis
    ↓
SDK 内部 Gateway
    ↓
192.168.10.10:31001
    ↓
WATER 底盘
```

## 最推荐的写法

```python
from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        print(robot.get_state(refresh=True))
        robot.forward(0.30)
        robot.turn_left(30)
        robot.stop()


if __name__ == "__main__":
    main()
```

`with WaterChassis() as robot:` 是 Python 的标准 `with` 上下文管理器语法，不是 WATER SDK 的特殊语法。它的好处是程序正常结束或发生异常时，SDK 都会自动调用 `robot.close()`，执行正常停车并清理内部连接。

如果不想用 `with`，也可以：

```python
from water_chassis_sdk import WaterChassis

robot = WaterChassis()
try:
    robot.forward(0.30)
finally:
    robot.close()
```

## 安装依赖

在这个压缩包根目录执行：

```powershell
python -m pip install -r requirements.txt
```

## 推荐接口

```python
robot.get_state()                 # 完整状态
robot.get_pose()                  # 位姿
robot.get_velocity()              # 速度
robot.get_power()                 # 电量
robot.get_safety()                # 急停/故障

robot.forward(0.30)               # 前进 0.30 m
robot.backward(0.20)              # 后退 0.20 m
robot.turn_left(30)               # 左转 30°
robot.turn_right(30)              # 右转 30°
robot.move_relative(20, 0.50)     # 左前方 20°，再走 0.50 m

robot.set_velocity(0.10, 0.20)    # 实时 v/ω 控制，一般 10 Hz 刷新
robot.stop()                       # 正常停车
robot.estop()                      # 软件急停
```

更多内容见 `docs/WATER底盘SDK使用手册.md`。
