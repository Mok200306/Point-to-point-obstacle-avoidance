# 为什么推荐 `with WaterChassis() as robot:`

这不是 WATER SDK 的特殊语法，而是 Python 标准上下文管理器语法。

```python
with WaterChassis() as robot:
    robot.forward(0.30)
```

大致等价于：

```python
robot = WaterChassis()
try:
    robot.forward(0.30)
finally:
    robot.close()
```

`WaterChassis` 实现了：

```python
__enter__()
__exit__()
```

所以 `with` 结束时 Python 会自动进入 `__exit__()`，SDK 再调用 `close()`。

本 SDK 的 `close()` 会：

1. 尝试正常停车；
2. 关闭 HTTP 客户端；
3. 如果 Gateway 是当前对象自动启动的，就结束它；
4. 关闭日志文件。

因此推荐把它理解成：

> “我在这个代码块里占用并使用底盘，代码块结束以后自动安全收尾。”

如果一个实例只用于读取状态、不能在退出时抢占其他控制器，可以构造
`WaterChassis(stop_on_close=False)`。这只适用于诊断模式；拥有运动控制权的实例不应
关闭正常停车。

它不是唯一写法，但对于机器人底盘这种需要明确资源和运动状态收尾的对象，非常合适。
