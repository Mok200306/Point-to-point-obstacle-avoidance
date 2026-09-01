# WATER 底盘 Python SDK V5.1 中文版

推荐入口：

```python
from water_chassis_sdk import WaterChassis

with WaterChassis() as robot:
    print(robot.get_state(refresh=True))
    # robot.forward(0.30)
```

- 不需要手动启动 Gateway；默认自动启动。
- 不需要自己处理 TCP / JSON / UUID / callback。
- DIRECT `v/ω`、距离闭环、转向闭环、航向角+距离、状态反馈均已封装。
- `config.json` 可修改 IP、速度、容差、控制增益和反馈显示。
- 中文控制台反馈已启用。
- 机器可读 API 名称仍以英文为主，保证 IDE 和跨语言兼容；额外提供中文别名。

完整说明请看：`WATER底盘SDK使用手册.md`。
