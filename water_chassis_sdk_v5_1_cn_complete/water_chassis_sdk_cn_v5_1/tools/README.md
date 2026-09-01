# 开发测试工具

这些文件不是普通调用者必须使用的。

- `mock_chassis_server.py`：没有真车时模拟 WATER TCP 服务，用于软件联调。
- `self_test.py`：原 V5 的基础自测脚本。

正式项目只需要 `water_chassis_sdk/`。
