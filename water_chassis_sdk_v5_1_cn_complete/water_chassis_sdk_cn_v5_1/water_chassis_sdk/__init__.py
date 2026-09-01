"""WATER 底盘 Python SDK 公共入口。

推荐使用英文 API 名称，便于 IDE 补全、跨项目协作和后续 ROS/C++ 对照。
同时提供少量中文别名，方便第一次接触 SDK 的同学快速理解。
"""

from .water_chassis import FeedbackCallback, WaterChassis, Water底盘
from .water_api_client import WaterApiError, WaterChassisClient

__version__ = "5.1.0-cn"

__all__ = [
    "WaterChassis",
    "Water底盘",
    "FeedbackCallback",
    "WaterChassisClient",
    "WaterApiError",
]
