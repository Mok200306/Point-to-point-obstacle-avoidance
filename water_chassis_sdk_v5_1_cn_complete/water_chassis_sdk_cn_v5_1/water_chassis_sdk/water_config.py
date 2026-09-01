"""WATER SDK 与内部 Gateway 共用的配置读取模块。

这里使用 JSON 而不是 YAML，目的是让整个 SDK 目录复制到其他项目后，
不需要再额外安装 YAML 解析依赖。部署时仍可通过环境变量覆盖关键网络和安全参数。
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = ROOT / "config.json"


def deep_get(data: dict[str, Any], path: str, default: Any = None) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def load_config(path: str | os.PathLike[str] | None = None) -> dict[str, Any]:
    """读取 WATER 配置并返回一份独立字典。

    配置文件查找顺序：
      1) 显式传入的 ``path``；
      2) 环境变量 ``WATER_CONFIG``；
      3) SDK 目录中的 ``config.json``。
    """
    resolved = Path(path or os.getenv("WATER_CONFIG") or DEFAULT_CONFIG_PATH).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"找不到 WATER 配置文件：{resolved}")
    with resolved.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"WATER 配置文件最外层必须是 JSON 对象：{resolved}")
    data = copy.deepcopy(data)
    data["_config_path"] = str(resolved)
    return data


def config_path(config: dict[str, Any]) -> str:
    return str(config.get("_config_path") or DEFAULT_CONFIG_PATH)
