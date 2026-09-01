"""WATER 底盘高层 Python SDK。

这是给视觉、规划、控制程序直接调用的入口。正常情况下，调用者只需要认识
``WaterChassis`` 这一个类；内部 FastAPI Gateway、厂家 TCP Socket、状态订阅、
重连和 watchdog 都由 SDK 自动管理。

V5.1 中文版的设计原则
----------------------
* DIRECT ``v / ω`` 是已经在真车上验证过的基础控制链。
* 距离、角度、相对航向+距离均使用 WATER 位姿反馈做闭环控制。
* 阻塞式动作默认可以实时输出执行进度，也支持回调给 GUI/ROS/日志系统。
* IP、速度、容差、控制增益、反馈显示等参数统一放在 ``config.json``。
* 仍然保留 HTTP REST 接口，非 Python 程序也可以调用。
* 机器可读字段名与方法名保留英文，这是刻意设计，便于 IDE、跨语言和团队协作；
  文档、注释、控制台反馈和常见错误提示均已汉化。
"""
from __future__ import annotations

import copy
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable, Literal

import requests

try:  # package import: from water_chassis_gateway_v5 import WaterChassis
    from .water_api_client import WaterChassisClient
    from .water_config import config_path as resolved_config_path
    from .water_config import deep_get, load_config
except ImportError:  # script/import-root mode: from water_chassis import WaterChassis
    from water_api_client import WaterChassisClient
    from water_config import config_path as resolved_config_path
    from water_config import deep_get, load_config


MotionMode = Literal["direct", "navigation"]
FeedbackCallback = Callable[[dict[str, Any]], None]

_ACTION_CN = {
    "rotate": "转向",
    "distance": "距离",
    "drive_for": "定时速度",
    "velocity": "速度控制",
    "navigation": "自主导航",
    "motion": "运动",
}
_PHASE_CN = {
    "STARTING": "准备中",
    "RUNNING": "执行中",
    "ROTATING": "转向中",
    "DRIVING": "行驶中",
    "FINISHED": "已完成",
    "ERROR": "错误",
    "IDLE": "空闲",
    "DIRECT": "直接控制",
    "NAV": "自主导航",
    "EXECUTING": "执行中",
    "ACCEPTED": "已接受",
    "SUCCEEDED": "成功",
    "FAILED": "失败",
    "CANCELED": "已取消",
    "REJECTED": "已拒绝",
}


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _version_tuple(value: str) -> tuple[int, ...]:
    out: list[int] = []
    for part in str(value).split("."):
        try:
            out.append(int(part))
        except ValueError:
            break
    return tuple(out or [0])


class WaterChassis:
    """WATER 底盘高层控制对象。

    最常用写法::

        from water_chassis_sdk import WaterChassis

        with WaterChassis() as robot:
            print(robot.get_state())
            robot.move_relative(heading_deg=30, distance_m=0.5)

    ``config.json`` 决定默认 IP、更新频率、容差、控制增益、速度限制和反馈输出。
    单次函数调用时显式传入的参数优先级更高，会覆盖 config 中的默认值。

    ``with`` 不是本 SDK 自创格式，而是 Python 的“上下文管理器”语法。
    离开 ``with`` 时会自动调用 ``close()``，先执行正常停车，再清理由当前
    ``WaterChassis`` 对象自动启动的 Gateway。
    """

    GATEWAY_MIN_VERSION = "1.3.0"

    def __init__(
        self,
        robot_host: str | None = None,
        robot_port: int | None = None,
        *,
        config_path: str | os.PathLike[str] | None = None,
        gateway_host: str | None = None,
        gateway_port: int | None = None,
        connect_timeout_s: float | None = None,
        auto_start_gateway: bool | None = None,
        stop_on_close: bool = True,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ):
        self.config = load_config(config_path)
        self.config_path = resolved_config_path(self.config)

        self.robot_host = str(robot_host or deep_get(self.config, "robot.host", "192.168.10.10"))
        self.robot_port = int(robot_port or deep_get(self.config, "robot.port", 31001))

        configured_gateway_host = str(deep_get(self.config, "gateway.host", "127.0.0.1"))
        self.gateway_host = str(gateway_host or configured_gateway_host)
        # 0.0.0.0 is valid for binding but not useful as the local request destination.
        if self.gateway_host == "0.0.0.0":
            self.gateway_host = "127.0.0.1"
        self.gateway_port = int(gateway_port or deep_get(self.config, "gateway.port", 8080))
        self.base_url = f"http://{self.gateway_host}:{self.gateway_port}"

        self.connect_timeout_s = float(
            connect_timeout_s
            if connect_timeout_s is not None
            else deep_get(self.config, "gateway.chassis_wait_timeout_s", 12.0)
        )
        self.auto_start_gateway = bool(
            deep_get(self.config, "gateway.auto_start", True)
            if auto_start_gateway is None
            else auto_start_gateway
        )
        self.stop_on_close = bool(stop_on_close)

        self.feedback_enabled = bool(
            deep_get(self.config, "feedback.enabled", True)
            if feedback is None
            else feedback
        )
        self.feedback_callback = feedback_callback

        self._root = Path(__file__).resolve().parent
        self._gateway_proc: subprocess.Popen | None = None
        self._gateway_log = None
        self.client = WaterChassisClient(self.base_url)

        try:
            self._ensure_gateway()
            self._wait_chassis()
        except Exception:
            # Do not leave an auto-started Gateway behind when the chassis is
            # offline or the first connection attempt fails.
            try:
                self.close()
            except Exception:
                pass
            raise

    # ==================================================================
    # Configuration / lifecycle
    # ==================================================================
    def get_config(self) -> dict[str, Any]:
        """返回当前已加载配置的一份副本。修改返回值不会直接修改 SDK 内部配置。"""
        return copy.deepcopy(self.config)

    def set_feedback(
        self,
        enabled: bool,
        callback: FeedbackCallback | None = None,
    ) -> None:
        """打开或关闭后续阻塞式动作的默认实时反馈，也可同时设置反馈回调函数。"""
        self.feedback_enabled = bool(enabled)
        if callback is not None:
            self.feedback_callback = callback

    def _http_alive(self) -> bool:
        try:
            r = requests.get(self.base_url + "/api/v1/health", timeout=0.35)
            return r.ok
        except Exception:
            return False

    def _ensure_gateway(self) -> None:
        if self._http_alive():
            state = self.client.state()
            existing_host = state.get("connection", {}).get("robot_host")
            existing_port = state.get("connection", {}).get("robot_port")
            health = self.client.health()
            existing_version = str(health.get("version") or "0")
            if existing_host != self.robot_host or int(existing_port or 0) != self.robot_port:
                raise RuntimeError(
                    f"Port {self.gateway_port} already has a WATER gateway for "
                    f"{existing_host}:{existing_port}; requested {self.robot_host}:{self.robot_port}. "
                    "请先关闭旧 Gateway。"
                )
            if _version_tuple(existing_version) < _version_tuple(self.GATEWAY_MIN_VERSION):
                raise RuntimeError(
                    f"An older WATER gateway ({existing_version}) is running on port "
                    f"{self.gateway_port}; V5 requires >= {self.GATEWAY_MIN_VERSION}. "
                    "请先关闭旧版本进程。"
                )
            return

        if not self.auto_start_gateway:
            raise RuntimeError(f"WATER Gateway 未运行： {self.base_url}")

        env = os.environ.copy()
        env["WATER_CONFIG"] = self.config_path
        env["ROBOT_HOST"] = self.robot_host
        env["ROBOT_PORT"] = str(self.robot_port)
        env["GATEWAY_HOST"] = "127.0.0.1"
        env["GATEWAY_PORT"] = str(self.gateway_port)

        configured_log_path = os.getenv("WATER_GATEWAY_LOG", "").strip()
        if not configured_log_path:
            configured_log_path = str(
                deep_get(self.config, "gateway.log_path", "") or "")
        log_path = (
            Path(configured_log_path).expanduser()
            if configured_log_path else self._root / "water_gateway.log"
        )
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._gateway_log = open(log_path, "w", encoding="utf-8")
        except OSError:
            # Installed ROS package shares are commonly read-only for the
            # runtime UID. Keep the SDK usable there without making the
            # Gateway log a startup blocker.
            fallback_root = Path(
                os.getenv("ROS_HOME", "") or tempfile.gettempdir())
            fallback_path = fallback_root / f"water_gateway_{self.gateway_port}.log"
            fallback_path.parent.mkdir(parents=True, exist_ok=True)
            log_path = fallback_path
            self._gateway_log = open(log_path, "w", encoding="utf-8")
        self._gateway_proc = subprocess.Popen(
            [sys.executable, str(self._root / "run_gateway.py")],
            cwd=str(self._root),
            env=env,
            stdout=self._gateway_log,
            stderr=subprocess.STDOUT,
        )

        startup_timeout = float(deep_get(self.config, "gateway.startup_timeout_s", 8.0))
        deadline = time.monotonic() + startup_timeout
        while time.monotonic() < deadline:
            if self._gateway_proc.poll() is not None:
                raise RuntimeError(f"WATER Gateway 启动过程中异常退出，请查看 {log_path}")
            if self._http_alive():
                return
            time.sleep(0.15)
        raise RuntimeError(f"WATER Gateway 启动失败，请查看 {log_path}")

    def _wait_chassis(self) -> None:
        deadline = time.monotonic() + self.connect_timeout_s
        last_health: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            try:
                last_health = self.client.health()
                if last_health.get("chassis_connected"):
                    return
            except Exception:
                pass
            time.sleep(0.4)
        raise RuntimeError(
            f"无法连接 WATER 底盘 {self.robot_host}:{self.robot_port}. "
            f"Last health={last_health}. 请检查底盘 WiFi/LAN，并查看 water_gateway.log。"
        )

    def close(self) -> None:
        """先执行正常停车，再关闭客户端；仅终止由当前对象自动启动的 Gateway。"""
        if self.stop_on_close:
            try:
                self.stop()
            except Exception:
                pass

        self.client.close()
        if self._gateway_proc is not None:
            try:
                self._gateway_proc.terminate()
                self._gateway_proc.wait(timeout=2.0)
            except Exception:
                try:
                    self._gateway_proc.kill()
                except Exception:
                    pass
            self._gateway_proc = None

        if self._gateway_log is not None:
            try:
                self._gateway_log.close()
            except Exception:
                pass
            self._gateway_log = None

    def __enter__(self) -> "WaterChassis":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ==================================================================
    # State / diagnostics
    # ==================================================================
    def get_state(self, *, refresh: bool = False) -> dict[str, Any]:
        """获取完整底盘状态。``refresh=True`` 时先主动刷新一次底盘状态。"""
        return self.client.state(refresh=refresh)

    def get_health(self) -> dict[str, Any]:
        """获取 SDK Gateway 与真实底盘的连接健康状态。"""
        return self.client.health()

    def get_capabilities(self) -> dict[str, Any]:
        """获取当前 SDK 暴露的控制模式、速度限制和状态能力。"""
        return self.client.capabilities()

    def get_pose(self, *, refresh: bool = False) -> dict[str, Any]:
        """获取地图坐标系位姿：x_m、y_m、yaw_rad、floor。"""
        return self.get_state(refresh=refresh)["pose"]

    def get_velocity(self) -> dict[str, Any]:
        """获取当前线速度和角速度。"""
        return self.get_state()["velocity"]

    def get_power(self) -> dict[str, Any]:
        """获取电量、电压、电流和充电状态。"""
        return self.get_state()["power"]

    def get_safety(self) -> dict[str, Any]:
        """获取急停、故障和 error_code 等安全状态。"""
        return self.get_state()["safety"]

    def get_motion_state(self) -> dict[str, Any]:
        """获取当前运动模式、厂家导航状态和 DIRECT 动作进度。"""
        return self.get_state()["motion"]

    def get_feedback_state(self) -> dict[str, Any] | None:
        """返回当前或最近一次 DIRECT 动作的进度记录。"""
        return self.get_motion_state().get("direct_action")

    def is_ready(self) -> bool:
        """底盘在线、位姿有效且未急停/故障时返回 True。"""
        return bool(self.get_state().get("ready_to_move"))

    # ==================================================================
    # Live feedback helper
    # ==================================================================
    def _feedback_should_print(self, enabled: bool) -> bool:
        return enabled and bool(deep_get(self.config, "feedback.print_to_console", True))

    def _emit_feedback(self, event: dict[str, Any], *, enabled: bool, callback: FeedbackCallback | None) -> None:
        cb = callback or self.feedback_callback
        if cb is not None:
            try:
                cb(copy.deepcopy(event))
            except Exception as exc:
                # A UI/log callback must never crash vehicle control.
                if self._feedback_should_print(enabled):
                    print(f"[WATER][反馈回调异常] {exc}")

        if not self._feedback_should_print(enabled):
            return

        action_raw = str(event.get("action") or "motion")
        phase_raw = str(event.get("phase") or "")
        action = _ACTION_CN.get(action_raw, action_raw.upper())
        phase = _PHASE_CN.get(phase_raw, phase_raw)
        progress = event.get("progress")
        progress_text = ""
        if deep_get(self.config, "feedback.show_progress", True) and isinstance(progress, (int, float)):
            progress_text = f" {max(0.0, min(1.0, float(progress))) * 100:5.1f}%"

        parts = [f"[WATER][{action}]", phase, progress_text]
        da = event.get("direct_action") or {}
        if da.get("action") == "distance":
            parts.append(f"已走={float(da.get('travelled_m') or 0.0):.3f}m")
            if da.get("remaining_m") is not None:
                parts.append(f"剩余={float(da['remaining_m']):.3f}m")
        elif da.get("action") == "rotate":
            if da.get("error_rad") is not None:
                parts.append(f"角度误差={math.degrees(float(da['error_rad'])):.1f}deg")
        elif event.get("navigation_remaining_m") is not None:
            parts.append(f"剩余={float(event['navigation_remaining_m']):.3f}m")

        if deep_get(self.config, "feedback.show_pose", True):
            pose = event.get("pose") or {}
            if pose.get("x_m") is not None and pose.get("y_m") is not None:
                yaw_deg = None if pose.get("yaw_rad") is None else math.degrees(float(pose["yaw_rad"]))
                parts.append(
                    f"位姿=({float(pose['x_m']):.3f},{float(pose['y_m']):.3f},"
                    f"{yaw_deg:.1f}deg)" if yaw_deg is not None else
                    f"位姿=({float(pose['x_m']):.3f},{float(pose['y_m']):.3f})"
                )

        if deep_get(self.config, "feedback.show_velocity", True):
            vel = event.get("velocity") or {}
            if vel:
                parts.append(
                    f"线速度={float(vel.get('linear_mps') or 0.0):.3f}m/s "
                    f"角速度={float(vel.get('angular_rps') or 0.0):.3f}rad/s"
                )

        if deep_get(self.config, "feedback.show_power", False):
            p = event.get("power") or {}
            if p.get("percent") is not None:
                parts.append(f"电量={p['percent']}%")

        print(" | ".join(p for p in parts if p))

    def _run_with_feedback(
        self,
        action: str,
        fn: Callable[[], dict[str, Any]],
        *,
        feedback: bool | None = None,
        callback: FeedbackCallback | None = None,
        duration_s: float | None = None,
    ) -> dict[str, Any]:
        enabled = self.feedback_enabled if feedback is None else bool(feedback)
        cb = callback or self.feedback_callback
        if not enabled and cb is None:
            return fn()

        interval = max(0.05, float(deep_get(self.config, "feedback.interval_s", 0.25)))
        stop_event = threading.Event()
        started = time.monotonic()

        def worker() -> None:
            session = requests.Session()
            try:
                while not stop_event.is_set():
                    try:
                        r = session.get(self.base_url + "/api/v1/chassis/state", timeout=0.7)
                        if r.ok:
                            state = r.json()
                            motion = state.get("motion", {})
                            da = motion.get("direct_action")
                            phase = None
                            progress = None
                            expected_direct_action = {
                                "rotate": "rotate",
                                "distance": "distance",
                                "drive_for": "velocity",
                            }.get(action)
                            if isinstance(da, dict) and (
                                expected_direct_action is None
                                or da.get("action") == expected_direct_action
                            ):
                                phase = da.get("phase")
                                progress = da.get("progress")
                            else:
                                # Avoid showing the previous primitive's terminal
                                # snapshot during the first few milliseconds of a new
                                # command, before the Gateway has published its new
                                # direct_action record.
                                da = None
                                phase = "STARTING"
                                progress = 0.0
                            if duration_s is not None:
                                elapsed = time.monotonic() - started
                                progress = min(1.0, max(0.0, elapsed / max(duration_s, 1e-9)))
                                phase = phase or "RUNNING"
                            event = {
                                "type": "progress",
                                "action": action,
                                "phase": phase or motion.get("vendor_running_status") or motion.get("gateway_mode"),
                                "elapsed_s": time.monotonic() - started,
                                "progress": progress,
                                "pose": state.get("pose"),
                                "velocity": state.get("velocity"),
                                "power": state.get("power"),
                                "safety": state.get("safety"),
                                "direct_action": da,
                                "navigation_remaining_m": motion.get("euclidean_distance_to_goal_m"),
                                "state": state,
                            }
                            self._emit_feedback(event, enabled=enabled, callback=callback)
                    except Exception:
                        pass
                    stop_event.wait(interval)
            finally:
                session.close()

        thread = threading.Thread(target=worker, name=f"water-feedback-{action}", daemon=True)
        thread.start()
        result: dict[str, Any] | None = None
        error: Exception | None = None
        try:
            result = fn()
            return result
        except Exception as exc:
            error = exc
            raise
        finally:
            stop_event.set()
            thread.join(timeout=max(1.0, interval * 3))
            try:
                state = self.get_state()
            except Exception:
                state = {}
            final_event = {
                "type": "finished" if error is None else "error",
                "action": action,
                "phase": "FINISHED" if error is None else "ERROR",
                "elapsed_s": time.monotonic() - started,
                "progress": 1.0 if error is None else None,
                "pose": state.get("pose"),
                "velocity": state.get("velocity"),
                "power": state.get("power"),
                "safety": state.get("safety"),
                "direct_action": state.get("motion", {}).get("direct_action") if state else None,
                "navigation_remaining_m": state.get("motion", {}).get("euclidean_distance_to_goal_m") if state else None,
                "state": state,
                "result": result,
                "error": None if error is None else repr(error),
            }
            self._emit_feedback(final_event, enabled=enabled, callback=callback)

    # ==================================================================
    # Low-level DIRECT control
    # ==================================================================
    def set_velocity(
        self,
        linear_mps: float,
        angular_rps: float,
        *,
        replace_current: bool = False,
    ) -> dict[str, Any]:
        """发送一帧线速度/角速度命令。

        实时规划器应按 ``direct.command_rate_hz`` 左右持续刷新（默认 10 Hz）。
        如果后续指令中断，Gateway watchdog 会自动发送零速度停车。
        """
        return self.client.set_velocity(
            linear_mps,
            angular_rps,
            replace_current=replace_current,
        )

    def drive_for(
        self,
        linear_mps: float,
        angular_rps: float,
        duration_s: float,
        *,
        rate_hz: float | None = None,
        replace_current: bool = False,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        """按固定 v/ω 持续运动指定秒数，结束后自动停车。适合简单测试动作。"""
        rate = float(rate_hz or deep_get(self.config, "direct.command_rate_hz", 10.0))
        return self._run_with_feedback(
            "drive_for",
            lambda: self.client.drive_for(
                linear_mps,
                angular_rps,
                duration_s,
                rate_hz=rate,
                replace_current=replace_current,
            ),
            feedback=feedback,
            callback=feedback_callback,
            duration_s=float(duration_s),
        )

    def spin_for(
        self,
        angular_rps: float = 0.25,
        duration_s: float = 3.0,
        *,
        rate_hz: float | None = None,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        """原地按固定角速度旋转指定秒数；正值左转，负值右转。"""
        return self.drive_for(
            0.0, angular_rps, duration_s,
            rate_hz=rate_hz,
            feedback=feedback,
            feedback_callback=feedback_callback,
        )

    # ==================================================================
    # Closed-loop DIRECT motion primitives
    # ==================================================================
    def rotate_by(
        self,
        angle_deg: float,
        *,
        max_angular_rps: float | None = None,
        tolerance_deg: float | None = None,
        timeout_s: float | None = None,
        rate_hz: float | None = None,
        replace_current: bool = True,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        """基于位姿反馈相对旋转指定角度（度）；正值左转，负值右转。"""
        max_w = float(max_angular_rps or deep_get(self.config, "direct.rotation.max_angular_rps", 0.30))
        tol = float(tolerance_deg or deep_get(self.config, "direct.rotation.tolerance_deg", 2.0))
        rate = float(rate_hz or deep_get(self.config, "direct.command_rate_hz", 10.0))
        return self._run_with_feedback(
            "rotate",
            lambda: self.client.rotate_by_direct(
                angle_deg,
                max_angular_rps=max_w,
                tolerance_deg=tol,
                timeout_s=timeout_s,
                rate_hz=rate,
                replace_current=replace_current,
            ),
            feedback=feedback,
            callback=feedback_callback,
        )

    def rotate_to(
        self,
        yaw_deg: float,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """旋转到地图坐标系的绝对 yaw 角（度）。"""
        pose = self.get_pose(refresh=True)
        current = float(pose["yaw_rad"])
        target = math.radians(float(yaw_deg))
        delta = _normalize_angle(target - current)
        return self.rotate_by(math.degrees(delta), **kwargs)

    def drive_distance(
        self,
        distance_m: float,
        *,
        speed_mps: float | None = None,
        tolerance_m: float | None = None,
        heading_hold: bool | None = None,
        max_heading_correction_rps: float | None = None,
        timeout_s: float | None = None,
        rate_hz: float | None = None,
        replace_current: bool = True,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        """基于位姿反馈闭环行驶指定距离；正值前进，负值后退。"""
        speed = float(speed_mps or deep_get(self.config, "direct.distance.speed_mps", 0.10))
        tol = float(tolerance_m or deep_get(self.config, "direct.distance.tolerance_m", 0.015))
        hold = bool(deep_get(self.config, "direct.distance.heading_hold", True) if heading_hold is None else heading_hold)
        max_corr = float(
            max_heading_correction_rps
            if max_heading_correction_rps is not None
            else deep_get(self.config, "direct.distance.max_heading_correction_rps", 0.20)
        )
        rate = float(rate_hz or deep_get(self.config, "direct.command_rate_hz", 10.0))
        return self._run_with_feedback(
            "distance",
            lambda: self.client.drive_distance_direct(
                distance_m,
                speed_mps=speed,
                tolerance_m=tol,
                heading_hold=hold,
                max_heading_correction_rps=max_corr,
                timeout_s=timeout_s,
                rate_hz=rate,
                replace_current=replace_current,
            ),
            feedback=feedback,
            callback=feedback_callback,
        )

    def move_relative_direct(
        self,
        heading_deg: float,
        distance_m: float,
        *,
        linear_speed_mps: float | None = None,
        angular_speed_rps: float | None = None,
        distance_tolerance_m: float | None = None,
        angle_tolerance_deg: float | None = None,
        replace_current: bool = True,
        timeout_s: float | None = None,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        """用两个 DIRECT 闭环完成“相对航向角 + 有符号距离”。

        执行顺序明确分成“转向”和“行驶距离”两段，因此反馈更直观，
        发生异常时也比把整个动作交给厂家自主导航更容易定位问题。
        """
        linear = float(linear_speed_mps or deep_get(self.config, "direct.relative.linear_speed_mps", 0.10))
        angular = float(angular_speed_rps or deep_get(self.config, "direct.relative.angular_speed_rps", 0.30))
        dist_tol = float(distance_tolerance_m or deep_get(self.config, "direct.relative.distance_tolerance_m", 0.015))
        angle_tol = float(angle_tolerance_deg or deep_get(self.config, "direct.relative.angle_tolerance_deg", 2.0))

        start_state = self.get_state(refresh=True)
        source_pose = copy.deepcopy(start_state.get("pose"))

        rotate_timeout = None
        drive_timeout = None
        if timeout_s is not None:
            tr = abs(math.radians(float(heading_deg))) / max(angular, 0.05)
            td = abs(float(distance_m)) / max(linear, 0.03)
            total = max(tr + td, 1e-6)
            rotate_timeout = max(1.0, float(timeout_s) * tr / total) if tr > 0 else 1.0
            drive_timeout = max(1.0, float(timeout_s) - rotate_timeout)

        rotation = self.rotate_by(
            heading_deg,
            max_angular_rps=angular,
            tolerance_deg=angle_tol,
            timeout_s=rotate_timeout,
            replace_current=replace_current,
            feedback=feedback,
            feedback_callback=feedback_callback,
        )
        translation = self.drive_distance(
            distance_m,
            speed_mps=linear,
            tolerance_m=dist_tol,
            timeout_s=drive_timeout,
            replace_current=True,
            feedback=feedback,
            feedback_callback=feedback_callback,
        )
        return {
            "ok": True,
            "mode": "direct",
            "heading_deg": float(heading_deg),
            "distance_m": float(distance_m),
            "source_pose": source_pose,
            "rotation": rotation,
            "translation": translation,
        }

    def move_relative(
        self,
        heading_deg: float,
        distance_m: float,
        *,
        mode: MotionMode = "direct",
        **kwargs: Any,
    ) -> dict[str, Any]:
        """执行“相对航向角 + 距离”。

        ``heading_deg``：0=当前正前方，正值=向左，负值=向右。
        默认 ``mode="direct"``，使用已验证的 DIRECT 闭环；设为
        ``mode="navigation"`` 时交给 WATER 厂家自主导航。
        """
        if mode == "direct":
            return self.move_relative_direct(heading_deg, distance_m, **kwargs)
        if mode == "navigation":
            return self.navigate_relative(heading_deg, distance_m, **kwargs)
        raise ValueError("mode 只能是 'direct' 或 'navigation'")

    def forward(self, distance_m: float, **kwargs: Any) -> dict[str, Any]:
        """前进指定距离（米）。"""
        return self.drive_distance(abs(float(distance_m)), **kwargs)

    def backward(self, distance_m: float, **kwargs: Any) -> dict[str, Any]:
        """后退指定距离（米）。"""
        return self.drive_distance(-abs(float(distance_m)), **kwargs)

    def turn_left(self, angle_deg: float = 90.0, **kwargs: Any) -> dict[str, Any]:
        """原地向左闭环旋转指定角度（度）。"""
        return self.rotate_by(abs(float(angle_deg)), **kwargs)

    def turn_right(self, angle_deg: float = 90.0, **kwargs: Any) -> dict[str, Any]:
        """原地向右闭环旋转指定角度（度）。"""
        return self.rotate_by(-abs(float(angle_deg)), **kwargs)

    # ==================================================================
    # WATER autonomous navigation (vendor map planning + obstacle avoidance)
    # ==================================================================
    def navigate_relative(
        self,
        heading_deg: float,
        distance_m: float,
        *,
        wait: bool = True,
        timeout_s: float | None = None,
        distance_tolerance_m: float | None = None,
        yaw_tolerance_deg: float | None = None,
        max_continuous_retries: int | None = None,
        replace_current: bool = False,
        final_yaw_deg: float | None = None,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        timeout = float(timeout_s or deep_get(self.config, "navigation.task_timeout_s", 60.0))
        dist_tol = float(distance_tolerance_m or deep_get(self.config, "navigation.distance_tolerance_m", 0.05))
        yaw_tol = float(yaw_tolerance_deg or deep_get(self.config, "navigation.yaw_tolerance_deg", 6.0))
        retries = int(
            deep_get(self.config, "navigation.max_continuous_retries", 30)
            if max_continuous_retries is None
            else max_continuous_retries
        )
        task = self.client.navigate_relative(
            heading_deg,
            distance_m,
            wait=False,
            timeout_s=timeout,
            distance_tolerance_m=dist_tol,
            yaw_tolerance_deg=yaw_tol,
            max_continuous_retries=retries,
            replace_current=replace_current,
            final_yaw_deg=final_yaw_deg,
        )
        if not wait:
            return task
        return self._run_with_feedback(
            "navigation",
            lambda: self.client.wait_task(task["task_id"], timeout_s=timeout + 5.0),
            feedback=feedback,
            callback=feedback_callback,
        )

    def navigate_to(
        self,
        x_m: float,
        y_m: float,
        yaw_deg: float,
        *,
        wait: bool = True,
        timeout_s: float | None = None,
        distance_tolerance_m: float | None = None,
        yaw_tolerance_deg: float | None = None,
        max_continuous_retries: int | None = None,
        replace_current: bool = False,
        feedback: bool | None = None,
        feedback_callback: FeedbackCallback | None = None,
    ) -> dict[str, Any]:
        timeout = float(timeout_s or deep_get(self.config, "navigation.task_timeout_s", 60.0))
        dist_tol = float(distance_tolerance_m or deep_get(self.config, "navigation.distance_tolerance_m", 0.05))
        yaw_tol = float(yaw_tolerance_deg or deep_get(self.config, "navigation.yaw_tolerance_deg", 6.0))
        retries = int(
            deep_get(self.config, "navigation.max_continuous_retries", 30)
            if max_continuous_retries is None
            else max_continuous_retries
        )
        task = self.client.navigate_to(
            x_m, y_m, yaw_deg,
            wait=False,
            timeout_s=timeout,
            distance_tolerance_m=dist_tol,
            yaw_tolerance_deg=yaw_tol,
            max_continuous_retries=retries,
            replace_current=replace_current,
        )
        if not wait:
            return task
        return self._run_with_feedback(
            "navigation",
            lambda: self.client.wait_task(task["task_id"], timeout_s=timeout + 5.0),
            feedback=feedback,
            callback=feedback_callback,
        )

    move_map = navigate_to

    def get_task(self, task_id: str) -> dict[str, Any]:
        return self.client.task(task_id)

    def wait_task(self, task_id: str, **kwargs: Any) -> dict[str, Any]:
        return self.client.wait_task(task_id, **kwargs)

    def cancel_navigation(self) -> dict[str, Any]:
        return self.client.cancel_navigation()

    # ==================================================================
    # Vendor helpers
    # ==================================================================
    def get_robot_info(self) -> dict[str, Any]:
        return self.client.robot_info()

    def get_diagnosis(self) -> dict[str, Any]:
        return self.client.diagnosis()

    def get_navigation_params(self) -> dict[str, Any]:
        return self.client.get_navigation_params()

    def set_navigation_speed(
        self,
        *,
        linear_mps: float | None = None,
        angular_rps: float | None = None,
    ) -> dict[str, Any]:
        return self.client.set_navigation_params(
            max_speed_linear=linear_mps,
            max_speed_angular=angular_rps,
        )

    def get_planned_path(self) -> dict[str, Any]:
        return self.client.planned_path()

    def query_accessible_point(self, x_m: float, y_m: float) -> dict[str, Any]:
        return self.client.accessible_point(x_m, y_m)

    def query_obstacle_distance(self, x_m: float, y_m: float) -> dict[str, Any]:
        return self.client.distance_probe(x_m, y_m)

    def plan_distance(
        self,
        start_x: float,
        start_y: float,
        start_floor: int,
        goal_x: float,
        goal_y: float,
        goal_floor: int,
    ) -> float:
        return self.client.plan_distance(start_x, start_y, start_floor, goal_x, goal_y, goal_floor)

    def get_map_list(self) -> dict[str, Any]:
        return self.client.map_list()

    def get_current_map(self) -> dict[str, Any]:
        return self.client.current_map()

    # ==================================================================
    # Stop / emergency stop
    # ==================================================================
    def stop(self) -> dict[str, Any]:
        return self.client.stop("WaterChassis.stop() 正常停车")

    def estop(self) -> dict[str, Any]:
        return self.client.estop()

    def release_estop(self) -> dict[str, Any]:
        """解除软件急停。实体急停仍需在机器人上物理解除。"""
        return self.client.release_estop()

    # ==================================================================
    # 可选中文别名
    # ==================================================================
    # Python 3 支持中文标识符。为了兼顾可读性和跨团队协作，文档仍推荐
    # 以英文方法名作为正式接口；下面这些别名仅作为中文团队的便利入口。
    获取配置 = get_config
    设置反馈 = set_feedback
    获取状态 = get_state
    获取健康状态 = get_health
    获取能力 = get_capabilities
    获取位姿 = get_pose
    获取速度 = get_velocity
    获取电量 = get_power
    获取安全状态 = get_safety
    获取运动状态 = get_motion_state
    获取动作反馈 = get_feedback_state
    是否就绪 = is_ready

    设置速度 = set_velocity
    定时运动 = drive_for
    定时原地转 = spin_for
    相对转向 = rotate_by
    转到绝对航向 = rotate_to
    行驶距离 = drive_distance
    相对移动 = move_relative
    前进 = forward
    后退 = backward
    左转 = turn_left
    右转 = turn_right

    相对自主导航 = navigate_relative
    导航到 = navigate_to
    获取任务 = get_task
    等待任务 = wait_task
    取消导航 = cancel_navigation

    获取机器人信息 = get_robot_info
    获取诊断 = get_diagnosis
    获取导航参数 = get_navigation_params
    设置导航速度 = set_navigation_speed
    获取规划路径 = get_planned_path
    查询可达点 = query_accessible_point
    查询障碍距离 = query_obstacle_distance
    规划距离 = plan_distance
    获取地图列表 = get_map_list
    获取当前地图 = get_current_map

    停止 = stop
    急停 = estop
    解除急停 = release_estop


# 可选中文类名别名。正式项目仍建议 ``WaterChassis``。
Water底盘 = WaterChassis
