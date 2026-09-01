"""Synchronous Python client for WATER Chassis Gateway V5.

Normal perception/planning code should usually import :class:`WaterChassis` from
``water_chassis.py`` instead. This lower layer is useful for remote/C++-adjacent
integration, tests, or callers that deliberately run the HTTP gateway separately.
"""
from __future__ import annotations

import math
import time
import uuid
from typing import Any

import requests


TERMINAL_TASK_STATES = {"SUCCEEDED", "FAILED", "CANCELED", "REJECTED"}


class WaterApiError(RuntimeError):
    pass


class WaterChassisClient:
    def __init__(
        self,
        base_url: str = "http://127.0.0.1:8080",
        timeout_s: float = 3.0,
        source: str | None = None,
    ):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.session = requests.Session()
        # One stable controller identity for a normal cmd_vel stream.
        self.source = source or f"python-{uuid.uuid4().hex[:8]}"

    def close(self) -> None:
        self.session.close()

    def _request(self, method: str, path: str, **kwargs) -> dict[str, Any]:
        kwargs.setdefault("timeout", self.timeout_s)
        try:
            r = self.session.request(method, self.base_url + path, **kwargs)
        except requests.RequestException as exc:
            raise WaterApiError(f"Gateway 请求失败： {exc}") from exc
        try:
            data = r.json()
        except Exception:
            data = {"raw": r.text}
        if r.status_code >= 400:
            raise WaterApiError(f"HTTP {r.status_code}: {data}")
        return data

    # ------------------------------------------------------------------
    # Read-only / diagnostics
    # ------------------------------------------------------------------
    def health(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/health")

    def state(self, *, refresh: bool = False) -> dict[str, Any]:
        if refresh:
            return self._request("POST", "/api/v1/chassis/refresh")
        return self._request("GET", "/api/v1/chassis/state")

    def capabilities(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/capabilities")

    def robot_info(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/chassis/info").get("results", {})

    def diagnosis(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/chassis/diagnosis").get("results", {})

    def task(self, task_id: str) -> dict[str, Any]:
        return self._request("GET", f"/api/v1/tasks/{task_id}")["task"]

    # ------------------------------------------------------------------
    # Vendor autonomous navigation
    # ------------------------------------------------------------------
    def navigate_relative(
        self,
        heading_deg: float,
        distance_m: float,
        *,
        wait: bool = True,
        timeout_s: float = 60.0,
        distance_tolerance_m: float = 0.05,
        yaw_tolerance_deg: float = 6.0,
        max_continuous_retries: int = 30,
        replace_current: bool = False,
        final_yaw_deg: float | None = None,
    ) -> dict[str, Any]:
        """Use WATER's own map navigation + obstacle avoidance.

        ``heading_deg`` is relative to the current robot heading: 0=front,
        positive=left, negative=right. This route goes through ``/api/move``.
        """
        payload: dict[str, Any] = {
            "request_id": f"caller-{uuid.uuid4().hex[:12]}",
            "heading_rad": math.radians(float(heading_deg)),
            "heading_frame": "base_link",
            "distance_m": float(distance_m),
            "distance_tolerance_m": float(distance_tolerance_m),
            "yaw_tolerance_rad": math.radians(float(yaw_tolerance_deg)),
            "timeout_s": float(timeout_s),
            "max_continuous_retries": int(max_continuous_retries),
            "replace_current": bool(replace_current),
            "generated_at_ms": int(time.time() * 1000),
            "max_age_ms": 1000,
        }
        if final_yaw_deg is not None:
            payload["final_yaw_rad"] = math.radians(float(final_yaw_deg))
            payload["final_yaw_frame"] = "base_link"

        result = self._request("POST", "/api/v1/motion/relative", json=payload, timeout=5)
        task = result["task"]
        if not wait:
            return task
        return self.wait_task(task["task_id"], timeout_s=timeout_s + 5.0)

    # Backward-compatible name. In V5 the high-level WaterChassis.move_relative()
    # defaults to the new direct closed-loop primitive; this low-level HTTP client
    # keeps move_relative as vendor NAV to avoid silently changing REST semantics.
    move_relative = navigate_relative

    def navigate_to(
        self,
        x_m: float,
        y_m: float,
        yaw_deg: float,
        *,
        wait: bool = True,
        timeout_s: float = 60.0,
        distance_tolerance_m: float = 0.05,
        yaw_tolerance_deg: float = 6.0,
        max_continuous_retries: int = 30,
        replace_current: bool = False,
    ) -> dict[str, Any]:
        payload = {
            "request_id": f"caller-{uuid.uuid4().hex[:12]}",
            "x_m": float(x_m),
            "y_m": float(y_m),
            "yaw_rad": math.radians(float(yaw_deg)),
            "distance_tolerance_m": float(distance_tolerance_m),
            "yaw_tolerance_rad": math.radians(float(yaw_tolerance_deg)),
            "timeout_s": float(timeout_s),
            "max_continuous_retries": int(max_continuous_retries),
            "replace_current": bool(replace_current),
            "generated_at_ms": int(time.time() * 1000),
            "max_age_ms": 1000,
        }
        result = self._request("POST", "/api/v1/motion/goal", json=payload, timeout=5)
        task = result["task"]
        if not wait:
            return task
        return self.wait_task(task["task_id"], timeout_s=timeout_s + 5.0)

    move_map = navigate_to

    def wait_task(
        self,
        task_id: str,
        timeout_s: float = 60.0,
        *,
        print_progress: bool = False,
        poll_s: float = 0.5,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            task = self.task(task_id)
            if print_progress:
                st = self.state()
                pose = st.get("pose", {})
                motion = st.get("motion", {})
                print(
                    f"task={task['state']:<10} "
                    f"pose=({pose.get('x_m')}, {pose.get('y_m')}, {pose.get('yaw_rad')}) "
                    f"remain={motion.get('euclidean_distance_to_goal_m')} "
                    f"phase={motion.get('vendor_running_status')} "
                    f"retry={motion.get('vendor_move_retry_times')}"
                )
            if task["state"] in TERMINAL_TASK_STATES:
                return task
            time.sleep(float(poll_s))
        self.stop("等待导航任务超时")
        raise TimeoutError(f"任务 {task_id} 在 {timeout_s:.1f}s 内未结束")

    def cancel_navigation(self) -> dict[str, Any]:
        return self._request("POST", "/api/v1/motion/cancel")

    # ------------------------------------------------------------------
    # Direct v/w control
    # ------------------------------------------------------------------
    def set_velocity(
        self,
        linear_mps: float,
        angular_rps: float,
        *,
        replace_current: bool = False,
    ) -> dict[str, Any]:
        """Send one cmd_vel frame.

        Continuous motion requires repeated calls (normally 10 Hz). If frames stop,
        the Gateway watchdog sends zero velocity automatically.
        """
        payload = {
            "source": self.source,
            "linear_mps": float(linear_mps),
            "angular_rps": float(angular_rps),
            "replace_current": bool(replace_current),
            "generated_at_ms": int(time.time() * 1000),
            "max_age_ms": 300,
        }
        return self._request("PUT", "/api/v1/motion/velocity", json=payload)

    def drive_for(
        self,
        linear_mps: float,
        angular_rps: float,
        duration_s: float,
        *,
        rate_hz: float = 10.0,
        replace_current: bool = False,
    ) -> dict[str, Any]:
        """在指定时间内持续发送固定 v/ω，结束后无论是否异常都会停车。"""
        duration_s = float(duration_s)
        rate_hz = float(rate_hz)
        if duration_s <= 0:
            raise ValueError("duration_s 必须大于 0")
        if rate_hz < 5.0:
            raise ValueError("rate_hz 必须不小于 5 Hz")

        start = time.monotonic()
        deadline = start + duration_s
        period = 1.0 / rate_hz
        first = True
        try:
            while time.monotonic() < deadline:
                self.set_velocity(
                    linear_mps,
                    angular_rps,
                    replace_current=(replace_current if first else False),
                )
                first = False
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(min(period, remaining))
        finally:
            try:
                self.stop("drive_for 执行结束")
            except Exception:
                pass
        return {
            "ok": True,
            "linear_mps": float(linear_mps),
            "angular_rps": float(angular_rps),
            "duration_s": time.monotonic() - start,
        }

    def spin_for(
        self,
        angular_rps: float = 0.25,
        duration_s: float = 3.0,
        *,
        rate_hz: float = 10.0,
    ) -> dict[str, Any]:
        return self.drive_for(0.0, angular_rps, duration_s, rate_hz=rate_hz)

    def rotate_by_direct(
        self,
        angle_deg: float,
        *,
        max_angular_rps: float | None = None,
        tolerance_deg: float | None = None,
        timeout_s: float | None = None,
        rate_hz: float | None = None,
        replace_current: bool = True,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "angle_rad": math.radians(float(angle_deg)),
            "timeout_s": timeout_s,
            "replace_current": bool(replace_current),
        }
        if max_angular_rps is not None:
            payload["max_angular_rps"] = float(max_angular_rps)
        if tolerance_deg is not None:
            payload["tolerance_rad"] = math.radians(float(tolerance_deg))
        if rate_hz is not None:
            payload["rate_hz"] = float(rate_hz)
        request_timeout = float(timeout_s or max(12.0, abs(float(angle_deg)) / 10.0 + 8.0))
        return self._request(
            "POST", "/api/v1/motion/direct/rotate", json=payload,
            timeout=request_timeout + 5.0,
        )

    def drive_distance_direct(
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
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "distance_m": float(distance_m),
            "timeout_s": timeout_s,
            "replace_current": bool(replace_current),
        }
        if speed_mps is not None:
            payload["speed_mps"] = abs(float(speed_mps))
        if tolerance_m is not None:
            payload["tolerance_m"] = float(tolerance_m)
        if heading_hold is not None:
            payload["heading_hold"] = bool(heading_hold)
        if max_heading_correction_rps is not None:
            payload["max_heading_correction_rps"] = abs(float(max_heading_correction_rps))
        if rate_hz is not None:
            payload["rate_hz"] = float(rate_hz)

        speed_for_timeout = abs(float(speed_mps or 0.10))
        request_timeout = float(
            timeout_s
            or max(12.0, abs(float(distance_m)) / max(speed_for_timeout, 0.03) * 3.0 + 8.0)
        )
        return self._request(
            "POST", "/api/v1/motion/direct/distance", json=payload,
            timeout=request_timeout + 5.0,
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
        rate_hz: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "heading_rad": math.radians(float(heading_deg)),
            "distance_m": float(distance_m),
            "replace_current": bool(replace_current),
            "timeout_s": timeout_s,
        }
        if linear_speed_mps is not None:
            payload["linear_speed_mps"] = abs(float(linear_speed_mps))
        if angular_speed_rps is not None:
            payload["angular_speed_rps"] = abs(float(angular_speed_rps))
        if distance_tolerance_m is not None:
            payload["distance_tolerance_m"] = float(distance_tolerance_m)
        if angle_tolerance_deg is not None:
            payload["angle_tolerance_rad"] = math.radians(float(angle_tolerance_deg))
        if rate_hz is not None:
            payload["rate_hz"] = float(rate_hz)

        linear_for_timeout = abs(float(linear_speed_mps or 0.10))
        angular_for_timeout = abs(float(angular_speed_rps or 0.30))
        estimated = (
            abs(math.radians(float(heading_deg))) / max(angular_for_timeout, 0.05)
            + abs(float(distance_m)) / max(linear_for_timeout, 0.03)
        )
        request_timeout = float(timeout_s or max(15.0, estimated * 3.0 + 10.0))
        return self._request(
            "POST", "/api/v1/motion/direct/relative", json=payload,
            timeout=request_timeout + 5.0,
        )

    # ------------------------------------------------------------------
    # Vendor NAV configuration / map helpers
    # ------------------------------------------------------------------
    def get_navigation_params(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/navigation/params").get("results", {})

    def set_navigation_params(
        self,
        *,
        max_speed_linear: float | None = None,
        max_speed_angular: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        if max_speed_linear is not None:
            payload["max_speed_linear"] = float(max_speed_linear)
        if max_speed_angular is not None:
            payload["max_speed_angular"] = float(max_speed_angular)
        return self._request("PUT", "/api/v1/navigation/params", json=payload)

    def planned_path(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/navigation/path").get("results", {})

    def plan_distance(
        self,
        start_x: float,
        start_y: float,
        start_floor: int,
        goal_x: float,
        goal_y: float,
        goal_floor: int,
    ) -> float:
        payload = {
            "start_x": float(start_x),
            "start_y": float(start_y),
            "start_floor": int(start_floor),
            "goal_x": float(goal_x),
            "goal_y": float(goal_y),
            "goal_floor": int(goal_floor),
        }
        results = self._request("POST", "/api/v1/navigation/plan-distance", json=payload).get("results", {})
        return float(results["distance"])

    def accessible_point(self, x_m: float, y_m: float) -> dict[str, Any]:
        return self._request(
            "GET", "/api/v1/map/accessible-point", params={"x_m": x_m, "y_m": y_m}
        ).get("results", {})

    def distance_probe(self, x_m: float, y_m: float) -> dict[str, Any]:
        return self._request(
            "GET", "/api/v1/map/distance-probe", params={"x_m": x_m, "y_m": y_m}
        ).get("results", {})

    def map_list(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/map/list").get("results", {})

    def current_map(self) -> dict[str, Any]:
        return self._request("GET", "/api/v1/map/current").get("results", {})

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------
    def stop(self, reason: str = "调用者请求停车") -> dict[str, Any]:
        return self._request("POST", "/api/v1/motion/stop", json={"reason": reason})

    def estop(self) -> dict[str, Any]:
        return self._request("POST", "/api/v1/safety/estop", json={"engaged": True})

    def release_estop(self) -> dict[str, Any]:
        # Explicit on purpose: never auto-release e-stop.
        return self._request("POST", "/api/v1/safety/estop", json={"engaged": False})
