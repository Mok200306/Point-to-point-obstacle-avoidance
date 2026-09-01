"""Offline software regression test using the bundled mock chassis.

Run from the V5 folder:
    python tools/self_test.py

This does not test the physical WATER chassis.  It checks the packaged startup,
feedback state, rotation, distance, relative motion, and normal shutdown path.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from water_chassis_sdk import WaterChassis


def main():
    mock = subprocess.Popen(
        [sys.executable, str(ROOT / "tools" / "mock_chassis_server.py")],
        cwd=str(ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(0.6)
        with WaterChassis(
            robot_host="127.0.0.1",
            gateway_port=18080,
            feedback=False,
        ) as robot:
            assert robot.get_health()["chassis_connected"] is True
            assert robot.get_state()["pose"]["x_m"] is not None

            rot = robot.rotate_by(15, feedback=False)
            assert rot["ok"] is True

            dist = robot.drive_distance(0.15, feedback=False)
            assert dist["ok"] is True
            assert abs(dist["remaining_m"]) <= 0.03

            rel = robot.move_relative(-10, 0.10, feedback=False)
            assert rel["ok"] is True

            fb = robot.get_feedback_state()
            assert isinstance(fb, dict)
            assert fb.get("phase") == "SUCCEEDED"

        print("WATER V5 self-test: PASS")
    finally:
        mock.terminate()
        try:
            mock.wait(timeout=2.0)
        except Exception:
            mock.kill()


if __name__ == "__main__":
    main()
