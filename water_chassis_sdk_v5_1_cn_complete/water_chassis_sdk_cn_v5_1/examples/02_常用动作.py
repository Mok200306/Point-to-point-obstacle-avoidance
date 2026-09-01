import time
from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        if not robot.is_ready():
            raise RuntimeError("底盘当前未就绪，请先检查连接、急停、故障和位姿。")

        robot.forward(0.30)
        time.sleep(0.5)

        robot.turn_left(30)
        time.sleep(0.5)

        robot.forward(0.20)
        time.sleep(0.5)

        robot.turn_right(30)
        robot.stop()


if __name__ == "__main__":
    main()
