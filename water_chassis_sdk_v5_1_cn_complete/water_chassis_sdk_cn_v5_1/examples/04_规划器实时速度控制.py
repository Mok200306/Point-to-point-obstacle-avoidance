"""典型视觉/局部规划器写法：循环输出 v、ω。"""
import time
from water_chassis_sdk import WaterChassis


def planner():
    # 这里替换成你自己的视觉/规划算法。
    # 返回：线速度 m/s，角速度 rad/s。
    return 0.08, 0.15


def main():
    with WaterChassis() as robot:
        try:
            while True:
                v, w = planner()
                robot.set_velocity(v, w)
                time.sleep(0.1)  # 10 Hz
        except KeyboardInterrupt:
            print("收到 Ctrl+C，准备停车。")
        finally:
            robot.stop()


if __name__ == "__main__":
    main()
