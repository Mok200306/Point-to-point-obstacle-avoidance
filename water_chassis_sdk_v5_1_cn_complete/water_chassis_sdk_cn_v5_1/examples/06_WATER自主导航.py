"""使用 WATER 厂家地图规划/避障，而不是 DIRECT 局部闭环。"""
from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        # 相对当前车头生成目标点，并交给 WATER 自主导航。
        result = robot.navigate_relative(
            heading_deg=0,
            distance_m=1.0,
            wait=True,
        )
        print(result)

        # 也可以直接给地图绝对坐标：
        # robot.navigate_to(x_m=1.0, y_m=2.0, yaw_deg=90, wait=True)


if __name__ == "__main__":
    main()
