from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        # heading_deg:
        #   0   = 当前正前方
        #   +30 = 左前方 30°
        #   -30 = 右前方 30°
        # distance_m:
        #   正值前进，负值后退
        result = robot.move_relative(
            heading_deg=30,
            distance_m=0.50,
        )
        print("执行结果:", result)


if __name__ == "__main__":
    main()
