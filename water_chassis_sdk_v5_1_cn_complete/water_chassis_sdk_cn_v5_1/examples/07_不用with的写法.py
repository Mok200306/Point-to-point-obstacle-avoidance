from water_chassis_sdk import WaterChassis


def main():
    robot = WaterChassis()
    try:
        print(robot.get_state(refresh=True))
        # robot.forward(0.20)
    finally:
        # 不用 with 时，务必在 finally 中 close()。
        # close() 会先正常停车，再清理 SDK 自己启动的 Gateway。
        robot.close()


if __name__ == "__main__":
    main()
