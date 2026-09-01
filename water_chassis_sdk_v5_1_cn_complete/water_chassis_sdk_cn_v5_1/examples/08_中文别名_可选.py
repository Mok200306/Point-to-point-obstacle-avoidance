"""中文别名只是方便初学者理解，正式团队项目仍推荐英文 API。"""
from water_chassis_sdk import Water底盘


def main():
    with Water底盘() as robot:
        print(robot.获取状态(refresh=True))

        # robot.前进(0.30)
        # robot.左转(30)
        # robot.相对移动(20, 0.30)
        robot.停止()


if __name__ == "__main__":
    main()
