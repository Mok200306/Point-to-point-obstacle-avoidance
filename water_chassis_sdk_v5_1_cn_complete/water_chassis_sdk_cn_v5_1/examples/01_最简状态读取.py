from water_chassis_sdk import WaterChassis


def main():
    with WaterChassis() as robot:
        state = robot.get_state(refresh=True)
        print("位姿:", state["pose"])
        print("速度:", state["velocity"])
        print("电量:", state["power"])
        print("安全:", state["safety"])


if __name__ == "__main__":
    main()
