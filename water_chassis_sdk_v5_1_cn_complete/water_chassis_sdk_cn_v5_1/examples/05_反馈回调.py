from water_chassis_sdk import WaterChassis


def on_feedback(event):
    """动作执行过程中会周期性进入这个函数。"""
    print(
        "动作=", event.get("action"),
        "阶段=", event.get("phase"),
        "进度=", event.get("progress"),
        "位姿=", event.get("pose"),
        "速度=", event.get("velocity"),
    )


def main():
    # feedback=False：关闭 SDK 自带控制台进度打印；只保留自己的回调。
    with WaterChassis(feedback=False, feedback_callback=on_feedback) as robot:
        robot.move_relative(heading_deg=20, distance_m=0.40)


if __name__ == "__main__":
    main()
