"""WATER SDK 最小调用示例。

这个文件默认只读取状态，不会自动让机器人移动。
确认真车环境安全后，可取消下方动作代码的注释进行测试。
"""
from water_chassis_sdk import WaterChassis


def main():
    # 推荐写法：with 代码块结束时会自动停车并释放 SDK 自己启动的资源。
    with WaterChassis() as robot:
        # 1. 获取状态
        state = robot.get_state(refresh=True)
        print("\n===== WATER 当前状态 =====")
        print("位姿：", state["pose"])
        print("速度：", state["velocity"])
        print("电量：", state["power"])
        print("安全：", state["safety"])
        print("是否可运动：", state.get("ready_to_move"))

        # 2. 运动前建议做一次安全判断
        if not robot.is_ready():
            print("底盘当前未就绪，本次不执行运动。")
            return

        # ============================================================
        # 下面是常用动作。第一次接真车时，建议一次只取消一个动作的注释。
        # ============================================================

        # 前进 30 cm
        # robot.forward(0.30)

        # 后退 20 cm
        # robot.backward(0.20)

        # 原地左转 30°
        # robot.turn_left(30)

        # 原地右转 30°
        # robot.turn_right(30)

        # 相对当前车头：先向左转 20°，再前进 50 cm
        # robot.move_relative(heading_deg=20, distance_m=0.50)

        # 按固定 v/ω 运行 2 秒：这里是原地左转
        # robot.drive_for(linear_mps=0.0, angular_rps=0.15, duration_s=2.0)

        # 3. 正常停车
        robot.stop()


if __name__ == "__main__":
    main()
