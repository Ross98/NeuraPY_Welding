"""
Example 8: Composite weld path (复合焊缝)
=========================================

使用 move_composite 串联：直线 + 圆弧 + 直线 + 摆动
常见于：长焊缝中段有管接头，需要绕过接头。
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController
from welding_package.parameters import WeaveParameters


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="welding_torch") as wc:

        # 准备路点
        p0 = [0.30, 0.00, 0.20, 3.1416, 0.0, 0.0]  # 起点
        p1 = [0.40, 0.00, 0.20, 3.1416, 0.0, 0.0]  # 直线终点
        p2 = [0.45, 0.05, 0.20, 3.1416, 0.0, 0.0]  # 圆弧中间点
        p3 = [0.50, 0.00, 0.20, 3.1416, 0.0, 0.0]  # 圆弧终点 (回到直线)
        p4 = [0.60, 0.00, 0.20, 3.1416, 0.0, 0.0]  # 最终点

        approach = copy_approach(p0)

        # 接近
        r.move_pose(approach)

        # 起弧
        wc.arc_on()

        # 构造复合运动
        commands = [
            {"linear":   {"blend_radius": 0.002, "target_pose": [p0, p1]}},
            {"circular": {"blend_radius": 0.002, "target_pose": [p1, p2, p3]}},
            {"linear":   {"blend_radius": 0.002, "target_pose": [p3, p4]}},
        ]

        # 摆动: 仅作用于第一段直线 (start=1, end=2)
        weave = WeaveParameters(pattern="sine", frequency=1.0,
                                amplitude_left=0.002, amplitude_right=0.002)
        weave_dict = weave.to_api_dict()
        weave_dict["start_target_pose_number"] = 1
        weave_dict["end_target_pose_number"] = 2

        composite_prop = {
            "speed": 0.010,
            "acceleration": 0.1,
            "jerk": 100.0,
            "blending_mode": "static",
            "blend_radius": 0.002,
            "commands": commands,
            "weaving": True,
            "weaving_parameters": [weave_dict],
            "current_joint_angles": r.get_current_joint_angles(),
        }
        feas, _ = r.check_composite_feasibility(**composite_prop)
        if not feas:
            raise RuntimeError("Composite weld not feasible")

        r.move_composite(**composite_prop)
        r.wait_motion_finished()

        wc.arc_off()
        r.move_pose(approach)


def copy_approach(p, dz=0.05):
    out = list(p)
    out[2] += dz
    return out


if __name__ == "__main__":
    main()
