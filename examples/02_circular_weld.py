"""
Example 2: Circular Weld (圆周焊)
=================================

围绕一个直径 80mm 的圆环法兰进行圆周焊。
move_circular 至少需要 3 个点确定一段圆弧。
"""
import time
import numpy as np
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess
from welding_package.parameters import WeaveParameters


def make_circle_poses(center, radius, z, rpy=None, n=12):
    """在 XOY 平面生成 n 个均匀分布的圆周点 (TCP 朝下, Rz 不变)."""
    rpy = rpy or [3.1416, 0.0, 0.0]
    poses = []
    for i in range(n):
        theta = 2 * np.pi * i / n
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        poses.append([x, y, z, *rpy])
    # 闭合：再加起点
    poses.append(poses[0])
    return poses


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="welding_torch") as wc:
        # 圆心在 (0.50, 0.00), 半径 0.04 (40 mm), 高度 0.18
        poses = make_circle_poses(
            center=[0.50, 0.00], radius=0.04, z=0.18,
            rpy=[3.1416, 0.0, 0.0], n=12
        )
        proc = WeldingProcess.circular(
            poses, speed=0.010,             # 10 mm/s
            weave=WeaveParameters(pattern="sine"),
        )
        wc.run(proc)


if __name__ == "__main__":
    main()
