"""
Example 1: Linear Weave Weld (直线摆动焊)
==========================================

焊接一条 200mm 长的对接焊缝，使用 sine 摆动。

注意：
- 以下 pose 是示意值，实际使用前请在 GUI 中示教得到
- 工具名称 'welding_torch' 需与 GUI 中创建的工具名一致
"""
import time
import numpy as np
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess
from welding_package.parameters import WeaveParameters


def main():
    r = Robot("192.168.2.13")          # 机器人 IP
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="welding_torch",
                           mode=WeldingMode.MIG_MAG) as wc:

        # 起弧点 / 收弧点 (X, Y, Z, Rx, Ry, Rz) 单位 m / rad
        start = [0.40, 0.10, 0.20, 3.1416, 0.0, 0.0]
        end   = [0.60, 0.10, 0.20, 3.1416, 0.0, 0.0]

        # 1) 先 dry-run 验证轨迹
        proc = WeldingProcess.linear_weave(
            start, end,
            speed=0.012,                                       # 12 mm/s
            weave=WeaveParameters(
                pattern="sine",
                frequency=1.0,                                 # 1 Hz
                amplitude_left=0.003, amplitude_right=0.003,   # ±3mm
                dwell_time_left=0.05, dwell_time_right=0.05,
            ),
        )
        wc.dry_run(proc)
        time.sleep(0.5)

        # 2) 正式焊接
        wc.run(proc)


if __name__ == "__main__":
    main()
