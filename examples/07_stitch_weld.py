"""
Example 7: Stitch Weld (断续焊)
================================

薄板拼接防变形：每焊 20mm 停 10mm。
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="welding_torch",
                           mode=WeldingMode.MIG_MAG) as wc:
        # 沿 X 方向一条直线
        seam = [
            [0.30, 0.20, 0.18, 3.1416, 0.0, 0.0],
            [0.55, 0.20, 0.18, 3.1416, 0.0, 0.0],
        ]
        proc = WeldingProcess.stitch(
            seam,
            stitch_length=0.02,   # 焊 20mm
            gap_length=0.01,      # 停 10mm
            speed=0.012,
        )
        wc.run(proc)


if __name__ == "__main__":
    main()
