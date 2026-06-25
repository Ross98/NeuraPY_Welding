"""
Example 4: Spot Weld (点焊)
============================

在 6 个铆接位置依次进行电阻点焊。
每个点停留 1.2 秒，IO 信号：
  - DO_SPOT_PRESS: 电极加压
  - DO_SPOT_WELD : 通电焊接
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="spot_gun",
                           mode=WeldingMode.SPOT) as wc:

        # 6 个铆接位置 (示意)
        spots = [
            [0.30,  0.10, 0.12, 3.1416, 0.0, 0.0],
            [0.30,  0.15, 0.12, 3.1416, 0.0, 0.0],
            [0.30,  0.20, 0.12, 3.1416, 0.0, 0.0],
            [0.30, -0.10, 0.12, 3.1416, 0.0, 0.0],
            [0.30, -0.15, 0.12, 3.1416, 0.0, 0.0],
            [0.30, -0.20, 0.12, 3.1416, 0.0, 0.0],
        ]
        for i, p in enumerate(spots, 1):
            print(f"Spot {i}/{len(spots)}")
            proc = WeldingProcess.spot(p, dwell=1.2)
            wc.run(proc)
            time.sleep(0.3)


if __name__ == "__main__":
    main()
