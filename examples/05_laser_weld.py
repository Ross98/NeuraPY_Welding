"""
Example 5: Laser Welding (激光焊)
=================================

光纤激光器，通过模拟量 AO_LASER_POWER 设置功率 (W)，
通过 DO_LASER_ON 开启激光出光，DO_LASER_GAS 控制保护气。
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="laser_head",
                           mode=WeldingMode.LASER) as wc:
        start = [0.45, 0.00, 0.25, 3.1416, 0.0, 0.0]
        end   = [0.55, 0.00, 0.25, 3.1416, 0.0, 0.0]

        proc = WeldingProcess.laser(
            start, end,
            laser_power=1500.0,      # 1500 W
            scan_freq=200.0,         # 200 Hz 振镜扫描
            speed=0.025,             # 25 mm/s
        )
        wc.run(proc)


if __name__ == "__main__":
    main()
