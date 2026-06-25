"""
Example 3: Multi-pass Weld (多层多道焊)
========================================

对一条厚板 V 形坡口进行 3 层焊接。
- 每层偏移 2mm (Z方向叠加)
- 每道之间等待层间温度 (2s)
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    with WeldingController(r, weld_tool="welding_torch",
                           mode=WeldingMode.TIG) as wc:
        start = [0.40, -0.20, 0.15, 3.1416, 0.0, 0.0]
        end   = [0.60, -0.20, 0.15, 3.1416, 0.0, 0.0]

        proc = WeldingProcess.multi_pass(
            start, end,
            passes=3,                       # 3 层
            layer_offset_z=0.002,           # 每层抬升 2mm
            speed=0.006,                    # 6 mm/s 较慢以保证熔深
        )
        wc.run(proc)


if __name__ == "__main__":
    main()
