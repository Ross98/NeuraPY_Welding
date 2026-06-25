"""
Example 6: Cobot Seam Tracking (协作力控焊缝跟踪)
==================================================

在 Z 方向施加 8N 的接触力，机器人自动适应工件微小高度变化。
适用于：
  - 工件有热变形
  - 工件装夹有 1-2mm 偏差
  - 与人协作的共享空间
"""
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingProcess
from welding_package.parameters import SeamTrackParameters


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)

    # 焊缝路径 (实际应从 CAD/CAM 离线编程)
    seam_path = [
        [0.40, -0.30, 0.18, 3.1416, 0.0, 0.0],
        [0.45, -0.20, 0.18, 3.1416, 0.0, 0.0],
        [0.50, -0.10, 0.18, 3.1416, 0.0, 0.0],
        [0.55,  0.00, 0.18, 3.1416, 0.0, 0.0],
        [0.60,  0.10, 0.18, 3.1416, 0.0, 0.0],
    ]

    with WeldingController(r, weld_tool="welding_torch") as wc:
        # 启用 8N 接触力 + hybrid 力/位混合控制
        seam_track = SeamTrackParameters(
            control_mode="hybrid",
            force_vector={
                "Fx": {"is_active": False, "value": 0.0},
                "Fy": {"is_active": False, "value": 0.0},
                "Fz": {"is_active": True,  "value": 8.0},
            },
        )
        proc = WeldingProcess.cobot_track(
            seam_path, seam_track=seam_track, speed=0.008
        )
        wc.run(proc)


if __name__ == "__main__":
    main()
