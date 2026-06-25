"""
Example 9: EIP (Ethernet/IP) 焊接
====================================

与焊机通过 Ethernet/IP 工业总线通讯，常见于：
  - Fronius TPS/i Fronius TPS 400i
  - Lincoln Power Wave S350 / S500
  - Miller Auto-Continuum 350
  - OTC DAIHEN Welbee

前置条件：
  1) 控制器已配置 EIP YAML (activate_ethernetIP_interface)
  2) 机器人作为 EIP Scanner 已上线, 焊机作为 Adapter 已连接
  3) /etc/neurapy/welder.json 已注册 (用于 set_external_device_actions)

工作模式：
  - 轮询: Python 端 write_job / arc_on
  - 触发: 注册位姿触发的焊机函数, 控制器自动调用
"""
import time
import numpy as np
from neurapy.robot import Robot
from welding_package import (
    WeldingController, WeldingProcess, WeldingMode,
    EIPWeldingLink, EIPWeldingConfig, EIPWeldingTagMap,
    EIPBackedWeldingController,
)


def example_polling_mode():
    """模式 1: Python 端轮询 - 简单直接"""
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)
    if r.is_robot_in_teach_mode():
        r.switch_to_automatic_mode()

    # 1) 建立 EIP 链路
    cfg = EIPWeldingConfig(
        yaml_path="/etc/neurapy/eip_welder.yaml",
        tags=EIPWeldingTagMap(
            # 字段名末尾的 _N 标识该 tag 在 DBLOUT 中的 0-based 索引
            dout_job_0="DOUT_weld_job",
            dout_current_1="DOUT_weld_current",
            dout_voltage_2="DOUT_weld_voltage",
            dout_wire_feed_3="DOUT_wire_feed",
            dout_gas_flow_4="DOUT_gas_flow",
            dout_arc_on="DOUT_arc_on",
            dout_wire_feed_on="DOUT_wire_feed_on",
            dout_gas_on="DOUT_gas_on",
            din_status="DINT_weld_status",
            din_error="DINT_error_code",
            din_current_actual="DINT_current_actual",
            din_voltage_actual="DINT_voltage_actual",
        ),
    )
    link = EIPWeldingLink(r, cfg)
    link.connect()

    try:
        # 2) 写焊接参数
        link.write_job(
            job=12,
            current=180.0,
            voltage=22.0,
            wire_feed=5.0,
            gas_flow=15.0,
        )

        # 3) 走到起点
        start = [0.40, 0.10, 0.20, 3.1416, 0, 0]
        end   = [0.60, 0.10, 0.20, 3.1416, 0, 0]
        r.move_pose([start[0], start[1], start[2] + 0.05,
                     start[3], start[4], start[5]])

        # 4) 起弧 (等 ARC_OK 状态位)
        if not link.arc_on(wait_ready=True, timeout=3.0):
            raise RuntimeError("Arc ignition failed")

        # 5) 焊接运动
        proc = WeldingProcess.linear_weave(
            start, end,
            speed=0.012,
        )
        # 借助原有 WeldingController 跑工艺 (但用 EIP 替换起弧)
        # 这里直接调用 move_linear
        linear_prop = {
            "speed": 0.012, "acceleration": 0.1, "jerk": 100.0,
            "blend_radius": 0.001, "blending_mode": "static",
            "enable_blending": True,
            "target_pose": [start, end],
            "current_joint_angles": r.get_current_joint_angles(),
        }
        r.check_linear_feasibility(**linear_prop)
        r.move_linear(**linear_prop)
        r.wait_motion_finished()

        # 6) 收弧
        link.arc_off()

        # 7) 实时监控: 实际电流电压
        actual_i = link.read_actual_current()
        actual_u = link.read_actual_voltage()
        print(f"Final actual: I={actual_i:.1f}A U={actual_u:.1f}V")

        # 8) 检查焊机错误
        if link.is_in_error():
            err = link.read_error_code()
            print(f"Welder error code: {err}")
    finally:
        link.disconnect()
        r.stop()


def example_pose_triggered_mode():
    """模式 2: 位姿触发的焊机函数调用 (低延迟)"""
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)
    if r.is_robot_in_teach_mode():
        r.switch_to_automatic_mode()

    cfg = EIPWeldingConfig(yaml_path="/etc/neurapy/eip_welder.yaml")
    link = EIPWeldingLink(r, cfg)
    link.connect()

    try:
        # 1) 定义焊接路径
        p0 = [0.40, 0.10, 0.20, 3.1416, 0, 0]
        p1 = [0.50, 0.10, 0.20, 3.1416, 0, 0]
        p2 = [0.60, 0.10, 0.20, 3.1416, 0, 0]

        # 2) 注册位姿触发的焊机动作
        link.register_pose_triggered_actions(
            device_json="welder.json",
            trigger_poses=[
                # 接近 p0 时调用 set_params (预置参数)
                ([p0[0], p0[1], p0[2] + 0.05, *p0[3:]], "set_params",
                 {"job": 12, "current": 180, "voltage": 22}),
                # 到达 p0 时起弧
                (p0, "arc_on", {}),
                # 中点 p1 调整参数
                (p1, "adjust", {"current": 200, "voltage": 24}),
                # 到达 p2 时收弧
                (p2, "arc_off", {}),
            ],
        )

        # 3) 走到接近点
        r.move_pose(p0)

        # 4) 启动焊接 (EIP 触发由控制器内部完成)
        linear_prop = {
            "speed": 0.010, "acceleration": 0.1, "jerk": 100.0,
            "blend_radius": 0.001, "blending_mode": "static",
            "enable_blending": True,
            "target_pose": [p0, p2],
            "current_joint_angles": r.get_current_joint_angles(),
        }
        r.move_linear(**linear_prop)
        r.wait_motion_finished()
    finally:
        link.disconnect()
        r.stop()


def example_eip_backed_controller():
    """模式 3: 用 EIPBackedWeldingController 像 WeldingController 一样使用"""
    from welding_package import WeldingProcess
    r = Robot("192.168.2.13")
    cfg = EIPWeldingConfig(yaml_path="/etc/neurapy/eip_welder.yaml")
    link = EIPWeldingLink(r, cfg)
    eip_ctrl = EIPBackedWeldingController(r, link, device_json="welder.json")

    with eip_ctrl as ctrl:
        # 写参数
        link.write_job(job=12, current=180, voltage=22, wire_feed=5.0, gas_flow=15.0)
        # 走工艺 (起弧/收弧由 EIPBackedWeldingController 在 EIP 模式下自动完成)
        proc = WeldingProcess.linear_weave(
            [0.40, 0.10, 0.20, 3.1416, 0, 0],
            [0.60, 0.10, 0.20, 3.1416, 0, 0],
        )
        # 注意: 工艺里的 controller.arc_on/arc_off 仍是 Digital IO 方式
        # 用户应改用 link.arc_on / link.arc_off 显式控制


if __name__ == "__main__":
    example_polling_mode()
