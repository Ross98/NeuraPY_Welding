"""
Example 10: 视觉 socket 通讯 - 相机驱动机器人跟踪
====================================================

场景: 相机 (线激光 / 弧光传感) 检测焊缝位置, 实时把目标点推给机器人,
机器人用 movelinear_online 跟随焊缝, 持续焊接.

启动方式:
  - 终端 A: 启动此脚本 (机器人端监听)
  - 终端 B: 启动相机 (作为 client 连接到机器人)
  或运行模拟相机 (脚本末尾的 fake_camera_server)
"""
import time
import json
import socket

from neurapy.robot import Robot

from welding_package import (
    VisionSocketLink,
    VisionSocketConfig,
    VisionMode,
    VisionRole,
    VisionMsgType,
    VisionProtocol,
)


def main():
    r = Robot("192.168.2.13")
    r.power_on(); time.sleep(2)
    if r.is_robot_in_teach_mode():
        r.switch_to_automatic_mode()
        time.sleep(1)

    # 1) 创建 socket 链路 (机器人作为 server 监听)
    link = VisionSocketLink(r, VisionSocketConfig(
        role=VisionRole.SERVER,
        host="0.0.0.0",
        port=9001,
        mode=VisionMode.POINT,
        max_linear_speed=0.020,    # 20mm/s, 跟踪速度
        target_tolerance=0.002,
        servo_cycle_s=0.008,        # 125Hz
        stop_on_disconnect=True,
        stop_on_heartbeat_loss=2.0,
    ))

    # 2) 注入回调: 记录/处理接收到的位姿
    pose_history = []
    def on_pose(p, ts):
        pose_history.append((ts, p))
        if len(pose_history) % 50 == 0:
            print(f"Received {len(pose_history)} poses, latest: {p}")
    link.on_pose = on_pose

    # 3) 启动 (阻塞, 直到 stop())
    try:
        print("Waiting for camera to connect on :9001 ...")
        link.start()
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        link.stop()
        print(f"Total poses: {len(pose_history)}")
        r.stop()


# --------------------------------------------------------------------- #
# 模拟相机 (开发测试用)
# --------------------------------------------------------------------- #
def fake_camera_server(host: str = "127.0.0.1", port: int = 9001,
                       num_poses: int = 100, hz: float = 50.0):
    """
    模拟相机以 50Hz 推送焊缝位置.
    用法: python 10_vision_socket.py fake_camera
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(1)
    print(f"[Fake camera] Listening on {host}:{port}")

    conn, addr = sock.accept()
    print(f"[Fake camera] Robot connected: {addr}")

    period = 1.0 / hz
    try:
        for i in range(num_poses):
            # 模拟一条 sin 波焊缝 (X 方向移动, Y 方向摆动)
            t = i * period
            pose = [
                0.40 + 0.0001 * i,                  # X 缓慢前进
                0.10 + 0.003 * (i % 20) / 20.0,     # Y 微摆
                0.20,
                3.1416, 0.0, 0.0,
            ]
            msg = {
                "type": VisionMsgType.POSE.value,
                "pose": pose,
                "frame": "world",
                "ts": time.time(),
            }
            conn.sendall(VisionProtocol.encode(msg))
            time.sleep(period)
        # 发一个 STOP
        conn.sendall(VisionProtocol.encode({"type": VisionMsgType.STOP.value}))
        print("[Fake camera] Done")
    finally:
        conn.close()
        sock.close()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "fake_camera":
        # 模拟相机端口必须与 robot 配置一致
        import os
        port = int(os.environ.get("VISION_PORT", "9001"))
        fake_camera_server(port=port)
    else:
        main()
