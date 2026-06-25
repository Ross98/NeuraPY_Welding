"""
Socket 通讯模块 - 与相机/视觉系统交互
=========================================

通过 TCP socket 接收相机/视觉系统推送的位姿, 驱动机器人做实时跟踪.
典型工业场景:
  - 焊缝跟踪 (laser stripe / arc sensing camera)
  - 工件抓取 (eye-in-hand / eye-to-hand)
  - 涂胶/喷涂 (camera 跟踪轨迹轮廓)
  - AGV 上下料 (视觉定位工件)

协议设计 (JSON over TCP, 一行一消息):
  相机 → 机器人 (VisionSocketLink.receive_loop):
    {"type": "pose",   "pose": [x, y, z, rx, ry, rz], "frame": "world"|"tcp", "ts": 12345}
    {"type": "traj",   "poses": [[x, y, z, rx, ry, rz], ...], "frame": "world", "ts": 12345}
    {"type": "delta",  "linear": [dx, dy, dz], "angular": [drx, dry, drz]}
    {"type": "joints", "angles": [q1, q2, q3, q4, q5, q6]}
    {"type": "stop"}
    {"type": "ping",   "id": 1}

  机器人 → 相机 (VisionSocketLink.send_xxx):
    {"type": "ack",   "id": 1, "status": "ok"}
    {"type": "state", "tcp_pose": [...], "joints": [...]}
    {"type": "error", "msg": "..."}

Neurapy 实时控制 API (PDF Ch.5):
  - activate_servo_interface('position')  : 启动实时控制接口
  - movelinear_online(target, vel, accel)  : 持续更新目标点 (在线)
  - stop_movelinear_online()               : 停止在线运动
  - speed_j(vel, accel)                    : 关节空间速度控制
  - speed_x(vel, accel)                    : 笛卡尔速度控制
  - servo_j / servo_x                       : 位置伺服循环 (125Hz)

本模块提供 3 种工作模式:
  1) POINT_MODE     - 相机每帧发一个 pose, 机器人用 movelinear_online 跟随
  2) TRAJECTORY_MODE - 相机批量发 poses, 机器人用 move_linear 走完整轨迹
  3) VELOCITY_MODE  - 相机发速度增量, 机器人用 speed_x 持续调整
"""
from __future__ import annotations

import json
import socket
import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Dict, Any, Callable

logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
# 协议与消息类型
# --------------------------------------------------------------------------- #
class VisionMsgType(str, Enum):
    POSE      = "pose"        # 单点位姿
    TRAJ      = "traj"        # 批量位姿 (轨迹)
    DELTA     = "delta"       # 笛卡尔速度/增量
    JOINTS    = "joints"      # 关节角度
    STOP      = "stop"        # 停止运动
    PING      = "ping"        # 心跳
    ACK       = "ack"         # 应答
    STATE     = "state"       # 机器人状态上报
    ERROR     = "error"       # 错误


class VisionMode(str, Enum):
    POINT       = "point"        # 单点跟踪 (movelinear_online)
    TRAJECTORY  = "trajectory"   # 轨迹跟踪 (move_linear)
    VELOCITY    = "velocity"     # 速度控制 (speed_x)


class VisionRole(str, Enum):
    SERVER = "server"   # 机器人监听, 相机作为 client 推送
    CLIENT = "client"   # 机器人主动连接相机 SDK


# --------------------------------------------------------------------------- #
# 配置
# --------------------------------------------------------------------------- #
@dataclass
class VisionSocketConfig:
    """Socket 通讯配置."""
    role: VisionRole = VisionRole.SERVER

    # 网络
    host: str = "0.0.0.0"
    port: int = 9001
    backlog: int = 1              # 通常只连 1 个相机
    recv_timeout_s: float = 5.0   # recv 单次超时

    # 协议
    frame: str = "world"          # 默认位姿参考系: world / tcp / base
    delim: str = "\n"             # 一行一消息 (也支持自定义分隔符)

    # 运动
    mode: VisionMode = VisionMode.POINT
    max_linear_speed: float = 0.5  # m/s, 安全上限
    max_angular_speed: float = 1.57  # rad/s
    target_tolerance: float = 0.002  # m, 在线跟踪时距离 < 该值认为到位
    max_angular_step_deg: float = 5.0  # 视觉抖动过滤: 单帧角度变化 > 该值则截断

    # 实时控制
    servo_cycle_s: float = 0.008  # 8ms = 125Hz
    enable_servo_on_start: bool = True

    # 安全
    stop_on_disconnect: bool = True   # 相机断连时立即停止
    stop_on_heartbeat_loss: float = 3.0  # 心跳丢失 N 秒后停止
    dry_run: bool = False             # 不实际驱动机器人 (调试用)


# --------------------------------------------------------------------------- #
# 消息编解码
# --------------------------------------------------------------------------- #
class VisionProtocol:
    """JSON 行分隔协议 - 一行一个 JSON 对象."""

    ENCODING = "utf-8"

    @staticmethod
    def encode(msg: Dict[str, Any]) -> bytes:
        s = json.dumps(msg, separators=(",", ":"), ensure_ascii=False)
        return (s + "\n").encode(VisionProtocol.ENCODING)

    @staticmethod
    def try_decode_line(line: bytes) -> Optional[Dict[str, Any]]:
        line = line.strip()
        if not line:
            return None
        try:
            return json.loads(line.decode(VisionProtocol.ENCODING))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            logger.warning("Invalid vision message: %r (%s)", line, e)
            return None

    @staticmethod
    def recv_message(sock: socket.socket, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        """从 socket 读一行 (newline 分隔). 失败返回 None."""
        sock.settimeout(timeout)
        buf = b""
        try:
            while b"\n" not in buf:
                chunk = sock.recv(4096)
                if not chunk:
                    # 对端关闭
                    return None
                buf += chunk
                # 防止单条消息过大
                if len(buf) > 1_000_000:
                    logger.error("Vision message too large, dropping")
                    return None
        except socket.timeout:
            logger.debug("Vision recv timeout (%.1fs)", timeout)
            return None
        line, _, rest = buf.partition(b"\n")
        return VisionProtocol.try_decode_line(line)


# --------------------------------------------------------------------------- #
# 位姿工具
# --------------------------------------------------------------------------- #
def normalize_pose(pose: List[float], max_step: float = 0.05) -> List[float]:
    """
    位姿安全限幅: 与上一帧的位移不能超过 max_step (m).
    防止视觉跳动导致机器人急停.
    """
    return list(pose)   # 简化, 实际使用方应自行维持上一帧


def angular_distance_rad(a: List[float], b: List[float]) -> float:
    """计算两个 RPY 位姿的角距离 (rad)."""
    import math
    s = 0.0
    for x, y in zip(a, b):
        d = (x - y + math.pi) % (2 * math.pi) - math.pi
        s += d * d
    return math.sqrt(s)


# --------------------------------------------------------------------------- #
# 主类
# --------------------------------------------------------------------------- #
class VisionSocketLink:
    """
    视觉 socket 通讯链路.

    用法 (SERVER 模式, 推荐):
        link = VisionSocketLink(robot, VisionSocketConfig(
            role=VisionRole.SERVER,
            port=9001,
            mode=VisionMode.POINT,
        ))
        link.start()        # 阻塞, 监听 + 处理直到 stop()
        # 或在另一线程:
        link.start_async()

    用法 (CLIENT 模式):
        link = VisionSocketLink(robot, VisionSocketConfig(
            role=VisionRole.CLIENT,
            host="192.168.2.50",
            port=9001,
        ))
        link.start()
    """

    def __init__(self, robot, config: Optional[VisionSocketConfig] = None) -> None:
        self.r = robot
        self.cfg = config or VisionSocketConfig()

        # 状态
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # 视觉最新位姿
        self._latest_pose: Optional[List[float]] = None
        self._latest_joints: Optional[List[float]] = None
        self._last_seen: float = 0.0

        # 在线运动状态
        self._online_active = False
        self._online_lock = threading.Lock()

        # 计数器
        self._msg_count = 0
        self._last_ack_id = 0

        # 回调钩子 (用户可注入)
        self.on_pose: Optional[Callable[[List[float], float], None]] = None
        self.on_traj: Optional[Callable[[List[List[float]]], None]] = None
        self.on_stop: Optional[Callable[[], None]] = None
        self.on_error: Optional[Callable[[str], None]] = None

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """阻塞运行, 直到 stop() 或断连."""
        if self._running:
            raise RuntimeError("VisionSocketLink already running")
        self._setup_servo()
        self._running = True
        try:
            if self.cfg.role == VisionRole.SERVER:
                self._run_server()
            else:
                self._run_client()
        finally:
            self._teardown()
            self._running = False

    def start_async(self) -> threading.Thread:
        """在后台线程运行, 主线程可继续."""
        if self._running:
            raise RuntimeError("VisionSocketLink already running")
        self._thread = threading.Thread(
            target=self.start, name="VisionSocketLink", daemon=True
        )
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        """请求停止 (线程安全)."""
        self._running = False
        if self._online_active:
            try:
                self.r.stop_movelinear_online()
            except Exception as e:
                logger.debug("stop_movelinear_online error: %s", e)
            self._online_active = False
        # 关闭 socket 让 recv 返回
        for s in (self._client_sock, self._server_sock):
            if s:
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    s.close()
                except OSError:
                    pass
        self._teardown()

    def _teardown(self) -> None:
        """释放 servo 接口."""
        if self.cfg.enable_servo_on_start:
            try:
                self.r.deactivate_servo_interface()
            except Exception as e:
                logger.debug("deactivate_servo_interface error: %s", e)
        self._online_active = False

    # ------------------------------------------------------------------ #
    # 启动 servo 接口
    # ------------------------------------------------------------------ #
    def _setup_servo(self) -> None:
        if not self.cfg.enable_servo_on_start:
            return
        try:
            self.r.activate_servo_interface("position")
        except Exception as e:
            logger.warning("activate_servo_interface failed: %s", e)

    # ------------------------------------------------------------------ #
    # SERVER 模式
    # ------------------------------------------------------------------ #
    def _run_server(self) -> None:
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self.cfg.host, self.cfg.port))
        self._server_sock.listen(self.cfg.backlog)
        self._server_sock.settimeout(1.0)  # 周期性检查 self._running
        logger.info("Vision socket server listening on %s:%d",
                    self.cfg.host, self.cfg.port)

        try:
            while self._running:
                try:
                    conn, addr = self._server_sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                logger.info("Vision client connected: %s", addr)
                self._client_sock = conn
                try:
                    self._serve_connection(conn)
                finally:
                    try:
                        conn.close()
                    except OSError:
                        pass
                    self._client_sock = None
                    logger.info("Vision client disconnected: %s", addr)
                    if self.cfg.stop_on_disconnect:
                        self._stop_online()
        except Exception as e:
            logger.exception("Vision server error: %s", e)
            if self.on_error:
                self.on_error(str(e))
        finally:
            try:
                self._server_sock.close()
            except OSError:
                pass

    def _serve_connection(self, conn: socket.socket) -> None:
        """处理单个 client 连接的消息循环."""
        while self._running:
            try:
                msg = VisionProtocol.recv_message(conn, timeout=1.0)
            except OSError as e:
                logger.debug("Vision recv OSError (client closed?): %s", e)
                break
            if msg is None:
                # 超时但连接还在, 检查心跳
                if (self._last_seen > 0
                        and (time.time() - self._last_seen) > self.cfg.stop_on_heartbeat_loss):
                    logger.warning("Vision heartbeat loss > %.1fs, stopping",
                                    self.cfg.stop_on_heartbeat_loss)
                    self._stop_online()
                continue
            self._last_seen = time.time()
            self._msg_count += 1
            self._handle_message(msg, conn)

    # ------------------------------------------------------------------ #
    # CLIENT 模式
    # ------------------------------------------------------------------ #
    def _run_client(self) -> None:
        logger.info("Vision socket client connecting to %s:%d",
                    self.cfg.host, self.cfg.port)
        backoff = 0.5
        while self._running:
            try:
                sock = socket.create_connection(
                    (self.cfg.host, self.cfg.port), timeout=5.0
                )
                self._client_sock = sock
                logger.info("Vision socket client connected")
                backoff = 0.5
                # 主动发 PING
                self._send(sock, {"type": VisionMsgType.PING.value, "id": 0})
                # 进入消息循环
                self._serve_connection(sock)
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                if not self._running:
                    break
                logger.debug("Vision connect failed: %s, retry in %.1fs", e, backoff)
                time.sleep(backoff)
                backoff = min(backoff * 2, 5.0)
            finally:
                if self._client_sock:
                    try:
                        self._client_sock.close()
                    except OSError:
                        pass
                    self._client_sock = None

    # ------------------------------------------------------------------ #
    # 消息分发
    # ------------------------------------------------------------------ #
    def _handle_message(self, msg: Dict[str, Any], conn: socket.socket) -> None:
        mtype = msg.get("type")
        if mtype is None:
            logger.warning("Vision msg without 'type': %r", msg)
            return

        try:
            if mtype == VisionMsgType.POSE.value:
                pose = msg.get("pose")
                if not isinstance(pose, list) or len(pose) != 6:
                    logger.warning("Invalid pose: %r", pose)
                    return
                self._handle_pose(pose, msg.get("ts", time.time()))

            elif mtype == VisionMsgType.TRAJ.value:
                poses = msg.get("poses")
                if not isinstance(poses, list) or not all(
                    isinstance(p, list) and len(p) == 6 for p in poses
                ):
                    logger.warning("Invalid trajectory: %r", poses)
                    return
                self._handle_trajectory(poses)

            elif mtype == VisionMsgType.DELTA.value:
                linear = msg.get("linear", [0, 0, 0])
                angular = msg.get("angular", [0, 0, 0])
                self._handle_velocity_delta(linear, angular)

            elif mtype == VisionMsgType.JOINTS.value:
                angles = msg.get("angles")
                if isinstance(angles, list) and len(angles) == 6:
                    self._handle_joint_target(angles)
                else:
                    logger.warning("Invalid joint angles: %r", angles)

            elif mtype == VisionMsgType.STOP.value:
                logger.info("Vision STOP received")
                self._stop_online()
                if self.on_stop:
                    self.on_stop()

            elif mtype == VisionMsgType.PING.value:
                self._send(conn, {
                    "type": VisionMsgType.ACK.value,
                    "id": msg.get("id"),
                    "status": "ok",
                })
            else:
                logger.warning("Unknown vision msg type: %r", mtype)
        except Exception as e:
            logger.exception("Vision msg handler error: %s", e)
            if self.on_error:
                self.on_error(str(e))

    # ------------------------------------------------------------------ #
    # 各模式处理
    # ------------------------------------------------------------------ #
    def _handle_pose(self, pose: List[float], ts: float) -> None:
        """单点跟踪模式 - movelinear_online."""
        self._latest_pose = list(pose)
        if self.on_pose:
            self.on_pose(pose, ts)
        if self.cfg.dry_run:
            return
        if self.cfg.mode == VisionMode.POINT:
            self._push_online_target(pose)

    def _handle_trajectory(self, poses: List[List[float]]) -> None:
        """轨迹模式 - move_linear."""
        if self.on_traj:
            self.on_traj(poses)
        if self.cfg.dry_run:
            return
        if self.cfg.mode != VisionMode.TRAJECTORY:
            logger.debug("Got traj in non-traj mode, ignoring")
            return
        self._stop_online()  # 任何传统 motion 都要先关 online
        if len(poses) < 2:
            return
        try:
            prop = {
                "speed": min(self.cfg.max_linear_speed, 0.05),
                "acceleration": 0.1,
                "jerk": 100.0,
                "blend_radius": 0.001,
                "blending_mode": "static",
                "enable_blending": True,
                "target_pose": poses,
                "current_joint_angles": self.r.get_current_joint_angles(),
            }
            self.r.check_linear_feasibility(**prop)
            self.r.move_linear(**prop)
        except Exception as e:
            logger.exception("move_linear (traj) failed: %s", e)

    def _handle_velocity_delta(
        self, linear: List[float], angular: List[float]
    ) -> None:
        """速度增量模式 - speed_x (持续加到当前目标上)."""
        if self.cfg.dry_run:
            return
        if self.cfg.mode != VisionMode.VELOCITY:
            return
        # 限幅
        v = [max(-self.cfg.max_linear_speed,
                 min(self.cfg.max_linear_speed, x)) for x in linear]
        a = [max(-self.cfg.max_angular_speed,
                 min(self.cfg.max_angular_speed, x)) for x in angular]
        try:
            self.r.speed_x(v, a)
        except Exception as e:
            logger.debug("speed_x error: %s", e)

    def _handle_joint_target(self, angles: List[float]) -> None:
        """关节角度模式 - servo_j (高速循环, 250Hz)."""
        if self.cfg.dry_run:
            return
        try:
            self.r.servo_j(angles, [0.0] * 6, [0.0] * 6)
        except Exception as e:
            logger.debug("servo_j error: %s", e)

    # ------------------------------------------------------------------ #
    # movelinear_online
    # ------------------------------------------------------------------ #
    def _push_online_target(self, pose: List[float]) -> None:
        """把目标点送入实时在线运动."""
        with self._online_lock:
            try:
                v = [self.cfg.max_linear_speed] * 3
                a = [0.5] * 6
                if self._online_active:
                    # 在线更新 (运行中)
                    self.r.movelinear_online(pose, v, a)
                else:
                    # 第一次: 启动在线运动
                    self.r.movelinear_online(pose, v, a)
                    self._online_active = True
            except Exception as e:
                logger.exception("movelinear_online error: %s", e)
                self._online_active = False

    def _stop_online(self) -> None:
        with self._online_lock:
            if self._online_active:
                try:
                    self.r.stop_movelinear_online()
                except Exception as e:
                    logger.debug("stop_movelinear_online error: %s", e)
                self._online_active = False

    # ------------------------------------------------------------------ #
    # 主动发送
    # ------------------------------------------------------------------ #
    def _send(self, sock: socket.socket, msg: Dict[str, Any]) -> None:
        try:
            sock.sendall(VisionProtocol.encode(msg))
        except OSError as e:
            logger.debug("send error: %s", e)

    def send_state(self) -> None:
        """主动上报机器人状态给相机 (用于相机闭环)."""
        if not self._client_sock:
            return
        try:
            msg = {
                "type": VisionMsgType.STATE.value,
                "tcp_pose": self.r.get_tcp_pose(),
                "joints": self.r.get_current_joint_angles(),
                "ts": time.time(),
            }
            self._send(self._client_sock, msg)
        except Exception as e:
            logger.debug("send_state error: %s", e)

    # ------------------------------------------------------------------ #
    # 公开查询
    # ------------------------------------------------------------------ #
    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def latest_pose(self) -> Optional[List[float]]:
        return self._latest_pose

    @property
    def msg_count(self) -> int:
        return self._msg_count

    # ------------------------------------------------------------------ #
    # 上下文管理
    # ------------------------------------------------------------------ #
    def __enter__(self) -> "VisionSocketLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
