"""
Welding Process Implementations
================================

每种工艺实现：
  1) 构造工艺参数 (焊缝轨迹、摆动、速度…)
  2) execute(controller) 调用 Neurapy API 完成运动
  3) 与 controller.arc_on / arc_off 协作完成起弧/收弧

所有轨迹都遵循 Neurapy 的 [X, Y, Z, Rx, Ry, Rz] 笛卡尔格式，
单位：m 与 rad。
"""
from __future__ import annotations

import time
import math
import logging
import copy
from abc import ABC, abstractmethod
from typing import List, Optional, Sequence

from .parameters import (
    WeaveParameters, MotionProfile, ArcParameters, SeamTrackParameters,
)
from .welding_controller import WeldingController
from ._interp import (
    interpolate_linear, cumdist, point_at,
)

logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- #
# 抽象基类
# --------------------------------------------------------------------------- #
class WeldingProcess(ABC):
    """所有焊接工艺的基类."""

    name: str = "WeldingProcess"

    def __init__(self, motion: Optional[MotionProfile] = None) -> None:
        self.motion = motion or MotionProfile()

    @abstractmethod
    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        """执行工艺. dry_run=True 表示只走轨迹不起弧."""

    # 工具：生成离散点轨迹
    @staticmethod
    def interpolate_linear(start: Sequence[float], end: Sequence[float],
                           step: float = 0.005) -> List[List[float]]:
        """
        在 start 和 end 之间按 step (m) 距离插值，返回路点列表。
        姿态使用 slerp (四元数球面插值)。
        """
        from scipy.spatial.transform import Slerp, Rotation
        import numpy as np
        s = np.asarray(start, dtype=float)
        e = np.asarray(end, dtype=float)
        ds = np.linalg.norm(e[:3] - s[:3])
        if ds < 1e-9:
            return [list(s), list(e)]
        n = max(int(math.ceil(ds / step)), 2)

        # 位置线性插值
        pos = np.linspace(s[:3], e[:3], n)
        # 姿态四元数插值
        rot_s = Rotation.from_euler("xyz", s[3:])
        rot_e = Rotation.from_euler("xyz", e[3:])
        slerp = Slerp([0, 1], Rotation.concatenate([rot_s, rot_e]))
        rpys = slerp(np.linspace(0, 1, n)).as_euler("xyz")

        return [list(np.concatenate([pos[i], rpys[i]])) for i in range(n)]

    # 工厂方法 (常用工艺的快速构造)
    @staticmethod
    def linear_weave(start_pose, end_pose, *,
                     speed: float = 0.01,
                     weave: Optional[WeaveParameters] = None,
                     arc: Optional[ArcParameters] = None,
                     seam_track: Optional[SeamTrackParameters] = None) -> "LinearWeaveWeld":
        motion = MotionProfile(speed=speed)
        return LinearWeaveWeld(start_pose, end_pose, motion, weave or WeaveParameters(),
                               arc or ArcParameters(), seam_track)

    @staticmethod
    def circular(poses: Sequence[Sequence[float]], *,
                 speed: float = 0.015,
                 weave: Optional[WeaveParameters] = None,
                 arc: Optional[ArcParameters] = None) -> "CircularWeld":
        return CircularWeld(list(poses), MotionProfile(speed=speed),
                            weave or WeaveParameters(), arc or ArcParameters())

    @staticmethod
    def multi_pass(start_pose, end_pose, *,
                   passes: int = 3,
                   layer_offset_z: float = 0.002,
                   speed: float = 0.008) -> "MultiPassWeld":
        return MultiPassWeld(start_pose, end_pose, passes,
                             layer_offset_z, MotionProfile(speed=speed))

    @staticmethod
    def spot(pose, dwell: float = 1.0) -> "SpotWeld":
        return SpotWeld(list(pose), dwell)

    @staticmethod
    def laser(start_pose, end_pose, *,
              laser_power: float = 2000.0,
              laser_power_scaling: float = 0.001,
              scan_freq: float = 100.0,
              speed: float = 0.02) -> "LaserWeld":
        return LaserWeld(start_pose, end_pose, laser_power,
                         laser_power_scaling, scan_freq,
                         MotionProfile(speed=speed))

    @staticmethod
    def cobot_track(poses, *,
                    seam_track: Optional[SeamTrackParameters] = None,
                    speed: float = 0.008) -> "CobotSeamTrack":
        return CobotSeamTrack(list(poses), seam_track or SeamTrackParameters(),
                              MotionProfile(speed=speed))

    @staticmethod
    def stitch(poses, stitch_length: float = 0.02, gap_length: float = 0.01,
               speed: float = 0.01) -> "StitchWeld":
        return StitchWeld(list(poses), stitch_length, gap_length,
                          MotionProfile(speed=speed))


# --------------------------------------------------------------------------- #
# 1. 直线摆动焊 (Linear Weave Weld)
# --------------------------------------------------------------------------- #
class LinearWeaveWeld(WeldingProcess):
    """
    直线焊缝 + 横向摆动 (sine / triangle / circle)。
    适用于对接、角接、搭接平直焊缝。
    """
    name = "LinearWeaveWeld"

    def __init__(self, start_pose, end_pose,
                 motion: MotionProfile, weave: WeaveParameters,
                 arc: ArcParameters,
                 seam_track: Optional[SeamTrackParameters] = None) -> None:
        super().__init__(motion)
        self.start_pose = list(start_pose)
        self.end_pose = list(end_pose)
        self.weave = weave
        self.arc = arc
        self.seam_track = seam_track

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r
        # 1) 接近起点 (PTP 到起点上方 5cm)
        approach = copy.deepcopy(self.start_pose)
        approach[2] += 0.05
        r.move_pose(approach)

        # 2) 准备工艺: 摆动开启
        weave_dict = self.weave.to_api_dict()
        # Neurapy 要求指定起止 target_pose_number
        weave_dict["start_target_pose_number"] = 1
        weave_dict["end_target_pose_number"] = 2

        # 3) 构造 move_linear property
        linear_prop = {
            "speed": self.motion.speed,
            "acceleration": self.motion.acceleration,
            "jerk": self.motion.jerk,
            "rotation_speed": self.motion.rotation_speed,
            "rotation_acceleration": self.motion.rotation_acceleration,
            "rotation_jerk": self.motion.rotation_jerk,
            "blend_radius": self.motion.blend_radius,
            "blending_mode": self.motion.blending_mode,
            "enable_blending": True,
            "target_pose": [self.start_pose, self.end_pose],
            "weaving": True,
            "weaving_parameters": [weave_dict],
            "current_joint_angles": r.get_current_joint_angles(),
        }
        # 4) 力控 / 焊缝跟踪
        if self.seam_track and self.seam_track.enabled:
            linear_prop["controller_parameters"] = {
                "control_mode": self.seam_track.control_mode,
                "force_vector": self.seam_track.force_vector,
                "torque_vector": self.seam_track.torque_vector,
            }

        # 5) 检查可行性
        feas, _ = r.check_linear_feasibility(**linear_prop)
        if not feas:
            raise RuntimeError("Linear weave weld not feasible")

        # 6) 起弧
        if not dry_run:
            controller.arc_on()

        # 7) 走到起点上方 2mm 接触
        contact = copy.deepcopy(self.start_pose)
        contact[2] = self.start_pose[2] + 0.002
        r.move_linear(target_pose=[contact])

        # 8) 实际焊接运动 (含摆动)
        r.move_linear(**linear_prop)

        # 9) 收弧
        if not dry_run:
            controller.arc_off()

        # 10) 抬枪
        r.move_pose(approach)
        logger.info("LinearWeaveWeld done")


# --------------------------------------------------------------------------- #
# 2. 圆弧 / 圆周焊 (Circular Weld)
# --------------------------------------------------------------------------- #
class CircularWeld(WeldingProcess):
    """
    圆弧 / 整圆焊接，典型应用：管法兰、环形焊缝、桶体外环缝。
    poses: 三个或以上的离散点 (move_circular 需要三个点)
    """
    name = "CircularWeld"

    def __init__(self, poses: List[List[float]], motion: MotionProfile,
                 weave: WeaveParameters, arc: ArcParameters) -> None:
        super().__init__(motion)
        if len(poses) < 3:
            raise ValueError("CircularWeld needs at least 3 poses")
        self.poses = poses
        self.weave = weave
        self.arc = arc

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r
        approach = copy.deepcopy(self.poses[0])
        approach[2] += 0.05
        r.move_pose(approach)

        circular_prop = {
            "speed": self.motion.speed,
            "acceleration": self.motion.acceleration,
            "jerk": self.motion.jerk,
            "rotation_speed": self.motion.rotation_speed,
            "rotation_acceleration": self.motion.rotation_acceleration,
            "rotation_jerk": self.motion.rotation_jerk,
            "blend_radius": self.motion.blend_radius,
            "blending_mode": self.motion.blending_mode,
            "enable_blending": True,
            "target_pose": self.poses,
            "weaving": self.weave.pattern == "circle",  # 圆周焊仅在用户显式设 circle 模式时开摆动
            "current_joint_angles": r.get_current_joint_angles(),
        }
        if self.weave.pattern == "circle":
            w = self.weave.to_api_dict()
            w["start_target_pose_number"] = 1
            w["end_target_pose_number"] = len(self.poses)
            circular_prop["weaving_parameters"] = [w]

        feas, _ = r.check_circular_feasibility(**circular_prop)
        if not feas:
            raise RuntimeError("Circular weld not feasible")

        if not dry_run:
            controller.arc_on()
        # 走到起弧点上方
        pre = copy.deepcopy(self.poses[0])
        pre[2] += 0.002
        r.move_linear(target_pose=[pre])
        r.move_circular(**circular_prop)
        if not dry_run:
            controller.arc_off()
        r.move_pose(approach)
        logger.info("CircularWeld done")


# --------------------------------------------------------------------------- #
# 3. 多层多道焊 (Multi-pass Weld)
# --------------------------------------------------------------------------- #
class MultiPassWeld(WeldingProcess):
    """
    多层多道焊：第一道打底，后续每道偏移 layer_offset_z。
    每道都执行一次 LinearWeaveWeld。
    """
    name = "MultiPassWeld"

    def __init__(self, start_pose, end_pose, passes: int = 3,
                 layer_offset_z: float = 0.002,
                 motion: Optional[MotionProfile] = None) -> None:
        super().__init__(motion or MotionProfile(speed=0.008))
        self.start_pose = list(start_pose)
        self.end_pose = list(end_pose)
        self.passes = max(1, int(passes))
        self.layer_offset_z = layer_offset_z

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        for k in range(self.passes):
            offset = self.layer_offset_z * k
            s = copy.deepcopy(self.start_pose)
            e = copy.deepcopy(self.end_pose)
            s[2] += offset
            e[2] += offset
            logger.info("MultiPassWeld: pass %d/%d (z+%.3f)",
                        k + 1, self.passes, offset)
            # 每道之间增加 interpass temperature 等待
            if k > 0 and not dry_run:
                time.sleep(2.0)
            layer = LinearWeaveWeld(
                s, e, self.motion, WeaveParameters(),
                controller.arc, None,
            )
            layer.execute(controller, dry_run=dry_run)


# --------------------------------------------------------------------------- #
# 4. 点焊 (Spot Weld)
# --------------------------------------------------------------------------- #
class SpotWeld(WeldingProcess):
    """
    电阻点焊 / 螺柱焊：在单点上施加压力+电流，停留 dwell 秒。
    """
    name = "SpotWeld"

    def __init__(self, pose: List[float], dwell: float = 1.0) -> None:
        super().__init__(MotionProfile(speed=0.02))
        self.pose = list(pose)
        self.dwell = float(dwell)

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r
        approach = copy.deepcopy(self.pose)
        approach[2] += 0.05
        r.move_pose(approach)
        # 接近
        r.move_linear(target_pose=[self.pose])

        if not dry_run:
            # 1) 触发电极加压
            try:
                r.io("set", io_name="DO_SPOT_PRESS", target_value=True)
            except Exception:
                pass
            time.sleep(0.2)  # 等电极加压稳
            # 2) 通电焊接
            try:
                r.io("set", io_name="DO_SPOT_WELD", target_value=True)
            except Exception:
                pass
            # 3) 维持 dwell
            time.sleep(self.dwell)
            # 4) 关焊接
            try:
                r.io("set", io_name="DO_SPOT_WELD", target_value=False)
                time.sleep(0.1)
                r.io("set", io_name="DO_SPOT_PRESS", target_value=False)
            except Exception:
                pass
        r.move_pose(approach)
        logger.info("SpotWeld done at %s, dwell=%.2fs", self.pose, self.dwell)


# --------------------------------------------------------------------------- #
# 5. 激光焊 (Laser Welding)
# --------------------------------------------------------------------------- #
class LaserWeld(WeldingProcess):
    """
    激光焊接：运动 + 连续激光输出 (通过模拟量/IO 触发激光器)。
    提供摆动以增加熔池宽度。

    参数:
        laser_power:        激光功率 (W), 如 2000W
        laser_power_scaling: 模拟量标度系数。
                             模拟量输出 AO_LASER_POWER 的输出值 =
                             laser_power * laser_power_scaling.
                             常见配置:
                             - 0-10V 系统, 10000W 满量程 → 0.001
                             - 4-20mA 系统, 5000W 满量程  → 0.004
        scan_freq:          振镜扫描频率 (Hz)
    """
    name = "LaserWeld"

    def __init__(self, start_pose, end_pose,
                 laser_power: float = 2000.0,
                 laser_power_scaling: float = 0.001,
                 scan_freq: float = 100.0,
                 motion: Optional[MotionProfile] = None) -> None:
        super().__init__(motion or MotionProfile(speed=0.02))
        self.start_pose = list(start_pose)
        self.end_pose = list(end_pose)
        self.laser_power = laser_power           # W
        self.laser_power_scaling = laser_power_scaling
        self.scan_freq = scan_freq

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r
        approach = copy.deepcopy(self.start_pose)
        approach[2] += 0.05
        r.move_pose(approach)

        # 1) 打开保护气
        try:
            r.io("set", io_name="DO_LASER_GAS", target_value=True)
        except Exception:
            pass
        time.sleep(0.5)

        # 2) 设置激光功率 (模拟量输出, 按标度系数转换)
        if not dry_run:
            analog_val = self.laser_power * self.laser_power_scaling
            try:
                r.io("set", io_name="AO_LASER_POWER",
                     target_value=analog_val)
            except Exception:
                logger.debug("Analog output not configured for laser power")

        # 3) 打开激光
        if not dry_run:
            try:
                r.io("set", io_name="DO_LASER_ON", target_value=True)
            except Exception:
                pass

        # 4) 运动焊接
        linear_prop = {
            "speed": self.motion.speed,
            "acceleration": self.motion.acceleration,
            "jerk": self.motion.jerk,
            "blend_radius": self.motion.blend_radius,
            "blending_mode": "static",
            "enable_blending": True,
            "target_pose": [self.start_pose, self.end_pose],
            "current_joint_angles": r.get_current_joint_angles(),
        }
        feas, _ = r.check_linear_feasibility(**linear_prop)
        if not feas:
            raise RuntimeError("Laser weld not feasible")
        pre = copy.deepcopy(self.start_pose)
        pre[2] += 0.001
        r.move_linear(target_pose=[pre])
        r.move_linear(**linear_prop)

        # 5) 关激光
        if not dry_run:
            try:
                r.io("set", io_name="DO_LASER_ON", target_value=False)
                r.io("set", io_name="AO_LASER_POWER", target_value=0.0)
            except Exception:
                pass
        # 6) 滞后停气
        time.sleep(1.0)
        try:
            r.io("set", io_name="DO_LASER_GAS", target_value=False)
        except Exception:
            pass
        r.move_pose(approach)
        logger.info("LaserWeld done, power=%.0fW", self.laser_power)


# --------------------------------------------------------------------------- #
# 6. 协作焊缝跟踪 (Cobot Seam Tracking via Force Control)
# --------------------------------------------------------------------------- #
class CobotSeamTrack(WeldingProcess):
    """
    基于力控的焊缝跟踪：
    - 在 Z 方向施加一个期望接触力 (e.g. 5N)
    - 控制器自动沿 X/Y 跟随工件微小偏差
    - 用于与人协作或工件有变形的场合
    """
    name = "CobotSeamTrack"

    def __init__(self, poses: List[List[float]],
                 seam_track: SeamTrackParameters,
                 motion: MotionProfile) -> None:
        super().__init__(motion)
        if len(poses) < 2:
            raise ValueError("CobotSeamTrack needs >= 2 poses")
        self.poses = poses
        self.seam_track = seam_track

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r

        # 1) 准备 servo 接口 (力控需要)
        if not dry_run:
            try:
                r.activate_servo_interface("torque")
            except Exception as e:
                logger.debug("servo interface already active: %s", e)

        # 2) 接近起弧点
        approach = copy.deepcopy(self.poses[0])
        approach[2] += 0.05
        r.move_pose(approach)

        # 3) 起弧
        if not dry_run:
            controller.arc_on()

        # 4) 用 move_composite 串联各段，每段开启力控
        # Neurapy 的 force/position 混合模式: controller_parameters
        commands = []
        for i in range(len(self.poses) - 1):
            seg = {
                "linear": {
                    "blend_radius": 0.002,
                    "target_pose": [self.poses[i], self.poses[i + 1]],
                }
            }
            commands.append(seg)

        composite_prop = {
            "speed": self.motion.speed,
            "acceleration": self.motion.acceleration,
            "jerk": self.motion.jerk,
            "blending_mode": "static",
            "blend_radius": 0.002,
            "commands": commands,
            "controller_parameters": {
                "control_mode": self.seam_track.control_mode,
                "force_vector": self.seam_track.force_vector,
                "torque_vector": self.seam_track.torque_vector,
            },
            "current_joint_angles": r.get_current_joint_angles(),
        }
        feas, _ = r.check_composite_feasibility(**composite_prop)
        if not feas:
            raise RuntimeError("Cobot seam track not feasible")
        r.move_composite(**composite_prop)
        r.wait_motion_finished()

        if not dry_run:
            controller.arc_off()

        try:
            r.deactivate_servo_interface()
        except Exception:
            pass
        r.move_pose(approach)
        logger.info("CobotSeamTrack done")


# --------------------------------------------------------------------------- #
# 7. 断续焊 / 段焊 (Stitch Weld)
# --------------------------------------------------------------------------- #
class StitchWeld(WeldingProcess):
    """
    段焊 / 断续焊：焊一段 → 停一段 → 焊一段，常用于薄板防变形。
    poses 给出整条路径上的若干关键点；
    stitch_length / gap_length 决定 焊接段 / 空走段 长度。
    """
    name = "StitchWeld"

    def __init__(self, poses: List[List[float]],
                 stitch_length: float = 0.02, gap_length: float = 0.01,
                 motion: Optional[MotionProfile] = None) -> None:
        super().__init__(motion or MotionProfile(speed=0.01))
        if stitch_length <= 0 or gap_length < 0:
            raise ValueError(
                f"stitch_length({stitch_length}) must be >0, "
                f"gap_length({gap_length}) must be >=0"
            )
        self.poses = poses
        self.stitch_length = stitch_length
        self.gap_length = gap_length

    def execute(self, controller: WeldingController, *, dry_run: bool = False) -> None:
        r = controller.r

        # 离散化整条路径
        all_points: List[List[float]] = [list(self.poses[0])]
        for i in range(len(self.poses) - 1):
            seg = interpolate_linear(self.poses[i], self.poses[i + 1], step=0.002)
            all_points.extend(seg[1:])

        # 沿路径累积距离
        distances = cumdist(all_points)
        total_len = distances[-1]
        if total_len < 1e-6:
            return

        # 起点
        approach = copy.deepcopy(self.poses[0])
        approach[2] += 0.05
        r.move_pose(approach)

        # 逐段执行
        cursor = 0.0
        while cursor < total_len - 1e-6:
            stitch_end = min(cursor + self.stitch_length, total_len)
            gap_end = min(stitch_end + self.gap_length, total_len)

            # 切到 stitch 起点
            start_pt = point_at(all_points, distances, cursor)
            r.move_linear(target_pose=[start_pt])

            if not dry_run:
                controller.arc_on()

            end_pt = point_at(all_points, distances, stitch_end)
            stitch_prop = {
                "speed": self.motion.speed,
                "acceleration": self.motion.acceleration,
                "jerk": self.motion.jerk,
                "blend_radius": 0.0,
                "enable_blending": False,
                "target_pose": [end_pt],
                "current_joint_angles": r.get_current_joint_angles(),
            }
            r.move_linear(**stitch_prop)

            if not dry_run:
                controller.arc_off()

            # gap 段空走
            if gap_end < total_len:
                gap_pt = point_at(all_points, distances, gap_end)
                r.move_linear(target_pose=[gap_pt], speed=0.05)

            cursor = gap_end

        r.move_pose(approach)
        logger.info("StitchWeld done: %d stitches", int(total_len / (self.stitch_length + self.gap_length)))