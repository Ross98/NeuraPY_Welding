"""
焊接工艺参数 (Welding Process Parameters)
=========================================

数据类集合，封装各焊接方式的工艺参数。所有单位遵循国际标准 (SI)。
"""
from dataclasses import dataclass, field
from typing import Dict, Any


@dataclass
class WeaveParameters:
    """摆动焊参数 - Weaving motion parameters (sinusoidal or circular)."""
    pattern: str = "sine"             # "sine" / "triangle" / "circle"
    frequency: float = 1.0            # Hz
    amplitude_left: float = 0.002     # m, 横向摆幅
    amplitude_right: float = 0.002    # m
    dwell_time_left: float = 0.0      # s, 左停留时间
    dwell_time_right: float = 0.0     # s, 右停留时间
    elevation: float = 0.0            # rad, 绕速度矢量旋转
    azimuth: float = 0.0              # rad, 绕Z轴旋转
    radius: float = 0.0               # m, 圆形摆动半径 (仅 circle 模式)

    def to_api_dict(self) -> Dict[str, Any]:
        """转换为 Neurapy API weaving_parameter 字典格式."""
        d: Dict[str, Any] = {
            "pattern": self.pattern,
            "frequency": self.frequency,
            "elevation": self.elevation,
            "azimuth": self.azimuth,
            "amplitude_left": self.amplitude_left,
            "amplitude_right": self.amplitude_right,
            "dwell_time_left": self.dwell_time_left,
            "dwell_time_right": self.dwell_time_right,
        }
        if self.pattern == "circle":
            d["radius"] = self.radius
        return d


@dataclass
class MotionProfile:
    """运动参数 - 通用运动曲线."""
    speed: float = 0.01               # m/s, 焊接速度 (典型 5-30 mm/s)
    acceleration: float = 0.05         # m/s^2
    jerk: float = 100.0                # m/s^3
    rotation_speed: float = 0.5       # rad/s
    rotation_acceleration: float = 1.57
    rotation_jerk: float = 100.0
    blend_radius: float = 0.001       # m, 圆滑过渡半径
    blending_mode: str = "static"     # "static" / "dynamic"

    def to_api_dict(self) -> Dict[str, Any]:
        return {
            "speed": self.speed,
            "acceleration": self.acceleration,
            "jerk": self.jerk,
            "rotation_speed": self.rotation_speed,
            "rotation_acceleration": self.rotation_acceleration,
            "rotation_jerk": self.rotation_jerk,
            "blend_radius": self.blend_radius,
            "blending_mode": self.blending_mode,
            "enable_blending": True,
        }


@dataclass
class ArcParameters:
    """电弧/能量参数 - 与焊接电源通讯。"""
    voltage: float = 22.0              # V, 电压
    current: float = 180.0             # A, 电流
    wire_feed_speed: float = 5.0       # m/min, 送丝速度
    gas_flow: float = 15.0             # L/min, 保护气体流量
    arc_on_delay: float = 0.3          # s, 起弧前延时
    arc_off_delay: float = 0.5         # s, 收弧后延时
    crater_fill_time: float = 0.4      # s, 弧坑填充时间
    pre_flow_time: float = 0.5         # s, 提前送气
    post_flow_time: float = 1.0        # s, 滞后停气

    # 数字 IO 触发配置 (典型 Fronius / Lincoln 电源)
    arc_on_digital_output: str = "DO_1"        # 起弧信号
    wire_feed_digital_output: str = "DO_2"     # 送丝信号
    gas_valve_digital_output: str = "DO_3"     # 保护气阀
    arc_ok_digital_input: str = "DI_1"         # 起弧成功反馈
    current_ok_digital_input: str = "DI_2"     # 电流正常反馈


@dataclass
class SeamTrackParameters:
    """焊缝跟踪参数 - 接触式 / 激光 / 电弧传感."""
    enabled: bool = True
    control_mode: str = "hybrid"       # "position"/"joint_impedance"/"hybrid"/"admittance"
    force_vector: Dict[str, Dict[str, float]] = field(default_factory=lambda: {
        "Fx": {"is_active": False, "value": 0.0},
        "Fy": {"is_active": False, "value": 0.0},
        "Fz": {"is_active": True,  "value": 5.0},  # 接触力 5N
    })
    torque_vector: Dict[str, Dict[str, float]] = field(default_factory=lambda: {
        "Mx": {"is_active": False, "value": 0.0},
        "My": {"is_active": False, "value": 0.0},
        "Mz": {"is_active": False, "value": 0.0},
    })
    correction_gain: float = 0.5
    max_lateral_correction: float = 0.005   # m, 单次最大横向纠偏
