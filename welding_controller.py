"""
Welding Controller - 焊接控制器
================================

封装 Neurapy Robot 类，提供：
  - 起弧 / 收弧 (通过 Digital IO 触发焊机)
  - 焊枪 / 工具切换
  - 工艺执行入口 (WeldingController.run)
  - 焊接过程中的安全/碰撞监控
  - 与焊机 / PLC 的数字 IO 通讯

使用前提：
  1) Robot 已上电并切到 automatic_mode
  2) 已配置 IO (activate_servo_interface / IO YAML)
  3) 焊枪工具已通过 set_tool() 加载
"""
from __future__ import annotations

import time
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Optional, List, Callable, Any

from neurapy.robot import Robot

from .parameters import ArcParameters, SeamTrackParameters, WeaveParameters

logger = logging.getLogger(__name__)


class WeldingMode(str, Enum):
    MIG_MAG = "mig_mag"
    TIG     = "tig"
    PLASMA  = "plasma"
    SPOT    = "spot"
    LASER   = "laser"
    STUD    = "stud"


@dataclass
class SafetyLimits:
    """焊接安全监控限制 (相对 TCP)."""
    max_force_xy: float = 50.0    # N
    max_force_z:  float = 80.0    # N
    max_torque:   float = 10.0    # Nm
    tcp_speed_max: float = 0.05   # m/s (50 mm/s)
    enable_collision: bool = True


class WeldingController:
    """High level welding controller wrapping the Neurapy Robot."""

    def __init__(
        self,
        robot: Robot,
        weld_tool: str = "welding_torch",
        mode: WeldingMode = WeldingMode.MIG_MAG,
        arc: Optional[ArcParameters] = None,
        safety: Optional[SafetyLimits] = None,
    ) -> None:
        self.r = robot
        self.weld_tool = weld_tool
        self.mode = mode
        self.arc = arc or ArcParameters()
        self.safety = safety or SafetyLimits()

        # 内部状态
        self._arc_on = False
        self._welding_active = False
        self._errors_before: List[Any] = []

        # 配置安全
        if self.safety.enable_collision:
            try:
                self.r.enable_collision_detection()
            except Exception as e:
                logger.warning("Collision detection enable failed: %s", e)

    # ------------------------------------------------------------------
    # 工具准备
    # ------------------------------------------------------------------
    def setup(self) -> None:
        """上电、切换模式、加载焊枪工具."""
        if not self.r.is_robot_powered_on():
            self.r.power_on()
            time.sleep(2.0)

        if self.r.is_robot_in_teach_mode():
            self.r.switch_to_automatic_mode()
            time.sleep(1.0)

        self.r.set_tool(self.weld_tool)
        logger.info("Welding controller ready. Mode=%s Tool=%s",
                    self.mode, self.weld_tool)

    # ------------------------------------------------------------------
    # 起弧 / 收弧
    # ------------------------------------------------------------------
    def arc_on(self) -> None:
        """Trigger welding power source: gas pre-flow, arc ignite."""
        if self._arc_on:
            return
        logger.info("Arc ON")
        # 1) 提前送气
        try:
            self.r.io("set", io_name=self.arc.gas_valve_digital_output,
                      target_value=True)
        except Exception:
            logger.debug("Gas valve DO not configured")
        time.sleep(self.arc.pre_flow_time)

        # 2) 起弧信号 (焊机收到后建立电弧)
        try:
            self.r.io("set", io_name=self.arc.arc_on_digital_output,
                      target_value=True)
            self.r.io("set", io_name=self.arc.wire_feed_digital_output,
                      target_value=True)
        except Exception:
            logger.debug("Arc-on DO not configured")

        # 3) 等待电弧建立反馈
        if self.arc.arc_ok_digital_input:
            try:
                ok = self.r.wait_for_digital_input(
                    io_name=self.arc.arc_ok_digital_input,
                    timeout=2.0,
                )
                if not ok:
                    raise RuntimeError("Arc ignition timeout")
            except Exception as e:
                logger.debug("No arc-ok feedback: %s", e)

        time.sleep(self.arc.arc_on_delay)
        self._arc_on = True
        self._welding_active = True

    def arc_off(self) -> None:
        """Crater fill, stop wire, stop arc, post-flow gas."""
        if not self._arc_on:
            return
        logger.info("Arc OFF")
        # 1) 弧坑填充
        time.sleep(self.arc.crater_fill_time)

        # 2) 停止送丝 & 起弧
        try:
            self.r.io("set", io_name=self.arc.wire_feed_digital_output,
                      target_value=False)
            self.r.io("set", io_name=self.arc.arc_on_digital_output,
                      target_value=False)
        except Exception:
            pass

        # 3) 滞后停气
        time.sleep(self.arc.post_flow_time)
        try:
            self.r.io("set", io_name=self.arc.gas_valve_digital_output,
                      target_value=False)
        except Exception:
            pass

        self._arc_on = False
        self._welding_active = False

    # ------------------------------------------------------------------
    # 速度监控 (焊接过程中)
    # ------------------------------------------------------------------
    def _check_tcp_speed(self) -> None:
        try:
            v = self.r.get_tcp_speed()
            if v > self.safety.tcp_speed_max:
                logger.warning("TCP speed %.4f m/s exceeds limit %.4f m/s",
                               v, self.safety.tcp_speed_max)
        except Exception:
            pass

    def _check_force(self) -> bool:
        try:
            w = self.r.get_tcp_wrench()
            # w: [Fx, Fy, Fz, Mx, My, Mz]
            fxy = (w[0] ** 2 + w[1] ** 2) ** 0.5
            if fxy > self.safety.max_force_xy or abs(w[2]) > self.safety.max_force_z:
                logger.error("Force limit exceeded: Fxy=%.2f Fz=%.2f", fxy, w[2])
                self.r.pause()
                return False
        except Exception:
            pass
        return True

    # ------------------------------------------------------------------
    # 工艺入口
    # ------------------------------------------------------------------
    def run(self, process: "WeldingProcess", blocking: bool = True) -> None:
        """
        Execute a welding process. Each process describes its motion plan
        and decides when to call arc_on() / arc_off() around it.
        """
        logger.info("Running process: %s", type(process).__name__)
        self._errors_before = self.r.get_errors()
        try:
            process.execute(self)
            if blocking:
                self.r.wait_motion_finished()
        except Exception as e:
            logger.exception("Welding process failed: %s", e)
            self.arc_off()
            raise
        finally:
            self._check_post_conditions()

    def dry_run(self, process: "WeldingProcess") -> None:
        """走空路径 (不起弧) — 用于示教验证."""
        logger.info("Dry run (no arc): %s", type(process).__name__)
        process.execute(self, dry_run=True)

    def _check_post_conditions(self) -> None:
        # 任何新增的 error 都打印
        try:
            errs_after = self.r.get_errors()
            new_errs = [e for e in errs_after if e not in self._errors_before]
            if new_errs:
                logger.error("New errors during welding: %s", new_errs)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 上下文管理
    # ------------------------------------------------------------------
    def __enter__(self) -> "WeldingController":
        self.setup()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._arc_on:
            self.arc_off()
