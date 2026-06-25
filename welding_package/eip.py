"""
EIP (Ethernet/IP) 通讯适配层
================================

工业焊接的常见实践：
  焊机 (Fronius / Lincoln / Miller / Yaskawa Motoman) 通过
  Ethernet/IP 工业总线与机器人控制器交换数据。本模块封装了
  机器人控制器作为 CIP Scanner 主动读写焊机 Adapter 寄存器的模式。

CIP 角色 (ODVA Volume 3, 与本模块代码实现一致):
  ┌────────────────────────────────────────────────────────────┐
  │  Scanner  (机器人控制器)                                  │
  │      - 主动发起 CIP 连接与读写请求                         │
  │      - 通常是 PLC / 机器人控制器 / SCADA                  │
  │                                                            │
  │  Adapter  (焊机)                                           │
  │      - 被动接受连接, 暴露 Assembly / Register 对象        │
  │      - 通常是焊机/远程 IO/变频器/传感器                   │
  └────────────────────────────────────────────────────────────┘

数据流方向 (与 CIP 角色正交, 由 Neurapy API 的 T2O / O2T 参数决定):
  ┌────────────────────────────────────────────────────────────┐
  │  T2O  (Target-to-Originator, Adapter -> Scanner)           │
  │      焊机 发送 状态 给 机器人 (Scanner 从 Adapter 读寄存器)│
  │      API: get_register_values(tag, first, last, T2O=True)   │
  │                                                            │
  │  O2T  (Originator-to-Target, Scanner -> Adapter)           │
  │      机器人 发送 指令 给 焊机 (Scanner 写 Adapter 寄存器) │
  │      API: set_dblout_register(first, last, *values)         │
  │           set_intout_register(first, last, *values)         │
  │           get_register_values(tag, first, last, T2O=False)  │
  └────────────────────────────────────────────────────────────┘

注: Neurapy 的 get_register_values(tag, ..., T2O=True/False) 中
   T2O=True 表示从 Adapter (Target) 读数据发往 Scanner (Originator),
   即焊机 → 机器人方向. 代码实现已遵循 CIP 规范.

两种使用模式:
  1) **轮询模式** - 在 Python 端显式调用 write_job / read_status
     适合: 简单集成, 不依赖位置触发

  2) **动作触发模式** - 通过 set_external_device_actions 把
     "到位姿 X 时调用焊机函数"注册到控制器内部
     适合: 高速焊接, 减少 Python 端循环延迟 (PDF Ch.5 set_external_device_actions)

本模块同时支持两种模式.
"""
from __future__ import annotations

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Any, Tuple, Sequence

from neurapy.robot import Robot

logger = logging.getLogger(__name__)


class EIPJobMode(str, Enum):
    """典型焊机的 Job 模式 (按 Fronius / Lincoln 约定)."""
    JOB_SELECT     = "job_select"          # 选择焊机已存的 job 号
    DYNAMIC_PARAM  = "dynamic_param"       # 动态写入焊接参数 (current/wire feed)
    SYNERGIC       = "synergic"            # 协同模式 (材料+厚度自动算参数)


@dataclass
class EIPWeldingTagMap:
    """
    EIP tag 名称与寄存器索引映射 - 描述你的 EIP YAML/EDS 配置里各 tag 含义.

    每个字段 (dout_*, din_*) 包含:
      - 字段名末尾的 1-2 位数字 = 该 tag 在 DBLOUT 寄存器组中的 0-based 索引
        (EIP 寄存器从 0 开始, 一个 REAL/INT 各占 1 元素)
      - 这种"按字段约定索引"的方式让用户只需看 EDS 即可知道写到哪,
        不必再单独配置 index.

    数据流:
      - 字段名以 dout_ 开头 = Scanner 写 Adapter (O2T, 机器人 → 焊机)
      - 字段名以 din_ 开头  = Scanner 读 Adapter (T2O, 焊机 → 机器人)

    默认布局 (Fronius 风格) — 共 5 个连续 DBLOUT 元素:
        idx 0: dout_job        (INT, O2T)
        idx 1: dout_current    (REAL, O2T)
        idx 2: dout_voltage    (REAL, O2T)
        idx 3: dout_wire_feed  (REAL, O2T)
        idx 4: dout_gas_flow   (REAL, O2T)

    常见 Fronius 配置示例:

      DOUT_weld_job        (INT, Scanner→Adapter)  - 焊机 job 号 (1-256)
      DOUT_weld_current    (REAL, Scanner→Adapter) - 焊接电流 (A)
      DOUT_weld_voltage    (REAL, Scanner→Adapter) - 焊接电压 (V)
      DOUT_wire_feed       (REAL, Scanner→Adapter) - 送丝速度 (m/min)
      DOUT_gas_flow        (REAL, Scanner→Adapter) - 保护气流量 (L/min)
      DOUT_arc_on          (BOOL, Scanner→Adapter) - 起弧指令
      DOUT_wire_feed_on    (BOOL, Scanner→Adapter) - 送丝使能
      DOUT_gas_on          (BOOL, Scanner→Adapter) - 保护气使能

      DINT_weld_status     (INT, Adapter→Scanner) - 焊机状态字 (bit0=ready, bit1=arc_ok, ...)
      DINT_error_code      (INT, Adapter→Scanner) - 错误码
      DINT_current_actual  (REAL, Adapter→Scanner) - 实际电流
      DINT_voltage_actual  (REAL, Adapter→Scanner) - 实际电压

    用户应根据自己的 EDS 文件修改本数据类. 字段名末尾索引 (e.g. `_0`, `_1`)
    决定了写入位置; 修改时**保持 EDS 一致**即可.
    """
    # ---- O2T (Scanner → Adapter, 机器人写焊机) ----
    # DBLOUT 寄存器: INT/REAL 共享同一个 0-based 索引空间
    dout_job_0: str = "DOUT_weld_job"          # idx 0, INT
    dout_current_1: str = "DOUT_weld_current"  # idx 1, REAL
    dout_voltage_2: str = "DOUT_weld_voltage"  # idx 2, REAL
    dout_wire_feed_3: str = "DOUT_wire_feed"   # idx 3, REAL
    dout_gas_flow_4: str = "DOUT_gas_flow"     # idx 4, REAL

    # BOOL 单独使用 INTOUT 位掩码 (EIP 惯例)
    dout_arc_on: str = "DOUT_arc_on"
    dout_wire_feed_on: str = "DOUT_wire_feed_on"
    dout_gas_on: str = "DOUT_gas_on"

    # ---- T2O (Adapter → Scanner, 机器人读焊机) ----
    din_status: str = "DINT_weld_status"           # INT
    din_error: str = "DINT_error_code"             # INT
    din_current_actual: str = "DINT_current_actual"  # REAL
    din_voltage_actual: str = "DINT_voltage_actual"  # REAL

    def write_index(self, field_name: str) -> int:
        """从字段名 (如 'dout_current_1') 解析出 DBLOUT 索引."""
        parts = field_name.rsplit("_", 1)
        if len(parts) == 2 and parts[1].isdigit():
            return int(parts[1])
        # BOOL/控制位没有索引 (使用 INTOUT 位寻址)
        return -1

    def dblout_field_for(self, kind: str) -> str:
        """根据语义类型返回字段名 (kind ∈ {job, current, voltage, wire_feed, gas_flow})."""
        return {
            "job":       self.dout_job_0,
            "current":   self.dout_current_1,
            "voltage":   self.dout_voltage_2,
            "wire_feed": self.dout_wire_feed_3,
            "gas_flow":  self.dout_gas_flow_4,
        }[kind]


@dataclass
class EIPWeldingConfig:
    """EIP 通讯总配置. 机器人作为 Scanner 通过 EIP 读/写焊机 Adapter."""
    yaml_path: str = ""                       # activate_ethernetIP_interface 的 YAML
    tags: EIPWeldingTagMap = field(default_factory=EIPWeldingTagMap)
    scan_timeout: float = 2.0                 # 等待焊机响应的最大秒数
    job_mode: EIPJobMode = EIPJobMode.JOB_SELECT


# ---------------------------------------------------------------------------- #
# 状态字 bit 布局 (Fronius 风格, 兼容大多数焊机)
# ---------------------------------------------------------------------------- #
class WeldStatusBit:
    READY         = 1 << 0   # bit0: 焊机就绪
    ARC_OK        = 1 << 1   # bit1: 电弧已建立
    WIRE_FEEDING  = 1 << 2   # bit2: 正在送丝
    GAS_FLOWING   = 1 << 3   # bit3: 保护气在流
    ERROR         = 1 << 7   # bit7: 焊机报错


# ---------------------------------------------------------------------------- #
# 主类
# ---------------------------------------------------------------------------- #
class EIPWeldingLink:
    """
    EIP 焊接通讯链路 (机器人 Scanner → 焊机 Adapter).

    用法:
        link = EIPWeldingLink(robot, EIPWeldingConfig(
            yaml_path="/etc/neurapy/eip_welder.yaml",
            tags=EIPWeldingTagMap(...),
        ))
        link.connect()
        link.write_job(job=12, current=180, voltage=22, wire_feed=5.0)
        link.arc_on(wait_ready=True)
        ... 焊接 ...
        link.arc_off()
        link.disconnect()
    """

    def __init__(self, robot: Robot, config: Optional[EIPWeldingConfig] = None) -> None:
        self.r = robot
        self.cfg = config or EIPWeldingConfig()
        self._connected = False
        self._arc_on = False
        self._status_word = 0
        self._status_ts = 0.0   # 状态字缓存时间戳

    # ------------------------------------------------------------------ #
    # 连接管理
    # ------------------------------------------------------------------ #
    def connect(self) -> bool:
        """激活 EIP 接口 + 验证通讯 (Scanner 主动建立连接)."""
        try:
            ok = self.r.activate_ethernetIP_interface(self.cfg.yaml_path)
        except Exception as e:
            logger.error("EIP activate failed: %s", e)
            return False
        if not ok:
            logger.error("EIP interface activation returned False")
            return False
        self._connected = True
        logger.info("EIP Scanner connected to Adapter via %s",
                    self.cfg.yaml_path or "default")
        return True

    def disconnect(self) -> None:
        """停用 EIP 接口."""
        if not self._connected:
            return
        try:
            self.r.deactivate_ethernetIP_interface()
        except Exception as e:
            logger.debug("EIP deactivation error: %s", e)
        self._connected = False
        logger.info("EIP Scanner disconnected")

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------ #
    # 底层读写 (Scanner 写 Adapter, O2T) - 每个 tag 字段名自带 _N 索引
    # ------------------------------------------------------------------ #
    def _set_real_tag(self, tag: str, value: float) -> bool:
        """Scanner 向 Adapter 的 REAL 标签 (DBLOUT 寄存器) 写 1 个元素.

        用 tag 反查字段名以得到 index; 若 tag 不在已知字段中,
        退化为使用 tag 名末尾 _N 形式.
        """
        idx = self._resolve_dblout_index(tag)
        if idx < 0:
            logger.debug("EIP _set_real_tag: %s has no DBLOUT index", tag)
            return False
        try:
            n = self.r.set_dblout_register(idx, idx, float(value))
            return n == 1
        except Exception as e:
            logger.debug("EIP set %s=%s failed: %s", tag, value, e)
            return False

    def _set_int_tag(self, tag: str, value: int) -> bool:
        """Scanner 向 Adapter 的 INT 标签 (INTOUT 寄存器) 写 1 个元素.

        修复: 之前误用 set_dblout_register 写 REAL 格式,
        导致 SINT/INT 字段 (如焊机 job 号) 写入错误字节.
        """
        idx = self._resolve_dblout_index(tag)
        if idx < 0:
            logger.debug("EIP _set_int_tag: %s has no DBLOUT index", tag)
            return False
        try:
            # INTOUT 与 DBLOUT 在 EIP 里通常共享同一索引空间,
            # Neurapy 提供独立 API 以确保按 INT 序列化
            n = self.r.set_intout_register(idx, idx, int(value))
            return n == 1
        except Exception as e:
            logger.debug("EIP set %s=%s failed: %s", tag, value, e)
            return False

    def _set_bool_tag(self, tag: str, value: bool) -> bool:
        """Scanner 向 Adapter 的 BOOL 标签 (INTOUT 位) 写 0/1."""
        idx = self._resolve_dblout_index(tag)
        if idx < 0:
            # BOOL 字段通常没有 _N 后缀, 用 0 索引 (用户应通过
            # EIPWeldingTagMap 的扩展机制提供 BOOL 索引)
            idx = 0
        try:
            n = self.r.set_intout_register(idx, idx, 1 if value else 0)
            return n == 1
        except Exception as e:
            logger.debug("EIP set %s=%s failed: %s", tag, value, e)
            return False

    def _resolve_dblout_index(self, tag: str) -> int:
        """从已知字段反查 tag 的 DBLOUT 索引.

        查找顺序:
          1) 与 EIPWeldingTagMap 字段值完全匹配 → 用字段名末尾 _N
          2) tag 名本身以 _N 结尾 → 解析 N
          3) 否则返回 -1 (表示无索引, 调用方应放弃)
        """
        # 1) 反查已知字段
        t = self.cfg.tags
        for fname in ("dout_job_0", "dout_current_1", "dout_voltage_2",
                      "dout_wire_feed_3", "dout_gas_flow_4"):
            if getattr(t, fname) == tag:
                return t.write_index(fname)
        # 2) 解析 tag 自身
        idx = self.cfg.tags.write_index(tag)
        if idx >= 0:
            return idx
        # 3) tag 形如 "DOUT_weld_job" 但我们的字段也形如这个, 上面已覆盖
        return -1

    def _read_int_tag(self, tag: str, first: int, last: int, t2o: bool = True) -> List[int]:
        """Scanner 从 Adapter 读 INT/REAL 寄存器组 (T2O=True 方向).

        内部封装以让 mock 测试能统一 hook 读操作.
        """
        try:
            return list(self.r.get_register_values(tag, first, last, T2O=t2o))
        except Exception as e:
            logger.debug("EIP read %s failed: %s", tag, e)
            return []

    # ------------------------------------------------------------------ #
    # 高级: Scanner 写一个 job
    # ------------------------------------------------------------------ #
    def write_job(
        self,
        job: Optional[int] = None,
        current: Optional[float] = None,
        voltage: Optional[float] = None,
        wire_feed: Optional[float] = None,
        gas_flow: Optional[float] = None,
    ) -> bool:
        """
        Scanner 写入一组焊接参数到焊机 Adapter.
        写顺序: gas_flow -> wire_feed -> voltage -> current -> job
        (焊机按此顺序应用可避免瞬时过流)

        每个参数的目标寄存器由 EIPWeldingTagMap 中对应字段的
        _N 后缀决定 (e.g. dout_current_1 -> DBLOUT[1]),
        因此与字段声明顺序解耦.
        """
        if not self._connected:
            logger.error("EIP not connected")
            return False
        ok = True
        if gas_flow is not None:
            ok &= self._set_real_tag(self.cfg.tags.dout_gas_flow_4, gas_flow)
        if wire_feed is not None:
            ok &= self._set_real_tag(self.cfg.tags.dout_wire_feed_3, wire_feed)
        if voltage is not None:
            ok &= self._set_real_tag(self.cfg.tags.dout_voltage_2, voltage)
        if current is not None:
            ok &= self._set_real_tag(self.cfg.tags.dout_current_1, current)
        if job is not None:
            ok &= self._set_int_tag(self.cfg.tags.dout_job_0, job)
        return ok

    # ------------------------------------------------------------------ #
    # 状态读取 (Scanner 从 Adapter 读, T2O) - 带 TTL 缓存避免重复 EIP 读
    # ------------------------------------------------------------------ #
    _STATUS_TTL_S = 0.05   # 50ms 缓存, 与 EIP 典型 RPI 匹配

    def _status_now(self) -> int:
        """读最新状态字 (优先用缓存, 过期则重读)."""
        if (time.time() - self._status_ts) < self._STATUS_TTL_S:
            return self._status_word
        vals = self._read_int_tag(
            self.cfg.tags.din_status, 1, 1, t2o=True,
        )
        if vals:
            self._status_word = int(vals[0])
        self._status_ts = time.time()
        return self._status_word

    def read_status(self) -> int:
        """Scanner 读焊机 Adapter 状态字 (16-bit). 立即返回 (走缓存)."""
        return self._status_now()

    def is_ready(self) -> bool:
        return bool(self._status_now() & WeldStatusBit.READY)

    def is_arc_ok(self) -> bool:
        return bool(self._status_now() & WeldStatusBit.ARC_OK)

    def is_in_error(self) -> bool:
        return bool(self._status_now() & WeldStatusBit.ERROR)

    def read_actual_current(self) -> Optional[float]:
        vals = self._read_int_tag(
            self.cfg.tags.din_current_actual, 1, 1, t2o=True,
        )
        return float(vals[0]) if vals else None

    def read_actual_voltage(self) -> Optional[float]:
        vals = self._read_int_tag(
            self.cfg.tags.din_voltage_actual, 1, 1, t2o=True,
        )
        return float(vals[0]) if vals else None

    def read_error_code(self) -> int:
        vals = self._read_int_tag(
            self.cfg.tags.din_error, 1, 1, t2o=True,
        )
        return int(vals[0]) if vals else 0

    # ------------------------------------------------------------------ #
    # 起弧 / 收弧
    # ------------------------------------------------------------------ #
    def arc_on(self, *, wait_ready: bool = True, timeout: Optional[float] = None) -> bool:
        """
        Scanner 通过 EIP 触发焊机起弧:
          1) 开保护气 (DOUT_gas_on = True)
          2) 等待 pre-flow
          3) 开送丝 (DOUT_wire_feed_on = True)
          4) 起弧 (DOUT_arc_on = True)
          5) 轮询焊机状态字等待 ARC_OK
        """
        if self._arc_on:
            return True
        t = self.cfg.tags
        timeout = timeout or self.cfg.scan_timeout

        # 1) 保护气
        if not self._set_bool_tag(t.dout_gas_on, True):
            logger.error("Failed to open gas")
            return False

        # 2) pre-flow
        time.sleep(0.5)

        # 3) 送丝 + 起弧
        self._set_bool_tag(t.dout_wire_feed_on, True)
        if not self._set_bool_tag(t.dout_arc_on, True):
            logger.error("Failed to set arc_on")
            return False

        # 4) 等待 ARC_OK
        if wait_ready:
            t0 = time.time()
            while time.time() - t0 < timeout:
                if self.is_arc_ok():
                    logger.info("EIP arc established in %.2fs", time.time() - t0)
                    self._arc_on = True
                    return True
                time.sleep(0.05)
            logger.error("EIP arc ignition timeout (%.1fs)", timeout)
            self.arc_off()
            return False

        self._arc_on = True
        return True

    def arc_off(self, *, post_flow: float = 1.0) -> None:
        """收弧: 停弧 -> 停丝 -> 滞后停气."""
        if not self._arc_on:
            return
        t = self.cfg.tags
        # 1) 停弧
        self._set_bool_tag(t.dout_arc_on, False)
        time.sleep(0.3)
        # 2) 停丝
        self._set_bool_tag(t.dout_wire_feed_on, False)
        # 3) 滞后停气
        time.sleep(post_flow)
        self._set_bool_tag(t.dout_gas_on, False)
        self._arc_on = False
        logger.info("EIP arc off")

    # ------------------------------------------------------------------ #
    # 动作触发模式 (位置到位姿自动调用)
    # ------------------------------------------------------------------ #
    def register_pose_triggered_actions(
        self,
        device_json: str,
        trigger_poses: Sequence[Tuple[Any, str, Dict[str, Any]]],
    ) -> bool:
        """
        注册位置触发的焊机动作. 当机器人到达 trigger_pose 时,
        控制器自动调用 function_name(function_parameters).

        Example - 在每个示教点处切换焊接参数:
            link.register_pose_triggered_actions(
                device_json="welder.json",
                trigger_poses=[
                    (p0, "start_arc", {"current": 180, "voltage": 22}),
                    (p1, "adjust",    {"current": 200, "voltage": 24}),
                    (p2, "stop_arc",  {}),
                ],
            )

        See: PDF Ch.5 set_external_device_actions
        """
        actions = []
        for i, (pose, func, params) in enumerate(trigger_poses):
            actions.append({
                "trigger_pose": pose,
                "function_name": func,
                "function_parameters": params,
            })
        try:
            ok = self.r.set_external_device_actions(device_json, actions)
            logger.info("Registered %d pose-triggered EIP actions on %s",
                        len(actions), device_json)
            return bool(ok)
        except Exception as e:
            logger.error("Failed to register pose-triggered actions: %s", e)
            return False

    def execute_function(
        self,
        device_json: str,
        function_name: str,
        function_parameters: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        立即调用外部设备的命名函数 (不等位置触发).
        对应 PDF Ch.5 execute_external_device_function.
        """
        try:
            ok = self.r.execute_external_device_function(
                device_json,
                function_name,
                function_parameters or {},
            )
            return bool(ok)
        except Exception as e:
            logger.error("execute_external_device_function %s failed: %s",
                         function_name, e)
            return False

    def safety_stop(self, device_json: str) -> bool:
        """调用焊机的安全停止 (硬件级)."""
        try:
            ok = self.r.execute_external_device_safety_stop(device_json)
            return bool(ok)
        except Exception as e:
            logger.error("execute_external_device_safety_stop failed: %s", e)
            return False

    # ------------------------------------------------------------------ #
    # 上下文管理
    # ------------------------------------------------------------------ #
    def __enter__(self) -> "EIPWeldingLink":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._arc_on:
            self.arc_off()
        self.disconnect()


# ---------------------------------------------------------------------------- #
# 辅助: 把 EIP 接入现有焊接流程
# ---------------------------------------------------------------------------- #
class EIPBackedWeldingController:
    """
    把 EIP 通讯作为后端, 复刻 `WeldingController` 的核心接口.

    适用于:
      - 工厂所有焊机都已通过 EIP 接入
      - 不希望管理 Digital IO 线路
      - 需要集中式状态监控 (current/voltage 实时回读)
    """

    def __init__(self, robot: Robot, eip: EIPWeldingLink,
                 device_json: str = "welder.json") -> None:
        self.r = robot
        self.eip = eip
        self.device_json = device_json
        self._arc_on = False
        self._errors_before: List[Any] = []

    def setup(self) -> None:
        if not self.r.is_robot_powered_on():
            self.r.power_on()
            time.sleep(2.0)
        if self.r.is_robot_in_teach_mode():
            self.r.switch_to_automatic_mode()
            time.sleep(1.0)
        if not self.eip.is_connected():
            self.eip.connect()

    def arc_on(self, **kwargs) -> bool:
        if self._arc_on:
            return True
        ok = self.eip.arc_on(**kwargs)
        if ok:
            self._arc_on = True
        return ok

    def arc_off(self) -> None:
        self.eip.arc_off()
        self._arc_on = False

    def __enter__(self) -> "EIPBackedWeldingController":
        self.setup()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._arc_on:
            self.arc_off()
