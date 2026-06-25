"""
Pytest tests for EIP (Ethernet/IP) welding adapter.
====================================================
"""
from __future__ import annotations

import time
import pytest

# conftest.py 注入 mock neurapy


# ===================================================================== #
# Fixture: 带 EIP 能力的 mock robot
# ===================================================================== #
# 从 conftest 导入基础 MockRobot, 然后扩展
import sys
from pathlib import Path as _Path
sys.path.insert(0, str(_Path(__file__).parent))
import conftest as _conftest
_BaseMockRobot = _conftest.MockRobot


class _EIPMockRobot(_BaseMockRobot):
    """扩展 MockRobot, 添加 EIP 寄存器/动作接口."""

    def __init__(self):
        super().__init__()
        # 模拟 EIP 寄存器空间
        self.dblout = {}        # O2T 写入 (index -> value)
        self.intout = {}
        self.t2o = {}           # T2O 读取 (tag -> values)
        self.connected = False
        self.actions = []
        # 默认状态: ready
        self.t2o["DINT_weld_status"] = [1]   # bit0=READY
        self.t2o["DINT_error_code"]   = [0]
        self.t2o["DINT_current_actual"] = [180.0]
        self.t2o["DINT_voltage_actual"] = [22.0]

    def _log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def activate_ethernetIP_interface(self, path):
        self._log("activate_ethernetIP_interface", path)
        self.connected = True
        return True

    def deactivate_ethernetIP_interface(self):
        self._log("deactivate_ethernetIP_interface")
        self.connected = False
        return True

    def set_dblout_register(self, first, last, *values):
        self._log("set_dblout_register", first, last, values)
        for i, v in enumerate(values):
            self.dblout[first + i] = v
        return len(values)

    def set_intout_register(self, first, last, *values):
        self._log("set_intout_register", first, last, values)
        for i, v in enumerate(values):
            self.intout[first + i] = v
        return len(values)

    def get_register_values(self, tag, first, last, T2O=False):
        self._log("get_register_values", tag, first, last, T2O)
        return list(self.t2o.get(tag, []))

    def set_external_device_actions(self, file_path, actions):
        self._log("set_external_device_actions", file_path, actions)
        self.actions = list(actions)
        return True

    def execute_external_device_function(self, file_path, fn, params):
        self._log("execute_external_device_function", file_path, fn, params)
        return True

    def execute_external_device_safety_stop(self, file_path):
        self._log("execute_external_device_safety_stop", file_path)
        return True


@pytest.fixture
def eip_robot():
    return _EIPMockRobot()


# ===================================================================== #
# TC-EIP-001: 导入与公开 API
# ===================================================================== #
def test_TC_EIP_001_imports():
    from welding_package import (
        EIPWeldingLink, EIPWeldingConfig, EIPWeldingTagMap,
        EIPJobMode, EIPBackedWeldingController, WeldStatusBit,
    )
    assert EIPJobMode.JOB_SELECT.value == "job_select"
    assert WeldStatusBit.ARC_OK == 0b10
    assert WeldStatusBit.ERROR == 0b10000000


# ===================================================================== #
# TC-EIP-002: connect / disconnect
# ===================================================================== #
def test_TC_EIP_002_connect_disconnect(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig(yaml_path="/tmp/eip.yaml"))
    assert link.is_connected() is False
    assert link.connect() is True
    assert eip_robot.connected is True
    assert link.is_connected() is True
    link.disconnect()
    assert eip_robot.connected is False


# ===================================================================== #
# TC-EIP-003: write_job 写入所有 5 个参数
# ===================================================================== #
def test_TC_EIP_003_write_job(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    ok = link.write_job(job=12, current=180.0, voltage=22.0,
                        wire_feed=5.0, gas_flow=15.0)
    assert ok is True
    # 4 个 REAL 参数走 set_dblout_register
    dout_calls = [c for c in eip_robot.calls if c[0] == "set_dblout_register"]
    assert len(dout_calls) == 4
    # 1 个 INT (job) 走 set_intout_register (修复: 之前误用 dblout)
    intout_calls = [c for c in eip_robot.calls if c[0] == "set_intout_register"]
    assert len(intout_calls) == 1
    # 索引验证: current -> idx 1, voltage -> idx 2, wire_feed -> idx 3, gas_flow -> idx 4
    indices = sorted(c[1][0] for c in dout_calls)
    assert indices == [1, 2, 3, 4]
    # job 写到 idx 0
    assert intout_calls[0][1][0] == 0
    assert intout_calls[0][1][2] == (12,)   # 写入值


# ===================================================================== #
# TC-EIP-004: arc_on 等待 ARC_OK
# ===================================================================== #
def test_TC_EIP_004_arc_on_waits_for_ready(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig(scan_timeout=1.0))
    link.connect()
    # 第 1 次 read 时返回 ready
    eip_robot.t2o["DINT_weld_status"] = [0b11]   # READY + ARC_OK
    ok = link.arc_on(wait_ready=True, timeout=0.5)
    assert ok is True
    # 验证 arc_on + gas_on 都置位
    # 在 dblout 中: 0 (gas), 1 (arc) 应该都是非零
    assert any(v > 0 for v in eip_robot.intout.values())


# ===================================================================== #
# TC-EIP-005: arc_on 超时
# ===================================================================== #
def test_TC_EIP_005_arc_on_timeout(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig(scan_timeout=0.3))
    link.connect()
    # 永远不返回 ARC_OK
    eip_robot.t2o["DINT_weld_status"] = [0]
    ok = link.arc_on(wait_ready=True, timeout=0.3)
    assert ok is False


# ===================================================================== #
# TC-EIP-006: arc_off 关闭所有
# ===================================================================== #
def test_TC_EIP_006_arc_off(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    eip_robot.t2o["DINT_weld_status"] = [0b11]
    link.arc_on(wait_ready=True, timeout=0.5)
    # 起弧后 dblout 应该全被置位
    initial_count = sum(1 for v in eip_robot.intout.values() if v > 0)
    assert initial_count >= 1
    link.arc_off(post_flow=0)
    # 收弧后 intout 应该被清零
    final_count = sum(1 for v in eip_robot.intout.values() if v > 0)
    assert final_count < initial_count


# ===================================================================== #
# TC-EIP-007: 位姿触发动作
# ===================================================================== #
def test_TC_EIP_007_register_pose_triggered(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    p0 = [0.4, 0.1, 0.2, 3.1416, 0, 0]
    p1 = [0.5, 0.1, 0.2, 3.1416, 0, 0]
    ok = link.register_pose_triggered_actions(
        device_json="welder.json",
        trigger_poses=[
            (p0, "arc_on", {}),
            (p1, "arc_off", {}),
        ],
    )
    assert ok is True
    assert len(eip_robot.actions) == 2
    set_calls = [c for c in eip_robot.calls if c[0] == "set_external_device_actions"]
    assert len(set_calls) == 1
    assert set_calls[0][1][0] == "welder.json"   # file_path


# ===================================================================== #
# TC-EIP-008: execute_function
# ===================================================================== #
def test_TC_EIP_008_execute_function(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    ok = link.execute_function("welder.json", "set_params",
                                {"job": 5, "current": 150})
    assert ok is True
    fn_calls = [c for c in eip_robot.calls if c[0] == "execute_external_device_function"]
    assert len(fn_calls) == 1


# ===================================================================== #
# TC-EIP-009: safety_stop
# ===================================================================== #
def test_TC_EIP_009_safety_stop(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    assert link.safety_stop("welder.json") is True
    assert any(c[0] == "execute_external_device_safety_stop" for c in eip_robot.calls)


# ===================================================================== #
# TC-EIP-010: read_actual_current/voltage
# ===================================================================== #
def test_TC_EIP_010_read_actual_values(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    eip_robot.t2o["DINT_current_actual"] = [195.5]
    eip_robot.t2o["DINT_voltage_actual"] = [23.7]
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    assert link.read_actual_current() == 195.5
    assert link.read_actual_voltage() == 23.7


# ===================================================================== #
# TC-EIP-011: 状态位检查
# ===================================================================== #
def test_TC_EIP_011_status_bits(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    eip_robot.t2o["DINT_weld_status"] = [0b11]      # READY+ARC_OK
    assert link.is_ready() is True
    assert link.is_arc_ok() is True
    assert link.is_in_error() is False
    # 改 status + 强制刷新缓存 (睡眠 > TTL)
    eip_robot.t2o["DINT_weld_status"] = [0b10000011]  # + ERROR
    time.sleep(0.06)   # _STATUS_TTL_S = 0.05
    assert link.is_in_error() is True


# ===================================================================== #
# TC-EIP-012: read_error_code
# ===================================================================== #
def test_TC_EIP_012_read_error_code(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    eip_robot.t2o["DINT_error_code"] = [42]
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    assert link.read_error_code() == 42


# ===================================================================== #
# TC-EIP-013: 上下文管理器
# ===================================================================== #
def test_TC_EIP_013_context_manager(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    with EIPWeldingLink(eip_robot, EIPWeldingConfig()) as link:
        assert link.is_connected() is True
    assert eip_robot.connected is False


# ===================================================================== #
# TC-EIP-014: arc_on 幂等
# ===================================================================== #
def test_TC_EIP_014_arc_on_idempotent(eip_robot):
    from welding_package import EIPWeldingLink, EIPWeldingConfig
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    link.connect()
    eip_robot.t2o["DINT_weld_status"] = [0b11]
    link.arc_on(wait_ready=False)
    n1 = sum(1 for c in eip_robot.calls if c[0] == "set_dblout_register")
    link.arc_on(wait_ready=False)   # 第二次
    n2 = sum(1 for c in eip_robot.calls if c[0] == "set_dblout_register")
    assert n1 == n2


# ===================================================================== #
# TC-EIP-015: EIPBackedWeldingController
# ===================================================================== #
def test_TC_EIP_015_backed_controller(eip_robot):
    from welding_package import (
        EIPWeldingLink, EIPWeldingConfig, EIPBackedWeldingController,
    )
    link = EIPWeldingLink(eip_robot, EIPWeldingConfig())
    ctrl = EIPBackedWeldingController(eip_robot, link, device_json="welder.json")
    ctrl.setup()
    assert link.is_connected() is True
    eip_robot.t2o["DINT_weld_status"] = [0b11]
    assert ctrl.arc_on() is True
    assert ctrl._arc_on is True
    ctrl.arc_off()
    assert ctrl._arc_on is False
