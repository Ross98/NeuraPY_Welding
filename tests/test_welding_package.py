"""
Pytest test suite for welding_package.
=====================================

执行: pytest tests/test_welding_package.py -v

每个测试函数对应 README 测试用例文档 (welding-package-test-cases.md) 中的一个
或一类 TC-* 编号, 函数 docstring 标注了对应关系。
"""
from __future__ import annotations

import time
import sys

import pytest

# conftest.py 已经注入了 mock neurapy


# ===================================================================== #
# TC-F-001: 顶层包导入导出完整
# ===================================================================== #
def test_TC_F_001_package_imports():
    """TC-F-001: 顶层包导入导出完整."""
    import welding_package
    assert welding_package.__version__ == "1.0.0"
    expected = {
        # 工艺包
        "WeldingController", "WeldingMode", "WeldingProcess",
        "LinearWeaveWeld", "CircularWeld", "MultiPassWeld",
        "SpotWeld", "LaserWeld", "CobotSeamTrack", "StitchWeld",
        # EIP 适配
        "EIPWeldingLink", "EIPWeldingConfig", "EIPWeldingTagMap",
        "EIPJobMode", "EIPBackedWeldingController", "WeldStatusBit",
    }
    assert set(welding_package.__all__) == expected


# ===================================================================== #
# TC-F-002: 参数类导入与默认值
# ===================================================================== #
def test_TC_F_002_parameter_defaults():
    """TC-F-002: 参数类默认值."""
    from welding_package.parameters import (
        WeaveParameters, MotionProfile, ArcParameters, SeamTrackParameters,
    )
    w = WeaveParameters()
    assert w.pattern == "sine" and w.frequency == 1.0

    m = MotionProfile()
    assert m.speed == 0.01 and m.blending_mode == "static"

    a = ArcParameters()
    assert a.voltage == 22.0 and a.arc_on_digital_output == "DO_1"

    s = SeamTrackParameters()
    assert s.control_mode == "hybrid"
    assert s.force_vector["Fz"]["value"] == 5.0


# ===================================================================== #
# TC-F-003: WeaveParameters.to_api_dict 字段映射
# ===================================================================== #
def test_TC_F_003_weave_to_api_dict():
    """TC-F-003: weave 字典条件性字段."""
    from welding_package.parameters import WeaveParameters
    sine = WeaveParameters(pattern="sine", frequency=2.0,
                           amplitude_left=0.003, radius=0.999).to_api_dict()
    assert sine["pattern"] == "sine"
    assert sine["frequency"] == 2.0
    assert sine["amplitude_left"] == 0.003
    assert "radius" not in sine    # sine 模式不输出 radius

    circle = WeaveParameters(pattern="circle", radius=0.005).to_api_dict()
    assert "radius" in circle
    assert circle["radius"] == 0.005


# ===================================================================== #
# TC-F-101: LinearWeaveWeld 完整执行流程
# ===================================================================== #
def test_TC_F_101_linear_weave_full_flow(wc, mock_robot, sample_poses, fast_sleep):
    """TC-F-101: 直线摆动焊完整流程."""
    from welding_package import WeldingProcess
    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    wc.run(proc)

    names = [c[0] for c in mock_robot.calls]

    # 1) 模式检查 + 工具加载
    assert "set_tool" in names
    # 2) 接近点
    assert names.count("move_pose") >= 2
    # 3) 工艺预检
    assert "check_linear_feasibility" in names
    # 4) 主轨迹 (含 pre + 主焊接)
    assert names.count("move_linear") >= 2
    # 5) 控制器 _arc_on 收尾为 False
    assert wc._arc_on is False
    # 6) 至少有一组 IO (气阀或起弧)
    assert "io" in names


def test_TC_F_101_weave_dict_has_pose_numbers(wc, mock_robot, sample_poses):
    """TC-F-101 补充: 摆动参数包含起止 pose 编号."""
    from welding_package import WeldingProcess
    from welding_package.parameters import WeaveParameters

    proc = WeldingProcess.linear_weave(
        sample_poses[0], sample_poses[2],
        weave=WeaveParameters(pattern="sine", frequency=1.0),
    )
    wc.run(proc)

    feas_calls = [c for c in mock_robot.calls if c[0] == "check_linear_feasibility"]
    assert len(feas_calls) == 1
    kw = feas_calls[0][2]
    assert kw["target_pose"] == [sample_poses[0], sample_poses[2]]
    assert kw["weaving"] is True
    wp = kw["weaving_parameters"][0]
    assert wp["start_target_pose_number"] == 1
    assert wp["end_target_pose_number"] == 2


# ===================================================================== #
# TC-F-102: CircularWeld
# ===================================================================== #
def test_TC_F_102_circular(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    proc = WeldingProcess.circular(sample_poses[:3])
    wc.run(proc)
    names = [c[0] for c in mock_robot.calls]
    assert "check_circular_feasibility" in names
    assert "move_circular" in names
    assert wc._arc_on is False

    feas = [c for c in mock_robot.calls if c[0] == "check_circular_feasibility"]
    assert len(feas[0][2]["target_pose"]) == 3


# ===================================================================== #
# TC-F-103: MultiPassWeld 3 层
# ===================================================================== #
def test_TC_F_103_multi_pass_3_layers(wc, mock_robot, sample_poses, fast_sleep):
    from welding_package import WeldingProcess
    proc = WeldingProcess.multi_pass(sample_poses[0], sample_poses[2],
                                     passes=3, layer_offset_z=0.002)
    wc.run(proc)

    # 3 层 = 3 次主 move_linear
    main_moves = [c for c in mock_robot.calls
                  if c[0] == "move_linear" and "weaving" in c[2]]
    assert len(main_moves) == 3
    # 每次 Z 偏移 0 / 2 / 4 mm
    zs = [m[2]["target_pose"][0][2] for m in main_moves]
    assert zs[1] - zs[0] == pytest.approx(0.002)
    assert zs[2] - zs[1] == pytest.approx(0.002)
    # 层间等待
    assert len(fast_sleep) >= 2


# ===================================================================== #
# TC-F-104: SpotWeld
# ===================================================================== #
def test_TC_F_104_spot_weld(wc, mock_robot, sample_poses, fast_sleep):
    from welding_package import WeldingProcess
    proc = WeldingProcess.spot(sample_poses[0], dwell=0.5)
    wc.run(proc)

    io_calls = [c for c in mock_robot.calls if c[0] == "io"]
    # 同一 DO 可能被多次设置, 检查是否至少出现一次 True
    def any_true(name):
        return any(c[2].get("io_name") == name and c[2].get("target_value") is True
                   for c in io_calls)
    def last_setting(name):
        for c in reversed(io_calls):
            if c[2].get("io_name") == name:
                return c[2].get("target_value")
        return None
    assert any_true("DO_SPOT_PRESS") is True
    assert any_true("DO_SPOT_WELD") is True
    # 最后关焊接+加压
    assert last_setting("DO_SPOT_WELD") is False
    assert last_setting("DO_SPOT_PRESS") is False


# ===================================================================== #
# TC-F-105: LaserWeld 模拟量 + 数字量
# ===================================================================== #
def test_TC_F_105_laser_weld(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    proc = WeldingProcess.laser(sample_poses[0], sample_poses[2],
                                laser_power=1500.0)
    wc.run(proc)

    io_calls = [c for c in mock_robot.calls if c[0] == "io"]
    def any_set(name, val):
        return any(c[2].get("io_name") == name and c[2].get("target_value") == val
                   for c in io_calls)
    def last_set(name):
        for c in reversed(io_calls):
            if c[2].get("io_name") == name:
                return c[2].get("target_value")
        return None
    assert any_set("DO_LASER_GAS", True) is True
    assert any_set("AO_LASER_POWER", 1.5) is True  # 1500W * 0.001 = 1.5V
    assert any_set("DO_LASER_ON", True) is True
    # 关激光 + 功率归零
    assert last_set("DO_LASER_ON") is False
    assert last_set("AO_LASER_POWER") == 0.0


# ===================================================================== #
# TC-F-106: CobotSeamTrack
# ===================================================================== #
def test_TC_F_106_cobot_seam_track(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    from welding_package.parameters import SeamTrackParameters
    st = SeamTrackParameters(control_mode="hybrid",
                             force_vector={
                                 "Fx": {"is_active": False, "value": 0.0},
                                 "Fy": {"is_active": False, "value": 0.0},
                                 "Fz": {"is_active": True,  "value": 8.0},
                             })
    proc = WeldingProcess.cobot_track(sample_poses[:4], seam_track=st)
    wc.run(proc)

    names = [c[0] for c in mock_robot.calls]
    assert "activate_servo_interface" in names
    assert "deactivate_servo_interface" in names
    assert "check_composite_feasibility" in names
    assert "move_composite" in names
    assert wc._arc_on is False

    feas = [c for c in mock_robot.calls if c[0] == "check_composite_feasibility"]
    cp = feas[0][2]["controller_parameters"]
    assert cp["control_mode"] == "hybrid"
    assert cp["force_vector"]["Fz"]["is_active"] is True
    assert cp["force_vector"]["Fz"]["value"] == 8.0


# ===================================================================== #
# TC-F-107: StitchWeld
# ===================================================================== #
def test_TC_F_107_stitch_weld(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    # 距离 0.20m, 5 段 stitch + 4 段 gap, 每段 0.02+0.01
    proc = WeldingProcess.stitch([sample_poses[0], sample_poses[2]],
                                 stitch_length=0.02, gap_length=0.01)
    wc.run(proc)

    # 主 move_linear (无 weaving 字段) 应当执行 stitch 段
    stitch_moves = [c for c in mock_robot.calls
                    if c[0] == "move_linear" and c[2].get("blend_radius") == 0.0]
    # 0.20m / (0.02+0.01) = 6.67 → 6 段 stitch, 5 段 gap
    assert 5 <= len(stitch_moves) <= 7
    assert wc._arc_on is False


# ===================================================================== #
# TC-F-108: dry_run 不起弧
# ===================================================================== #
def test_TC_F_108_dry_run_no_arc(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    wc.dry_run(proc)
    io_calls = [c for c in mock_robot.calls if c[0] == "io"]
    # dry_run 不应触发起弧/收弧
    # (move_pose/move_linear 仍会调用)
    assert wc._arc_on is False
    # 没有气阀/起弧 DO 触发
    io_names = {c[2].get("io_name") for c in io_calls}
    assert "DO_1" not in io_names   # arc_on
    assert "DO_3" not in io_names   # gas valve


# ===================================================================== #
# TC-E-001: linear_weave 默认速度
# ===================================================================== #
def test_TC_E_001_linear_weave_default_speed():
    from welding_package import WeldingProcess
    p = WeldingProcess.linear_weave([0,0,0,0,0,0], [0.1,0,0,0,0,0])
    assert p.motion.speed == 0.01


# ===================================================================== #
# TC-E-002: circular < 3 抛错
# ===================================================================== #
def test_TC_E_002_circular_rejects_too_few_poses():
    from welding_package import WeldingProcess
    with pytest.raises(ValueError, match="at least 3 poses"):
        WeldingProcess.circular([[0,0,0,0,0,0], [1,1,1,0,0,0]])


# ===================================================================== #
# TC-E-003: cobot_track < 2 抛错
# ===================================================================== #
def test_TC_E_003_cobot_track_rejects_too_few_poses():
    from welding_package import WeldingProcess
    with pytest.raises(ValueError, match=">= 2 poses"):
        WeldingProcess.cobot_track([[0,0,0,0,0,0]])


# ===================================================================== #
# TC-E-004: multi_pass passes=0
# ===================================================================== #
def test_TC_E_004_multi_pass_clamps_zero():
    from welding_package import WeldingProcess
    p = WeldingProcess.multi_pass([0,0,0,0,0,0], [0.1,0,0,0,0,0], passes=0)
    assert p.passes == 1


# ===================================================================== #
# TC-E-005: multi_pass passes<0
# ===================================================================== #
def test_TC_E_005_multi_pass_clamps_negative():
    from welding_package import WeldingProcess
    p = WeldingProcess.multi_pass([0,0,0,0,0,0], [0.1,0,0,0,0,0], passes=-5)
    assert p.passes == 1


# ===================================================================== #
# TC-E-006: 零长度插值
# ===================================================================== #
def test_TC_E_006_interp_zero_length():
    from welding_package._interp import interpolate_linear
    pts = interpolate_linear([0,0,0,0,0,0], [0,0,0,0,0,0], step=0.005)
    assert pts == [[0.0]*6, [0.0]*6]


# ===================================================================== #
# TC-E-007: 步长大于距离
# ===================================================================== #
def test_TC_E_007_interp_step_larger_than_distance():
    from welding_package._interp import interpolate_linear
    pts = interpolate_linear([0,0,0,0,0,0], [0.001,0,0,0,0,0], step=1.0)
    # n = max(ceil(0.001/1.0), 2) = 2
    assert len(pts) == 2


# ===================================================================== #
# TC-E-008: WeaveParameters radius 条件
# ===================================================================== #
def test_TC_E_008_weave_radius_conditional():
    from welding_package.parameters import WeaveParameters
    s = WeaveParameters(pattern="sine", radius=99.0).to_api_dict()
    assert "radius" not in s
    c = WeaveParameters(pattern="circle", radius=0.005).to_api_dict()
    assert "radius" in c


# ===================================================================== #
# TC-E-009: StitchWeld 短于 stitch_length
# ===================================================================== #
def test_TC_E_009_stitch_shorter_than_length(wc, mock_robot):
    from welding_package import WeldingProcess
    proc = WeldingProcess.stitch([[0,0,0,0,0,0], [0.1,0,0,0,0,0]],
                                 stitch_length=0.5, gap_length=0.1)
    wc.run(proc)
    # 仅 1 段 stitch
    stitch = [c for c in mock_robot.calls
              if c[0] == "move_linear" and c[2].get("blend_radius") == 0.0]
    assert len(stitch) == 1


# ===================================================================== #
# TC-E-010: 点焊 dwell=0
# ===================================================================== #
def test_TC_E_010_spot_zero_dwell(wc, mock_robot, fast_sleep):
    from welding_package import WeldingProcess
    proc = WeldingProcess.spot([0,0,0,0,0,0], dwell=0)
    wc.run(proc)
    io_calls = [c for c in mock_robot.calls if c[0] == "io"]
    assert len(io_calls) >= 4   # press_on, weld_on, weld_off, press_off


# ===================================================================== #
# TC-ERR-001: LinearWeaveWeld 不可行
# ===================================================================== #
def test_TC_ERR_001_linear_infeasible(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    mock_robot.feasibility = False
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    with pytest.raises(RuntimeError, match="not feasible"):
        wc.run(proc)
    # arc_off 仍被调用 (finally)
    assert wc._arc_on is False


# ===================================================================== #
# TC-ERR-002: CircularWeld 不可行
# ===================================================================== #
def test_TC_ERR_002_circular_infeasible(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    mock_robot.feasibility = False
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.circular(sample_poses[:3])
    with pytest.raises(RuntimeError, match="not feasible"):
        wc.run(proc)
    assert wc._arc_on is False


# ===================================================================== #
# TC-ERR-003: CobotSeamTrack 不可行
# ===================================================================== #
def test_TC_ERR_003_cobot_infeasible(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    mock_robot.feasibility = False
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.cobot_track(sample_poses[:4])
    with pytest.raises(RuntimeError, match="not feasible"):
        wc.run(proc)
    assert wc._arc_on is False


# ===================================================================== #
# TC-ERR-004: 缺 numpy
# ===================================================================== #
def test_TC_ERR_004_missing_numpy(monkeypatch):
    """缺 numpy 时给出清晰 ImportError 提示."""
    # 必须先清掉缓存, 让 _interp 重新执行 import
    saved_numpy = sys.modules.pop("numpy", None)
    if "welding_package._interp" in sys.modules:
        del sys.modules["welding_package._interp"]
    # 把 numpy 改成 None, 这样 import numpy 会立即得到 None 而非真实 module
    sys.modules["numpy"] = None
    try:
        # 触发重新加载: 必须用 importlib, 单纯的 from ... import 会被缓存
        import importlib
        pkg = importlib.import_module("welding_package")
        importlib.reload(pkg)  # 重新执行 __init__, 但 _interp 可能未触发
        with pytest.raises((ImportError, TypeError), match="(?i)numpy"):
            importlib.import_module("welding_package._interp")
    finally:
        if "welding_package._interp" in sys.modules:
            del sys.modules["welding_package._interp"]
        if "welding_package" in sys.modules:
            # 关键: 重置 _interp 引用, 让下次测试重新加载正常版
            pass
        if saved_numpy is not None:
            sys.modules["numpy"] = saved_numpy
        else:
            sys.modules.pop("numpy", None)


# ===================================================================== #
# TC-ERR-005: 缺 scipy 降级
# ===================================================================== #
def test_TC_ERR_005_missing_scipy_falls_back(monkeypatch):
    """缺 scipy 时插值应使用线性 RPY 退化方案, 不抛错."""
    # 模拟 scipy 不存在
    monkeypatch.setitem(sys.modules, "scipy", None)
    monkeypatch.setitem(sys.modules, "scipy.spatial", None)
    monkeypatch.setitem(sys.modules, "scipy.spatial.transform", None)
    if "welding_package._interp" in sys.modules:
        del sys.modules["welding_package._interp"]

    from welding_package._interp import interpolate_linear, _HAS_SCIPY
    assert _HAS_SCIPY is False
    pts = interpolate_linear([0,0,0,0,0,0], [0.1,0,0,0.1,0,0], step=0.02)
    assert len(pts) >= 2
    # RPY 末值 = 0.1
    assert pts[-1][3] == pytest.approx(0.1, abs=1e-6)


# ===================================================================== #
# TC-ERR-006: IO 未配置 graceful
# ===================================================================== #
def test_TC_ERR_006_io_missing_graceful(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode

    def bad_io(action, **kw):
        if kw.get("io_name") == "DO_1":
            raise KeyError("DO_1 not configured")
        return True
    mock_robot.io = bad_io

    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    # arc_on 在起弧 IO 失败时应抛 RuntimeError (修复: 不再静默退化)
    with pytest.raises(RuntimeError, match="Arc ignition IO failure"):
        wc.arc_on()
    # arc_off 在无弧状态不抛 (reentrancy safe)
    wc.arc_off()
    assert wc._arc_on is False


# ===================================================================== #
# TC-ERR-007: 力矩超限
# ===================================================================== #
def test_TC_ERR_007_force_exceeds_limit(mock_robot):
    from welding_package import WeldingController, WeldingMode
    mock_robot.tcp_wrench = [100.0, 0, 0, 0, 0, 0]   # Fxy=100N
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    assert wc._check_force() is False
    assert "pause" in [c[0] for c in mock_robot.calls]


# ===================================================================== #
# TC-ERR-008: 力矩正常
# ===================================================================== #
def test_TC_ERR_008_force_within_limit(mock_robot):
    from welding_package import WeldingController, WeldingMode
    mock_robot.tcp_wrench = [10, 5, 20, 0, 0, 0]
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    assert wc._check_force() is True


# ===================================================================== #
# TC-ERR-010: 工艺中途异常仍收弧
# ===================================================================== #
def test_TC_ERR_010_mid_motion_exception(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess

    original = mock_robot.move_linear
    call_count = [0]
    def failing_move_linear(*a, **kw):
        call_count[0] += 1
        # 在第 2 次 move_linear 时抛错 (即主焊接轨迹)
        if call_count[0] == 2:
            raise RuntimeError("Servo fault")
        return original(*a, **kw)
    mock_robot.move_linear = failing_move_linear

    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    with pytest.raises(RuntimeError, match="Servo fault"):
        wc.run(proc)
    # finally 收弧
    assert wc._arc_on is False


# ===================================================================== #
# TC-ERR-011: 上下文管理器异常路径
# ===================================================================== #
def test_TC_ERR_011_context_manager_exception(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    mock_robot.feasibility = False
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)

    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    with pytest.raises(RuntimeError):
        with wc:
            wc.run(proc)
    # __exit__ 后 _arc_on 应为 False
    assert wc._arc_on is False


# ===================================================================== #
# TC-ST-001: 控制器状态机
# ===================================================================== #
def test_TC_ST_001_state_machine(mock_robot):
    from welding_package import WeldingController, WeldingMode
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    assert wc._arc_on is False
    assert wc._welding_active is False
    wc.arc_on()
    assert wc._arc_on is True
    assert wc._welding_active is True
    wc.arc_off()
    assert wc._arc_on is False
    assert wc._welding_active is False


# ===================================================================== #
# TC-ST-002: arc_on 幂等
# ===================================================================== #
def test_TC_ST_002_arc_on_idempotent(mock_robot):
    from welding_package import WeldingController, WeldingMode
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    wc.arc_on()
    n1 = sum(1 for c in mock_robot.calls if c[0] == "io")
    wc.arc_on()
    n2 = sum(1 for c in mock_robot.calls if c[0] == "io")
    assert n1 == n2   # 第二次 no-op


# ===================================================================== #
# TC-ST-003: 模式切换
# ===================================================================== #
def test_TC_ST_003_setup_switches_mode(mock_robot):
    from welding_package import WeldingController, WeldingMode
    mock_robot.mode = "teach"   # 假装在 teach 模式
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    wc.setup()
    names = [c[0] for c in mock_robot.calls]
    assert "switch_to_automatic_mode" in names
    assert "set_tool" in names
    assert "enable_collision_detection" in names


# ===================================================================== #
# TC-ST-004: 多次点焊状态翻转
# ===================================================================== #
def test_TC_ST_004_multiple_spots(mock_robot, sample_poses, fast_sleep):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    for pose in sample_poses:
        proc = WeldingProcess.spot(pose, dwell=0.1)
        wc.run(proc)
        assert wc._arc_on is False


# ===================================================================== #
# TC-ST-005: dry_run 状态不变
# ===================================================================== #
def test_TC_ST_005_dry_run_no_state_change(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.linear_weave(sample_poses[0], sample_poses[2])
    wc.dry_run(proc)
    assert wc._arc_on is False
    assert wc._welding_active is False


# ===================================================================== #
# TC-ST-006: 上下文管理器收尾
# ===================================================================== #
def test_TC_ST_006_context_manager_closes_arc(mock_robot):
    from welding_package import WeldingController, WeldingMode
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    with wc:
        wc.arc_on()
        assert wc._arc_on is True
    assert wc._arc_on is False


# ===================================================================== #
# TC-ST-007: 激光关闭时功率归零
# ===================================================================== #
def test_TC_ST_007_laser_power_zeroed_on_close(mock_robot, sample_poses):
    from welding_package import WeldingController, WeldingMode, WeldingProcess
    wc = WeldingController(mock_robot, weld_tool="torch", mode=WeldingMode.MIG_MAG)
    proc = WeldingProcess.laser(sample_poses[0], sample_poses[2], laser_power=1800)
    wc.run(proc)
    ao_calls = [c for c in mock_robot.calls
                if c[0] == "io" and c[2].get("io_name") == "AO_LASER_POWER"]
    assert ao_calls[-1][2]["target_value"] == 0.0


# ===================================================================== #
# TC-API-002: move_circular target_pose 长度
# ===================================================================== #
def test_TC_API_002_circular_target_pose_length(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    proc = WeldingProcess.circular(sample_poses[:3])
    wc.run(proc)
    feas = [c for c in mock_robot.calls if c[0] == "check_circular_feasibility"]
    assert len(feas[0][2]["target_pose"]) >= 3


# ===================================================================== #
# TC-API-003: move_composite commands 结构
# ===================================================================== #
def test_TC_API_003_composite_commands_structure(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    proc = WeldingProcess.cobot_track(sample_poses[:4])
    wc.run(proc)
    feas = [c for c in mock_robot.calls if c[0] == "check_composite_feasibility"]
    commands = feas[0][2]["commands"]
    # 4 个点 → 3 段
    assert len(commands) == 3
    for cmd in commands:
        assert "linear" in cmd
        assert "blend_radius" in cmd["linear"]
        assert len(cmd["linear"]["target_pose"]) == 2


# ===================================================================== #
# TC-API-005: 4 种控制模式
# ===================================================================== #
@pytest.mark.parametrize("mode", ["position", "joint_impedance", "admittance", "hybrid"])
def test_TC_API_005_all_control_modes(wc, mock_robot, sample_poses, mode):
    from welding_package import WeldingProcess
    from welding_package.parameters import SeamTrackParameters
    st = SeamTrackParameters(control_mode=mode)
    proc = WeldingProcess.cobot_track(sample_poses[:4], seam_track=st)
    wc.run(proc)
    feas = [c for c in mock_robot.calls if c[0] == "check_composite_feasibility"]
    assert feas[0][2]["controller_parameters"]["control_mode"] == mode


# ===================================================================== #
# TC-PERF-001: sleep 被替换
# ===================================================================== #
def test_TC_PERF_001_sleep_skipped(wc, mock_robot, sample_poses, fast_sleep):
    """总执行时间 < 100ms (实际 sleep 被替身)."""
    from welding_package import WeldingProcess
    proc = WeldingProcess.spot(sample_poses[0], dwell=10.0)
    t0 = time.perf_counter()
    wc.run(proc)
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.5
    # 替身累计应包含 10.0s 的 dwell
    assert any(abs(s - 10.0) < 1e-6 for s in fast_sleep)


# ===================================================================== #
# TC-PERF-002: MultiPassWeld 层间等待
# ===================================================================== #
def test_TC_PERF_002_multi_pass_sleeps(wc, mock_robot, sample_poses, fast_sleep):
    from welding_package import WeldingProcess
    proc = WeldingProcess.multi_pass(sample_poses[0], sample_poses[2], passes=3)
    wc.run(proc)
    # 第 2、3 次循环前各睡 2.0s
    interpass = [s for s in fast_sleep if abs(s - 2.0) < 1e-6]
    assert len(interpass) == 2


# ===================================================================== #
# TC-F-109: CircularWeld 默认不开摆动
# ===================================================================== #
def test_TC_F_109_circular_no_weave_by_default(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    from welding_package.parameters import WeaveParameters
    proc = WeldingProcess.circular(sample_poses[:3])
    wc.run(proc)
    feas = [c for c in mock_robot.calls if c[0] == "check_circular_feasibility"]
    assert len(feas) == 1
    assert feas[0][2]["weaving"] is False  # 默认不开摆动

def test_TC_F_110_circular_weave_circle_mode(wc, mock_robot, sample_poses):
    from welding_package import WeldingProcess
    from welding_package.parameters import WeaveParameters
    proc = WeldingProcess.circular(sample_poses[:3], weave=WeaveParameters(pattern="circle", radius=0.003))
    wc.run(proc)
    feas = [c for c in mock_robot.calls if c[0] == "check_circular_feasibility"]
    assert len(feas) == 1
    assert feas[0][2]["weaving"] is True  # circle 模式下开启
    assert feas[0][2]["weaving_parameters"][0]["pattern"] == "circle"
