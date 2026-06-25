"""
Shared pytest fixtures for welding_package tests.

提供:
  - mock_robot: 替身 Robot 类, 记录所有调用
  - fast_sleep: 让 time.sleep 立即返回
  - sample_poses: 常用测试位姿
"""
import sys
import time
import types
from pathlib import Path

# 让 `import welding_package` 在 pytest 收集时能找到包
# 仓库根 = tests/ 的父目录
sys.path.insert(0, str(Path(__file__).parent.parent))

import pytest


# --------------------------------------------------------------------- #
# 1) 构造一个完整的 neurapy mock
# --------------------------------------------------------------------- #
class MockRobot:
    """替身 Robot, 记录所有调用, 默认返回合理值."""

    def __init__(self, *args, **kwargs):
        self.calls = []                # [(method_name, args, kwargs)]
        self.joint_angles = [0.0] * 6
        self.tcp_pose = [0.5, 0.0, 0.2, 3.1416, 0.0, 0.0]
        self.tcp_wrench = [0.1, 0.2, 5.0, 0.0, 0.0, 0.0]
        self.tcp_speed = 0.01
        self.errors = []
        self.feasibility = True
        self.mode = "automatic"

    def _log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def power_on(self):                                self._log("power_on")
    def power_off(self):                               self._log("power_off")
    def is_robot_powered_on(self):                     return True
    def is_robot_in_teach_mode(self):                  return self.mode == "teach"
    def switch_to_automatic_mode(self):                self.mode = "automatic"; self._log("switch_to_automatic_mode")
    def switch_to_teach_mode(self):                    self.mode = "teach"; self._log("switch_to_teach_mode")
    def set_tool(self, name):                          self._log("set_tool", name)
    def get_current_joint_angles(self):                return list(self.joint_angles)
    def get_tcp_pose(self):                            return list(self.tcp_pose)
    def get_tcp_wrench(self):                          return list(self.tcp_wrench)
    def get_tcp_speed(self):                           return self.tcp_speed
    def get_errors(self):                              return list(self.errors)
    def enable_collision_detection(self):             self._log("enable_collision_detection")
    def disable_collision_detection(self):             self._log("disable_collision_detection")
    def move_pose(self, *a, **kw):                     self._log("move_pose", *a, **kw)
    def move_linear(self, *a, **kw):                   self._log("move_linear", *a, **kw)
    def move_circular(self, *a, **kw):                self._log("move_circular", *a, **kw)
    def move_composite(self, *a, **kw):               self._log("move_composite", *a, **kw)
    def plan_move_linear(self, *a, **kw):             return 1
    def check_linear_feasibility(self, *a, **kw):      self._log("check_linear_feasibility", *a, **kw); return [self.feasibility, [0.0]*6]
    def check_circular_feasibility(self, *a, **kw):   self._log("check_circular_feasibility", *a, **kw); return [self.feasibility, [0.0]*6]
    def check_composite_feasibility(self, *a, **kw):  self._log("check_composite_feasibility", *a, **kw); return [self.feasibility, [0.0]*6]
    def wait_motion_finished(self):                    self._log("wait_motion_finished")
    def pause(self):                                   self._log("pause")
    def resume(self):                                  self._log("resume")
    def io(self, action, **kw):                        self._log("io", action, **kw); return True
    def activate_servo_interface(self, mode):          self._log("activate_servo_interface", mode)
    def deactivate_servo_interface(self):              self._log("deactivate_servo_interface")
    def set_override(self, v):                         self._log("set_override", v)
    def get_override(self):                            return 1.0


# --------------------------------------------------------------------- #
# 2) 注入到 sys.modules (模块顶层, 不是 fixture!)
#    必须在所有 test 文件 import welding_package 之前完成,
#    因为 __init__.py 触发 from neurapy.robot import Robot.
# --------------------------------------------------------------------- #
_neurapy = types.ModuleType("neurapy")
_neurapy_robot = types.ModuleType("neurapy.robot")
_neurapy_robot.Robot = MockRobot
_neurapy.robot = _neurapy_robot
# 清理旧包缓存 (如果 neurapy 已被其他包预装载为 None 等)
for k in list(sys.modules):
    if k == "neurapy" or k.startswith("neurapy."):
        del sys.modules[k]
sys.modules["neurapy"] = _neurapy
sys.modules["neurapy.robot"] = _neurapy_robot


# --------------------------------------------------------------------- #
# 3) 让 time.sleep 立即返回
# --------------------------------------------------------------------- #
@pytest.fixture
def fast_sleep(monkeypatch):
    sleeps = []
    def _fake_sleep(s):
        sleeps.append(s)
    monkeypatch.setattr(time, "sleep", _fake_sleep)
    return sleeps


# --------------------------------------------------------------------- #
# 4) 常用 fixture
# --------------------------------------------------------------------- #
@pytest.fixture
def mock_robot():
    return MockRobot()


@pytest.fixture
def wc(mock_robot):
    """默认的 WeldingController, 已调用 setup()."""
    from welding_package import WeldingController, WeldingMode
    c = WeldingController(mock_robot, weld_tool="torch",
                          mode=WeldingMode.MIG_MAG)
    c.setup()
    return c


@pytest.fixture
def sample_poses():
    """5 个标准测试位姿 (X, Y, Z, Rx, Ry, Rz)."""
    return [
        [0.40,  0.10, 0.20, 3.1416, 0.0, 0.0],
        [0.50,  0.10, 0.20, 3.1416, 0.0, 0.0],
        [0.60,  0.10, 0.20, 3.1416, 0.0, 0.0],
        [0.50,  0.20, 0.20, 3.1416, 0.0, 0.0],
        [0.50, -0.20, 0.20, 3.1416, 0.0, 0.0],
    ]


def call_names(mock_robot):
    """Helper: 提取所有调用过的方法名序列."""
    return [c[0] for c in mock_robot.calls]


def calls_of(mock_robot, name):
    """Helper: 提取指定方法的调用列表 (含参数)."""
    return [c for c in mock_robot.calls if c[0] == name]
