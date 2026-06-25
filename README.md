# NeuraPY_Welding

基于 **Neura Robotics Neurapy v5.1.88** 的焊接工艺包。封装 7 种常见焊接工艺 + **EtherNet/IP** 工业总线通讯，可直接用于 Lara 5/8、Maira 7 等机器人。

## 特性

| 模块 | 能力 |
|---|---|
| **7 种焊接工艺** | 直线摆动、圆弧圆周、多层多道、电阻点焊、激光焊、力控焊缝跟踪、断续段焊 |
| **EIP 焊机通讯** | 机器人作为 CIP Scanner，读写焊机 Adapter 寄存器，支持 Fronius / Lincoln / Miller / OTC |
| **两种 EIP 模式** | 轮询 (`write_job` + `arc_on`) + 位置触发 (`set_external_device_actions`) |
| **安全监控** | TCP 速度限幅、力矩超限自动暂停、碰撞检测 |
| **测试覆盖** | 62 个 pytest 测试，mock Neurapy 运行，CI 自动验证 |

## 快速开始

```python
import time
from neurapy.robot import Robot
from welding_package import WeldingController, WeldingMode, WeldingProcess

r = Robot("192.168.2.13")         # 机器人 IP
r.power_on(); time.sleep(2)

with WeldingController(r, weld_tool="welding_torch") as wc:
    proc = WeldingProcess.linear_weave(
        start_pose=[0.40, 0.10, 0.20, 3.1416, 0, 0],
        end_pose  =[0.60, 0.10, 0.20, 3.1416, 0, 0],
        speed=0.012,               # 12 mm/s
    )
    wc.dry_run(proc)               # 走空验证
    time.sleep(0.5)
    wc.run(proc)                   # 正式焊接
```

### 安装

```bash
pip install neurapy scipy numpy pytest   # 部署
# neurapy 是 Neura Robotics 私有 SDK，联系 Neura 获取
```

## 目录结构

```
NeuraPY_Welding/
  ├── welding_package/           # 焊接工艺包
  │   ├── __init__.py           # 统一导出
  │   ├── welding_controller.py # 焊接控制器 (Digital IO 模式)
  │   ├── processes.py          # 7 种工艺实现
  │   ├── parameters.py         # 工艺参数数据类
  │   ├── eip.py                # EtherNet/IP 适配
  │   ├── _interp.py            # 姿态插值 helper
  │   └── examples/             # 9 个示例
  ├── tests/                    # 62 个 pytest 测试
  │   ├── conftest.py
  │   ├── test_welding_package.py
  │   └── test_eip.py
  ├── requirements.txt
  └── .github/workflows/tests.yml
```

## 焊接工艺

| 工艺 | 类名 | 适用场景 | 关键 API |
|---|---|---|---|
| 直线摆动焊 | `LinearWeaveWeld` | 对接 / 角接 / 搭接平直焊缝 | `move_linear` + `weaving` |
| 圆弧 / 圆周焊 | `CircularWeld` | 管法兰 / 环形 / 桶体外环缝 | `move_circular` |
| 多层多道焊 | `MultiPassWeld` | 厚板 V 形坡口 | 多次 `LinearWeaveWeld` |
| 电阻点焊 | `SpotWeld` | 铆接 / 螺柱焊 | IO 触发 + 定时 |
| 激光焊 | `LaserWeld` | 光纤 / 碟片激光器 | `move_linear` + 模拟量功率 |
| 协作力控跟踪 | `CobotSeamTrack` | 工件变形 / 协作空间 | `move_composite` + 阻抗/力位混合 |
| 断续焊 | `StitchWeld` | 薄板防变形 | 分段 `move_linear` |

## EIP 焊机通讯

### CIP 角色

| 角色 | 本系统 | 行为 |
|---|---|---|
| **Scanner** | 机器人控制器 | 主动发起 CIP 连接，读写焊机 Adapter 寄存器 |
| **Adapter** | 焊机 (Fronius / Lincoln / Miller / OTC) | 被动接受连接，暴露 Process Data |

### 数据流

```
O2T (Originator → Target, Scanner → Adapter):   T2O (Target → Originator, Adapter → Scanner):
  机器人 写 焊机                                     焊机 报告 状态 给 机器人
  ───────────                                       ───────────
  dout_job / dout_current                          din_status / din_error
  dout_voltage / dout_wire_feed                     din_current_actual / din_voltage_actual
  dout_gas_flow / dout_arc_on
```

### 用法

```python
from welding_package import EIPWeldingLink, EIPWeldingConfig, EIPWeldingTagMap

link = EIPWeldingLink(robot, EIPWeldingConfig(
    yaml_path="/etc/neurapy/eip_welder.yaml",
    tags=EIPWeldingTagMap(
        dout_job_0="DOUT_weld_job",
        dout_current_1="DOUT_weld_current",
        dout_voltage_2="DOUT_weld_voltage",
        dout_wire_feed_3="DOUT_wire_feed",
        dout_gas_flow_4="DOUT_gas_flow",
        dout_arc_on="DOUT_arc_on",
        din_status="DINT_weld_status",
        din_error="DINT_error_code",
        din_current_actual="DINT_current_actual",
    ),
))
link.connect()
link.write_job(job=12, current=180, voltage=22, wire_feed=5.0, gas_flow=15.0)
link.arc_on(wait_ready=True, timeout=3.0)
# ... 焊接 ...
link.arc_off()
```

### 位置触发模式 (推荐高速焊接)

到位姿时控制器自动调用焊机函数，延迟 <1ms：

```python
link.register_pose_triggered_actions(
    device_json="welder.json",
    trigger_poses=[
        (p_approach, "set_params", {"job": 12, "current": 180, "voltage": 22}),
        (p_start,    "arc_on",    {}),
        (p_mid,      "adjust",    {"current": 200, "voltage": 24}),
        (p_end,      "arc_off",   {}),
    ],
)
```

## 运行测试

```bash
cd NeuraPY_Welding
pip install numpy scipy pytest
pytest tests/ -v
# 62 passed
```

CI 每次 push 自动运行 （[GitHub Actions](.github/workflows/tests.yml)）。

## 依赖

| 包 | 来源 | 用途 |
|---|---|---|
| `neurapy>=5.1.88` | Neura Robotics 私有 SDK | 机器人控制 |
| `numpy>=1.24` | PyPI | 数值运算 |
| `scipy>=1.10` | PyPI | 姿态插值 (slerp) |
| `pytest>=7.4` | PyPI | 测试 |

## 示例

| 文件 | 内容 |
|---|---|
| `examples/01_linear_weave.py` | 直线摆动焊 |
| `examples/02_circular_weld.py` | 圆周焊 |
| `examples/03_multi_pass.py` | 多层多道焊 |
| `examples/04_spot_weld.py` | 点焊 |
| `examples/05_laser_weld.py` | 激光焊 |
| `examples/06_cobot_seam_track.py` | 力控焊缝跟踪 |
| `examples/07_stitch_weld.py` | 断续焊 |
| `examples/08_composite_path.py` | 复合路径 |
| `examples/09_eip_welding.py` | EIP 焊接 (3 种模式) |

## License

MIT
