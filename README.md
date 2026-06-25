# NeuraPY_Welding

基于 **Neura Robotics Neurapy v5.1.88** Python API 的焊接工艺包，集成
**EtherNet/IP 工业总线**，覆盖常见焊接工艺。

## 特性

- **7 种焊接工艺** — 直线摆动、圆弧、多层多道、点焊、激光焊、协作力控跟踪、断续焊
- **EIP 适配层** — 机器人作为 CIP Scanner，通过 `set_dblout_register` / `get_register_values`
  主动读写焊机 Adapter，支持 Fronius / Lincoln / Miller / OTC 等品牌
- **两种工作模式** — 轮询 (`write_job` / `arc_on`) + 位置触发 (`set_external_device_actions`)

## 快速开始

```bash
pip install neurapy scipy numpy
```

详细文档见 [`welding_package/README.md`](welding_package/README.md)。

## 目录

```
welding_package/
  ├── __init__.py           # 统一导出
  ├── parameters.py         # 工艺参数数据类
  ├── welding_controller.py # 焊接控制器 (Digital IO 模式)
  ├── processes.py          # 7 种工艺实现
  ├── eip.py                # EtherNet/IP 适配 (CIP Scanner)
  ├── _interp.py            # 姿态插值 helper
  └── examples/             # 9 个示例

tests/
  ├── conftest.py              # pytest fixtures (mock Neurapy)
  ├── test_welding_package.py  # 47 工艺测试
  └── test_eip.py              # 15 EIP 测试
```

## License

MIT
