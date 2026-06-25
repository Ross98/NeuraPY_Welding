"""
Welding Process Package for Neura Robotics Neurapy v5.1.88
============================================================

A Python package providing common welding processes and motion patterns
for Neura robot arms (Lara 5/8, Maira 7, etc.), built on top of the
Neurapy Robot class.

Supported processes:
  - LinearWeaveWeld:    Straight seam with sinusoidal/triangular weaving
  - CircularWeld:       Circular / arc seam (e.g. pipe circumference)
  - MultiPassWeld:      Multi-pass layered welding
  - SpotWeld:           Resistance / spot welding by IO trigger
  - LaserWeld:          Laser welding with seam tracking
  - CobotSeamTrack:     Collaborative seam tracking via force control
  - StitchWeld:         Stitch / intermittent welding

Welding power source communication:
  - EIPWeldingLink:     Ethernet/IP scanner/adapter for Fronius /
                        Lincoln / Miller / OTC welders

Typical usage
-------------
    from neurapy.robot import Robot
    from welding_package import WeldingController, WeldingProcess

    r = Robot("192.168.2.13")
    wc = WeldingController(r, weld_tool="torch_01")

    proc = WeldingProcess.linear_weave(
        start_pose=..., end_pose=...,
        speed=0.01, weave_amplitude=0.003, weave_freq=1.0,
    )
    wc.run(proc)
    r.stop()

Author : generated for Neura Robotics Neurapy v5.1.88
License: MIT
"""
from .welding_controller import WeldingController, WeldingMode
from .processes import (
    WeldingProcess,
    LinearWeaveWeld,
    CircularWeld,
    MultiPassWeld,
    SpotWeld,
    LaserWeld,
    CobotSeamTrack,
    StitchWeld,
)
from .eip import (
    EIPWeldingLink,
    EIPWeldingConfig,
    EIPWeldingTagMap,
    EIPJobMode,
    EIPBackedWeldingController,
    WeldStatusBit,
)

__all__ = [
    "WeldingController",
    "WeldingMode",
    "WeldingProcess",
    "LinearWeaveWeld",
    "CircularWeld",
    "MultiPassWeld",
    "SpotWeld",
    "LaserWeld",
    "CobotSeamTrack",
    "StitchWeld",
    "EIPWeldingLink",
    "EIPWeldingConfig",
    "EIPWeldingTagMap",
    "EIPJobMode",
    "EIPBackedWeldingController",
    "WeldStatusBit",
]

__version__ = "1.0.0"
