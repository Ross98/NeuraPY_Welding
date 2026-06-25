"""
Interpolation helpers for welding trajectories.

封装 scipy 依赖，未安装 scipy 时给出清晰的安装提示。
"""
from __future__ import annotations

import math
from typing import List, Sequence

try:
    import numpy as np
    if np is None:  # pragma: no cover
        raise ImportError("numpy 被设置为 None")
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "welding_package 需要 numpy. 请运行: pip install numpy"
    ) from e

try:
    from scipy.spatial.transform import Slerp, Rotation
    _HAS_SCIPY = True
except ImportError:
    Slerp = None      # type: ignore
    Rotation = None   # type: ignore
    _HAS_SCIPY = False


def require_scipy() -> None:
    """Raise a clear error if scipy is missing."""
    if not _HAS_SCIPY:
        raise ImportError(
            "此工艺需要 scipy (插值/姿态 slerp). "
            "请运行: pip install scipy"
        )


def interpolate_linear(start: Sequence[float], end: Sequence[float],
                       step: float = 0.005) -> List[List[float]]:
    """
    在 start / end 之间按 step (m) 距离插值，返回路点列表。
    姿态 slerp 插值；若 scipy 不可用则退化为线性插值 (RPY 各分量)。
    """
    s = np.asarray(start, dtype=float)
    e = np.asarray(end, dtype=float)
    ds = float(np.linalg.norm(e[:3] - s[:3]))
    if ds < 1e-9:
        return [list(s), list(e)]
    n = max(int(math.ceil(ds / step)), 2)

    pos = np.linspace(s[:3], e[:3], n)

    if _HAS_SCIPY:
        rot_s = Rotation.from_euler("xyz", s[3:])
        rot_e = Rotation.from_euler("xyz", e[3:])
        slerp = Slerp([0, 1], Rotation.concatenate([rot_s, rot_e]))
        rpys = slerp(np.linspace(0, 1, n)).as_euler("xyz")
    else:
        # 退化: 线性插值 RPY (短距离精度可接受)
        rpys = np.linspace(s[3:], e[3:], n)

    return [list(np.concatenate([pos[i], rpys[i]])) for i in range(n)]


def cumdist(points: List[Sequence[float]]) -> List[float]:
    """沿点列累积距离 (m)."""
    d = [0.0]
    for i in range(1, len(points)):
        d.append(d[-1] + float(np.linalg.norm(
            np.asarray(points[i][:3]) - np.asarray(points[i - 1][:3])
        )))
    return d


def point_at(points: List[Sequence[float]],
             distances: List[float], target_d: float) -> List[float]:
    """返回路径上累计距离为 target_d 的点 (线性插值)."""
    for i in range(len(distances) - 1):
        if distances[i] <= target_d <= distances[i + 1]:
            span = distances[i + 1] - distances[i]
            if span < 1e-9:
                return list(points[i])
            t = (target_d - distances[i]) / span
            a, b = points[i], points[i + 1]
            return [a[k] + t * (b[k] - a[k]) for k in range(6)]
    return list(points[-1])
