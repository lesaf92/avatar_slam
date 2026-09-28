"""4-DoF pose algebra and frame conventions.

A 4-DoF pose is a NumPy array ``[x, y, z, yaw]`` (metres, radians) describing
a gravity-aligned frame (docs/conventions.md §4). ``T_a_from_b`` maps points
expressed in frame ``b`` into frame ``a``.

All functions accept single poses of shape ``(4,)``; point functions accept
``(3,)`` or batched ``(n, 3)`` arrays.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]


def wrap_angle(angle: ArrayLike) -> FloatArray | float:
    """Wrap angle(s) to ``(-pi, pi]``."""
    wrapped = np.arctan2(np.sin(angle), np.cos(angle))
    # arctan2 returns exactly -pi for (sin=-0, cos=-1); map it to +pi (half-open interval).
    wrapped = np.where(wrapped == -np.pi, np.pi, wrapped)
    if np.ndim(wrapped) == 0:
        return float(wrapped)
    return wrapped


def rot_z(yaw: float) -> FloatArray:
    """Rotation matrix about +z by ``yaw`` radians."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def compose(a: ArrayLike, b: ArrayLike) -> FloatArray:
    """Return ``a ∘ b`` (apply ``b`` first, then ``a``)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    p = a[:3] + rot_z(a[3]) @ b[:3]
    return np.array([p[0], p[1], p[2], wrap_angle(a[3] + b[3])])


def inverse(a: ArrayLike) -> FloatArray:
    """Return the inverse transform ``a⁻¹``."""
    a = np.asarray(a, dtype=float)
    p = -(rot_z(-a[3]) @ a[:3])
    return np.array([p[0], p[1], p[2], wrap_angle(-a[3])])


def between(a: ArrayLike, b: ArrayLike) -> FloatArray:
    """Return ``a⁻¹ ∘ b``, i.e. pose ``b`` expressed in frame ``a``."""
    return compose(inverse(a), b)


def transform_points(T: ArrayLike, points: ArrayLike) -> FloatArray:
    """Map point(s) from frame ``b`` to frame ``a`` given ``T = T_a_from_b``."""
    T = np.asarray(T, dtype=float)
    pts = np.asarray(points, dtype=float)
    return pts @ rot_z(T[3]).T + T[:3]


def inverse_transform_points(T: ArrayLike, points: ArrayLike) -> FloatArray:
    """Map point(s) from frame ``a`` to frame ``b`` given ``T = T_a_from_b``."""
    T = np.asarray(T, dtype=float)
    pts = np.asarray(points, dtype=float)
    return (pts - T[:3]) @ rot_z(T[3])


def transform_poses(T: ArrayLike, poses: ArrayLike) -> FloatArray:
    """Left-multiply a batch of poses ``(n, 4)`` by ``T``."""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    out = np.empty_like(poses)
    out[:, :3] = transform_points(T, poses[:, :3])
    out[:, 3] = wrap_angle(poses[:, 3] + float(np.asarray(T)[3]))
    return out


def align_4dof(
    src: ArrayLike,
    dst: ArrayLike,
    weights: ArrayLike | None = None,
    estimate_z: bool = True,
) -> FloatArray:
    """Least-squares 4-DoF alignment ``T`` such that ``dst ≈ T ∘ src``.

    Closed-form yaw-only Umeyama/Kabsch (no scale). With ``estimate_z=False``
    the vertical offset is fixed to zero (both sets share the waterline datum).

    Parameters
    ----------
    src, dst
        Corresponding points, shape ``(n, 3)`` with ``n >= 2`` (or ``n >= 1``
        if the result only needs translation).
    weights
        Optional non-negative weights, shape ``(n,)``.
    """
    src = np.atleast_2d(np.asarray(src, dtype=float))
    dst = np.atleast_2d(np.asarray(dst, dtype=float))
    if src.shape != dst.shape or src.shape[1] != 3:
        raise ValueError("src and dst must both have shape (n, 3)")
    w = np.ones(len(src)) if weights is None else np.asarray(weights, dtype=float)
    if np.any(w < 0) or w.sum() <= 0:
        raise ValueError("weights must be non-negative with a positive sum")
    w = w / w.sum()
    c_src = w @ src
    c_dst = w @ dst
    a = src - c_src
    b = dst - c_dst
    sin_sum = np.sum(w * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    cos_sum = np.sum(w * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    yaw = float(np.arctan2(sin_sum, cos_sum)) if (sin_sum or cos_sum) else 0.0
    t = c_dst - rot_z(yaw) @ c_src
    if not estimate_z:
        t[2] = 0.0
    return np.array([t[0], t[1], t[2], wrap_angle(yaw)])


# --- Legacy-frame conversions (driver boundary only, conventions §3) -------------------


def ned_to_enu(v: ArrayLike) -> FloatArray:
    """Convert vector(s) from NED to ENU: ``(x_e, y_e, z_e) = (y_n, x_n, -z_n)``."""
    v = np.asarray(v, dtype=float)
    return np.stack([v[..., 1], v[..., 0], -v[..., 2]], axis=-1)


def enu_to_ned(v: ArrayLike) -> FloatArray:
    """Convert vector(s) from ENU to NED (self-inverse of :func:`ned_to_enu`)."""
    return ned_to_enu(v)


def frd_to_flu(v: ArrayLike) -> FloatArray:
    """Convert body vector(s) from FRD to FLU: ``(x, -y, -z)``."""
    v = np.asarray(v, dtype=float)
    return np.stack([v[..., 0], -v[..., 1], -v[..., 2]], axis=-1)


def flu_to_frd(v: ArrayLike) -> FloatArray:
    """Convert body vector(s) from FLU to FRD (self-inverse)."""
    return frd_to_flu(v)


def yaw_ned_to_enu(yaw_ned: ArrayLike) -> FloatArray | float:
    """Convert a heading measured clockwise from North into ENU yaw (CCW from East)."""
    return wrap_angle(np.pi / 2 - np.asarray(yaw_ned, dtype=float))
