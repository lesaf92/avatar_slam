"""Range data → 3-D points (sensor, body, and world frames).

Conventions (checked against rendered Gazebo data, docs/LOG.md):

* ``gpu_lidar`` ranges are row-major ``(v, h)``; ray ``(j, i)`` has elevation
  ``v_min + j Δv`` and azimuth ``h_min + i Δh`` (both increasing, counter-
  clockwise from the boresight / upwards), ``Δ = fov / (n - 1)``.
* ``depth_camera`` images hold the depth along the optical axis (the sensor
  ``x`` axis); pixel ``(row, col)`` looks at ``y ∝ -(col - c_x)``,
  ``z ∝ -(row - c_y)``, with square pixels and ``f = (W / 2) / tan(hfov / 2)``.
* The sensor frame is FLU; a mount pitch ``θ > 0`` tilts the boresight down:
  ``p_body = R_y(θ) p_sensor``. The rig (body) frame is gravity-aligned
  (kinematic rigs, conventions §4), so ``p_world = R_z(yaw) p_body + t``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import rot_z
from avatar.tier2.sdf import RaySensorSpec

FloatArray = NDArray[np.float64]


def ray_angles(spec: RaySensorSpec) -> tuple[FloatArray, FloatArray]:
    """Azimuths ``(h,)`` and elevations ``(v,)`` [rad] of a ``gpu_lidar`` scan."""
    h = np.linspace(-0.5 * spec.h_fov_rad, 0.5 * spec.h_fov_rad, spec.h_samples)
    if spec.v_samples > 1:
        v = np.linspace(-0.5 * spec.v_fov_rad, 0.5 * spec.v_fov_rad, spec.v_samples)
    else:
        v = np.zeros(1)
    return h, v


def scan_directions(spec: RaySensorSpec) -> FloatArray:
    """Unit ray directions ``(v, h, 3)`` in the sensor frame."""
    h, v = ray_angles(spec)
    vv, hh = np.meshgrid(v, h, indexing="ij")
    return np.stack([np.cos(vv) * np.cos(hh), np.cos(vv) * np.sin(hh), np.sin(vv)], axis=-1)


def depth_directions(spec: RaySensorSpec) -> FloatArray:
    """Per-pixel vectors ``(rows, cols, 3)`` with unit ``x`` (multiply by depth)."""
    w, hgt = spec.h_samples, spec.v_samples
    f = 0.5 * w / np.tan(0.5 * spec.h_fov_rad)
    cols = np.arange(w) - 0.5 * (w - 1)
    rows = np.arange(hgt) - 0.5 * (hgt - 1)
    rr, cc = np.meshgrid(rows, cols, indexing="ij")
    return np.stack([np.ones_like(rr), -cc / f, -rr / f], axis=-1)


def rot_y(theta: float) -> FloatArray:
    """Rotation about +y by ``theta`` [rad] (positive pitches +x downwards)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def sensor_points(spec: RaySensorSpec, data: NDArray) -> tuple[FloatArray, NDArray[np.bool_]]:
    """Points ``(n, 3)`` in the **body** frame and the validity mask of the input grid.

    ``data`` is one scan ``(v, h)`` of ranges [m] or one depth image
    ``(rows, cols)`` [m]; non-finite values and values outside the sensor's
    range limits are invalid.
    """
    d = np.asarray(data, dtype=np.float64)
    valid = np.isfinite(d) & (d >= spec.min_range_m) & (d <= spec.max_range_m * 0.999)
    dirs = depth_directions(spec) if spec.kind == "depth" else scan_directions(spec)
    p_sensor = dirs[valid] * d[valid][:, None]
    return p_sensor @ rot_y(spec.pitch_rad).T, valid


def body_to_world(pose: FloatArray, p_body: FloatArray) -> FloatArray:
    """Map body-frame points to world ENU given the rig pose ``[x, y, z, yaw]``."""
    return p_body @ rot_z(float(pose[3])).T + np.asarray(pose[:3], dtype=float)
