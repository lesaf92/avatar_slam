"""Agent configurations and ground-truth trajectory generation."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import wrap_angle
from avatar.types import Domain, LinkType


@dataclass(frozen=True)
class OdometryNoise:
    """Odometry error model for 4-DoF keyframe increments.

    The per-increment standard deviation grows with the distance ``d`` travelled:
    ``σ = sqrt(σ0² + k² d)`` (random-walk drift). ``yaw_bias_std_rad_per_m`` draws
    a constant per-agent heading bias (rad per metre) that the estimator does
    **not** know about, which models unmodelled systematic drift.
    """

    sigma_xy_per_sqrt_m: float = 0.03
    sigma_z_per_sqrt_m: float = 0.01
    sigma_yaw_per_sqrt_m: float = 0.002
    sigma_xy_floor_m: float = 0.01
    sigma_z_floor_m: float = 0.005
    sigma_yaw_floor_rad: float = 0.001
    yaw_bias_std_rad_per_m: float = 5e-4

    def sigmas(self, distance_m: float) -> NDArray[np.float64]:
        """Nominal 1-σ of an increment ``[x, y, z, yaw]`` after ``distance_m``."""
        d = max(distance_m, 0.0)
        sxy = np.hypot(self.sigma_xy_floor_m, self.sigma_xy_per_sqrt_m * np.sqrt(d))
        sz = np.hypot(self.sigma_z_floor_m, self.sigma_z_per_sqrt_m * np.sqrt(d))
        syaw = np.hypot(self.sigma_yaw_floor_rad, self.sigma_yaw_per_sqrt_m * np.sqrt(d))
        return np.array([sxy, sxy, sz, syaw])


# Domain-typical defaults (order-of-magnitude, see docs/architecture.md §5).
ODOMETRY_DEFAULTS: dict[Domain, OdometryNoise] = {
    Domain.AERIAL: OdometryNoise(0.04, 0.02, 0.003, yaw_bias_std_rad_per_m=5e-4),  # VIO
    Domain.GROUND: OdometryNoise(0.03, 0.01, 0.002, yaw_bias_std_rad_per_m=5e-4),  # wheel+IMU
    Domain.SURFACE: OdometryNoise(0.04, 0.005, 0.003, yaw_bias_std_rad_per_m=1e-3),  # LIO/VIO
    Domain.UNDERWATER: OdometryNoise(0.02, 0.005, 0.002, yaw_bias_std_rad_per_m=1.5e-3),  # DVL-INS
}


@dataclass(frozen=True)
class AgentConfig:
    """Static description of one agent in a scenario.

    Attributes
    ----------
    sensors
        Names of sensor models attached (keys of ``avatar.sim.sensors.SENSOR_LIBRARY``).
    absolute_z
        Absolute vertical sensing: ``"depth"`` (pressure), ``"surface"`` (USV hull
        on the waterline), ``"baro"`` (barometric altitude) or ``None``.
    comm
        Link types this agent carries a radio/modem for.
    """

    agent_id: int
    name: str
    domain: Domain
    waypoints: NDArray[np.float64]  # (n, 3) world ENU
    speed_mps: float
    sensors: tuple[str, ...]
    comm: tuple[LinkType, ...]
    absolute_z: str | None = None
    loop: bool = True
    odometry: OdometryNoise | None = None
    extra: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0 <= self.agent_id <= 254:
            raise ValueError("agent_id must be in [0, 254]")
        wp = np.asarray(self.waypoints, dtype=float)
        if wp.ndim != 2 or wp.shape[1] != 3 or len(wp) < 2:
            raise ValueError("waypoints must have shape (n>=2, 3)")
        if self.speed_mps <= 0:
            raise ValueError("speed_mps must be positive")

    @property
    def odometry_noise(self) -> OdometryNoise:
        """Odometry model (explicit or the domain default)."""
        return self.odometry or ODOMETRY_DEFAULTS[self.domain]


def sample_trajectory(cfg: AgentConfig, times_s: NDArray[np.float64]) -> NDArray[np.float64]:
    """Ground-truth 4-DoF poses ``(n, 4)`` along the waypoint path at constant speed.

    Yaw follows the direction of horizontal motion (the previous heading is kept
    on purely vertical segments). If ``cfg.loop`` the path is closed and
    repeated; otherwise the agent stops at the last waypoint.
    """
    wp = np.asarray(cfg.waypoints, dtype=float)
    if cfg.loop:
        wp = np.vstack([wp, wp[:1]])
    seg = np.diff(wp, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    keep = seg_len > 1e-9
    wp_start, seg, seg_len = wp[:-1][keep], seg[keep], seg_len[keep]
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]

    heading = np.arctan2(seg[:, 1], seg[:, 0])
    horizontal = np.hypot(seg[:, 0], seg[:, 1]) > 1e-6
    for k in range(len(heading)):  # vertical segments keep the previous heading
        if not horizontal[k]:
            heading[k] = heading[k - 1] if k > 0 else 0.0

    s = np.asarray(times_s, dtype=float) * cfg.speed_mps
    s = np.mod(s, total) if cfg.loop else np.clip(s, 0.0, total - 1e-9)
    k = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(seg) - 1)
    frac = (s - cum[k]) / seg_len[k]
    pos = wp_start[k] + frac[:, None] * seg[k]
    yaw = wrap_angle(heading[k])
    return np.column_stack([pos, yaw])


# --- Waypoint pattern helpers ----------------------------------------------------------


def lawnmower(
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    spacing_m: float,
    z: float,
    along: str = "x",
) -> NDArray[np.float64]:
    """Boustrophedon survey pattern at constant ``z``."""
    (x0, x1), (y0, y1) = x_range, y_range
    pts = []
    if along == "x":
        ys = np.arange(y0, y1 + 1e-9, spacing_m)
        for i, y in enumerate(ys):
            xs = (x0, x1) if i % 2 == 0 else (x1, x0)
            pts += [(xs[0], y, z), (xs[1], y, z)]
    else:
        xs = np.arange(x0, x1 + 1e-9, spacing_m)
        for i, x in enumerate(xs):
            ys_ = (y0, y1) if i % 2 == 0 else (y1, y0)
            pts += [(x, ys_[0], z), (x, ys_[1], z)]
    return np.array(pts, dtype=float)


def rectangle(
    x_range: tuple[float, float], y_range: tuple[float, float], z: float
) -> NDArray[np.float64]:
    """Counter-clockwise rectangular loop at constant ``z``."""
    (x0, x1), (y0, y1) = x_range, y_range
    return np.array([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)], dtype=float)
