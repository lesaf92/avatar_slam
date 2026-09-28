"""Library of prerecorded trajectory types and their placement in the world.

A trajectory is a polyline of waypoints ``(n, 3)`` in the world ENU frame that
an agent follows at constant speed (``avatar.sim.agents.sample_trajectory``;
yaw follows the direction of motion). Each type is generated in a local
*pattern frame*: it starts at the origin, grows along +x (``length``) and +y
(``width``), and is then rotated by ``heading_deg`` and translated to
``start``. Curved types are densely sampled (``step_m``), so heading changes
continuously. Polygonal types keep sharp corners.

======================  ===========================================================
kind                    shape (what it stresses)
======================  ===========================================================
``lawnmower``           boustrophedon survey, 180° turn pairs (coverage)
``rectangle``           4 × 90° corners, closed loop (loop closure)
``line``                out-and-back transect, 180° reversal (no loop)
``zigzag``              sharp alternating turns of ``2·atan(width/leg)`` (gyro scale)
``circle``              constant turn rate, closed loop
``figure8``             alternating turn direction, self-crossing (loop closure)
``spiral``              expanding Archimedean spiral, no revisits (drift)
``helix``               circle climbing/descending between ``z`` and ``z_top``
``yoyo``                straight legs with depth oscillating between ``z`` and
                        ``z_top`` (e.g. to the surface and back: surfacing windows)
``hold``                static node
``csv``                 replay of a recorded path (``file``: CSV with x,y,z columns)
======================  ===========================================================

``TrajectorySpec`` bundles a kind with its placement and speed so that
scenarios, CLI overrides (``--scenario-arg paths.uuv_0.kind=figure8``) and
YAML scenario files share one representation.
"""

from __future__ import annotations

import csv
from collections.abc import Callable
from dataclasses import dataclass, field, fields, replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def lawnmower(length=60.0, width=16.0, spacing=8.0, z=0.0, **_) -> FloatArray:
    ys = np.arange(0.0, width + 1e-9, spacing)
    pts = []
    for i, y in enumerate(ys):
        xs = (0.0, length) if i % 2 == 0 else (length, 0.0)
        pts += [(xs[0], y, z), (xs[1], y, z)]
    return np.asarray(pts, dtype=float)


def rectangle(length=60.0, width=40.0, z=0.0, **_) -> FloatArray:
    return np.array([(0, 0, z), (length, 0, z), (length, width, z), (0, width, z)], dtype=float)


def line(length=60.0, z=0.0, **_) -> FloatArray:
    return np.array([(0, 0, z), (length, 0, z)], dtype=float)


def zigzag(length=60.0, width=10.0, legs=8, z=0.0, **_) -> FloatArray:
    xs = np.linspace(0.0, length, int(legs) + 1)
    ys = np.where(np.arange(len(xs)) % 2 == 0, 0.0, width)
    return np.column_stack([xs, ys, np.full(len(xs), z)])


def circle(radius=10.0, z=0.0, step_m=1.0, **_) -> FloatArray:
    n = max(8, int(np.ceil(2 * np.pi * radius / step_m)))
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    # starts at the origin, centre at (0, radius): first motion along +x
    return np.column_stack([radius * np.sin(th), radius * (1 - np.cos(th)), np.full(n, z)])


def figure8(length=40.0, width=16.0, z=0.0, step_m=1.0, **_) -> FloatArray:
    # Bernoulli-like lemniscate x = a sin t, y = b sin t cos t, through the origin.
    a, b = 0.5 * length, width
    n = max(16, int(np.ceil(2 * np.pi * max(a, b) / step_m)))
    t = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([a * np.sin(t), b * np.sin(t) * np.cos(t), np.full(n, z)])


def spiral(turns=3.0, spacing=6.0, z=0.0, step_m=1.0, **_) -> FloatArray:
    # Archimedean spiral r = spacing * θ / 2π, starting at the origin.
    theta_max = 2 * np.pi * turns
    th, pts = 0.0, []
    while th <= theta_max:
        r = spacing * th / (2 * np.pi)
        pts.append((r * np.cos(th), r * np.sin(th), z))
        th += step_m / max(r, 1.0)
    return np.asarray(pts, dtype=float)


def helix(radius=10.0, z=0.0, z_top=5.0, turns=2.0, step_m=1.0, **_) -> FloatArray:
    n = max(16, int(np.ceil(2 * np.pi * radius * turns / step_m)))
    th = np.linspace(0.0, 2 * np.pi * turns, n)
    zz = z + (z_top - z) * 0.5 * (1 - np.cos(th / turns))  # up then down: loops cleanly
    return np.column_stack([radius * np.sin(th), radius * (1 - np.cos(th)), zz])


def yoyo(length=60.0, z=-6.0, z_top=0.0, period=20.0, step_m=1.0, **_) -> FloatArray:
    n = max(4, int(np.ceil(length / step_m)))
    x = np.linspace(0.0, length, n + 1)
    zz = z + (z_top - z) * 0.5 * (1 - np.cos(2 * np.pi * x / period))
    out = np.column_stack([x, np.zeros_like(x), zz])
    return np.vstack([out, out[::-1][1:]])  # out and back along the same line


def hold(z=0.0, **_) -> FloatArray:
    return np.array([(0.0, 0.0, z), (0.0, 0.0, z)])


def from_csv(file: str = "", z=None, **_) -> FloatArray:
    """Waypoints from a CSV with header ``x,y,z`` (metres, pattern frame)."""
    with Path(file).open(newline="") as f:
        rows = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in csv.DictReader(f)]
    pts = np.asarray(rows, dtype=float)
    if z is not None:
        pts[:, 2] += float(z) - pts[0, 2]
    return pts


TRAJECTORY_LIBRARY: dict[str, Callable[..., FloatArray]] = {
    "lawnmower": lawnmower,
    "rectangle": rectangle,
    "line": line,
    "zigzag": zigzag,
    "circle": circle,
    "figure8": figure8,
    "spiral": spiral,
    "helix": helix,
    "yoyo": yoyo,
    "hold": hold,
    "csv": from_csv,
}

CLOSED_KINDS = frozenset({"rectangle", "circle", "figure8", "helix", "hold"})


@dataclass(frozen=True)
class TrajectorySpec:
    """A trajectory type plus its placement.

    Attributes
    ----------
    kind
        Key of :data:`TRAJECTORY_LIBRARY`.
    start
        World ENU ``(x, y)`` [m] of the first waypoint.
    z
        Height [m] above the waterline datum (negative = depth).
    heading_deg
        Rotation of the pattern about +z (0 = pattern's +x points East).
    speed_mps
        Constant speed along the path.
    params
        Shape parameters of the kind (``length``, ``width``, ``radius``, …).
    loop
        Repeat the path (default: closed kinds loop; open kinds go back and forth
        because their waypoint list returns to the start, or stop at the end).
    """

    kind: str
    start: tuple[float, float] = (0.0, 0.0)
    z: float = 0.0
    heading_deg: float = 0.0
    speed_mps: float = 1.0
    params: dict = field(default_factory=dict)
    loop: bool = True

    def __post_init__(self) -> None:
        if self.kind not in TRAJECTORY_LIBRARY:
            choices = sorted(TRAJECTORY_LIBRARY)
            raise ValueError(f"unknown trajectory kind {self.kind!r}; choose from {choices}")
        if self.speed_mps <= 0:
            raise ValueError("speed_mps must be positive")

    def waypoints(self) -> FloatArray:
        """World ENU waypoints ``(n, 3)``."""
        local = TRAJECTORY_LIBRARY[self.kind](z=self.z, **self.params)
        c, s = np.cos(np.deg2rad(self.heading_deg)), np.sin(np.deg2rad(self.heading_deg))
        R = np.array([[c, -s], [s, c]])
        xy = local[:, :2] @ R.T + np.asarray(self.start, dtype=float)
        wp = np.column_stack([xy, local[:, 2]])
        if len(wp) < 2:
            wp = np.vstack([wp, wp])
        return wp

    def with_overrides(self, overrides: dict | None) -> TrajectorySpec:
        """Copy with fields/params replaced; unknown keys go into ``params``."""
        if not overrides:
            return self
        names = {f.name for f in fields(self)} - {"params"}
        direct = {k: v for k, v in overrides.items() if k in names}
        extra = {k: v for k, v in overrides.items() if k not in names and k != "params"}
        params = {**self.params, **overrides.get("params", {}), **extra}
        if "start" in direct:
            direct["start"] = tuple(float(v) for v in direct["start"])
        if "kind" in direct and direct["kind"] != self.kind and "params" not in overrides:
            params = extra  # a new kind starts from its own defaults
        return replace(self, **direct, params=params)


def describe_library() -> str:
    """One line per kind with its default parameters (for ``avatar trajectories``)."""
    import inspect

    lines = []
    for name, fn in TRAJECTORY_LIBRARY.items():
        sig = inspect.signature(fn)
        defaults = ", ".join(
            f"{k}={p.default}"
            for k, p in sig.parameters.items()
            if p.default is not inspect.Parameter.empty and k not in ("z",)
        )
        lines.append(f"{name:<10} {defaults}")
    return "\n".join(lines)
