"""Static world model: structures, landmark parts, and the waterline datum.

A :class:`Structure` is a physical object with a horizontal footprint and a
vertical span ``[z_min, z_max]`` in the world ENU frame, where ``z = 0`` is the
waterline (docs/conventions.md §2). Sensors never observe a structure as a
whole. They observe its **parts**: the portion above the water (optical
sensors) and the portion below (acoustic sensors). A pier pile that crosses
the waterline therefore yields two :class:`LandmarkPart` objects that share
``(x, y)`` but have different vertical centroids. This is the *coaxial*
model that Avatar SLAM exploits for cross-medium association.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from avatar.semantics import class_id
from avatar.types import Medium


@dataclass(frozen=True)
class Structure:
    """A static object in the world (ground truth)."""

    object_id: int
    class_name: str
    center_xy: tuple[float, float]
    z_min: float
    z_max: float
    footprint: tuple[float, float]  # extent along world x and y [m]

    def __post_init__(self) -> None:
        if self.z_max <= self.z_min:
            raise ValueError(f"structure {self.object_id}: z_max must exceed z_min")
        class_id(self.class_name)  # validates the name

    @property
    def spans_waterline(self) -> bool:
        """True if the structure crosses the waterline (has both parts)."""
        return self.z_min < 0.0 < self.z_max

    def part(self, medium: Medium) -> LandmarkPart | None:
        """Return the part of this structure in ``medium`` (or ``None``)."""
        if medium == Medium.ABOVE:
            lo, hi = max(self.z_min, 0.0), self.z_max
        else:
            lo, hi = self.z_min, min(self.z_max, 0.0)
        if hi <= lo:
            return None
        return LandmarkPart(
            object_id=self.object_id,
            medium=medium,
            class_name=self.class_name,
            position=np.array([self.center_xy[0], self.center_xy[1], 0.5 * (lo + hi)]),
            extent=np.array([self.footprint[0], self.footprint[1], hi - lo]),
        )


@dataclass(frozen=True, eq=False)
class LandmarkPart:
    """The portion of a structure visible in one medium (ground truth)."""

    object_id: int
    medium: Medium
    class_name: str
    position: NDArray[np.float64]  # centroid of the part, world ENU [m]
    extent: NDArray[np.float64]  # (footprint x, footprint y, part height) [m]


@dataclass
class World:
    """Scenario world: structures + simple terrain description.

    Land occupies ``x < shoreline_x``, water ``x >= shoreline_x``.
    """

    structures: list[Structure]
    shoreline_x: float = 0.0
    land_z: float = 1.5
    seabed_z: float = -12.0
    bounds_xy: tuple[float, float, float, float] = (-60.0, 120.0, -80.0, 80.0)
    parts: list[LandmarkPart] = field(init=False)
    part_positions: NDArray[np.float64] = field(init=False)
    part_media: NDArray[np.int64] = field(init=False)

    def __post_init__(self) -> None:
        ids = [s.object_id for s in self.structures]
        if len(set(ids)) != len(ids):
            raise ValueError("structure object_ids must be unique")
        parts: list[LandmarkPart] = []
        for s in self.structures:
            for medium in (Medium.ABOVE, Medium.BELOW):
                p = s.part(medium)
                if p is not None:
                    parts.append(p)
        self.parts = parts
        self.part_positions = np.array([p.position for p in parts]) if parts else np.zeros((0, 3))
        self.part_media = np.array([int(p.medium) for p in parts], dtype=np.int64)

    def is_water(self, x: float, y: float) -> bool:
        """True if the horizontal location is over water (``y`` unused in v0 layouts)."""
        return x >= self.shoreline_x

    def structure(self, object_id: int) -> Structure:
        """Look up a structure by id."""
        for s in self.structures:
            if s.object_id == object_id:
                return s
        raise KeyError(object_id)
