"""Scenario definitions (Tier-1). v0 provides the *Harbour* scenario.

Harbour layout (world ENU, metres; land for x < 0, water for x >= 0):

* Pier A: two rows of free-standing piles (y ≈ ±4) from x ≈ 4 to 58, crossing
  the waterline (seabed to +1.5…+3.5 m).
* Pier B: one pile row at y ≈ 45.
* Three moored hulls crossing the waterline, and buoys in open water.
* Seabed-only objects: rocks, mooring blocks, a pipeline.
* Land-only objects: bollards along the quay, containers, crane legs, light
  poles, trees.

Team (default): ``usv_0`` (anchor; LiDAR + camera + sonar; RF + acoustic),
``auv_0`` / ``auv_1`` (sonar + turbid camera + depth; acoustic only),
``uav_0`` (aerial LiDAR + camera + baro; RF), ``ugv_0`` (LiDAR + camera; RF).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from avatar.comm.channel import ACOUSTIC_DEFAULT, RF_DEFAULT, ChannelModel
from avatar.sim.agents import AgentConfig, lawnmower, rectangle
from avatar.sim.world import Structure, World
from avatar.types import Domain, LinkType


@dataclass
class Scenario:
    """A world, a team, and link models."""

    name: str
    world: World
    agents: list[AgentConfig]
    channels: dict[LinkType, ChannelModel] = field(
        default_factory=lambda: {LinkType.RF: RF_DEFAULT, LinkType.ACOUSTIC: ACOUSTIC_DEFAULT}
    )
    anchor_id: int = 0

    def agent(self, agent_id: int) -> AgentConfig:
        for a in self.agents:
            if a.agent_id == agent_id:
                return a
        raise KeyError(agent_id)


def _place(rng, existing: list[tuple[float, float]], x_rng, y_rng, min_sep, tries=200):
    for _ in range(tries):
        p = (float(rng.uniform(*x_rng)), float(rng.uniform(*y_rng)))
        if all(np.hypot(p[0] - q[0], p[1] - q[1]) >= min_sep for q in existing):
            existing.append(p)
            return p
    raise RuntimeError("could not place object; relax constraints")


def harbor_world(rng: np.random.Generator) -> World:
    """Randomised harbour world (layout fixed, jitter and sizes random)."""
    s: list[Structure] = []
    placed: list[tuple[float, float]] = []

    def add(cls, xy, z_min, z_max, fp):
        s.append(Structure(len(s), cls, (float(xy[0]), float(xy[1])), z_min, z_max, fp))
        placed.append((float(xy[0]), float(xy[1])))

    seabed, land = -12.0, 1.5
    # Pier A: two rows of piles, irregular spacing/diameter/height (reduces aliasing).
    for row_y in (-4.0, 4.0):
        for x in np.arange(4.0, 58.1, 6.0):
            d = float(rng.uniform(0.4, 1.2))
            add(
                "pile",
                (x + rng.uniform(-0.8, 0.8), row_y + rng.uniform(-0.4, 0.4)),
                seabed,
                float(rng.uniform(1.5, 3.5)),
                (d, d),
            )
    # Pier B: single row.
    for x in np.arange(4.0, 40.1, 6.0):
        d = float(rng.uniform(0.4, 1.2))
        add(
            "pile",
            (x + rng.uniform(-0.8, 0.8), 45.0 + rng.uniform(-0.4, 0.4)),
            seabed,
            float(rng.uniform(1.5, 3.5)),
            (d, d),
        )
    # Moored hulls (cross the waterline).
    for xy in ((20.0, -12.0), (42.0, -12.0), (26.0, 30.0)):
        add(
            "hull",
            (xy[0] + rng.uniform(-1, 1), xy[1] + rng.uniform(-0.5, 0.5)),
            -float(rng.uniform(1.5, 2.5)),
            float(rng.uniform(2.5, 4.0)),
            (float(rng.uniform(10, 16)), float(rng.uniform(3.5, 5.0))),
        )
    # Buoys and their mooring blocks.
    for _ in range(6):
        xy = _place(rng, placed, (65, 110), (-60, 60), 8.0)
        add("buoy", xy, -1.0, float(rng.uniform(0.8, 1.5)), (1.5, 1.5))
        add(
            "mooring_block",
            (xy[0] + rng.uniform(-3, 3), xy[1] + rng.uniform(-3, 3)),
            seabed,
            seabed + 1.0,
            (2.0, 2.0),
        )
    # Seabed-only objects.
    for _ in range(14):
        xy = _place(rng, placed, (5, 115), (-70, 70), 5.0)
        w = float(rng.uniform(1.0, 3.0))
        add("rock", xy, seabed, seabed + float(rng.uniform(1.0, 2.5)), (w, w * rng.uniform(0.6, 1)))
    for x in np.arange(12.0, 100.0, 12.0):
        add("pipeline", (x, -35.0 + rng.uniform(-0.5, 0.5)), seabed, seabed + 0.6, (8.0, 0.8))
    # Land-only objects.
    for y in np.arange(-60.0, 60.1, 10.0):
        add(
            "bollard",
            (-1.5 + rng.uniform(-0.2, 0.2), y + rng.uniform(-1.5, 1.5)),
            land,
            land + 0.8,
            (0.4, 0.4),
        )
    for _ in range(10):
        xy = _place(rng, placed, (-55, -12), (-70, 70), 8.0)
        add("container", xy, land, land + 2.6, (6.0, 2.5))
    for xy in ((-8.0, -22.0), (-8.0, -14.0), (-16.0, -22.0), (-16.0, -14.0)):
        add("crane_leg", xy, land, land + 18.0, (1.0, 1.0))
    for _ in range(6):
        xy = _place(rng, placed, (-40, -4), (-70, 70), 6.0)
        add("light_pole", xy, land, land + 8.0, (0.3, 0.3))
    for _ in range(8):
        xy = _place(rng, placed, (-58, -30), (-75, 75), 6.0)
        add("tree", xy, land, land + float(rng.uniform(4, 8)), (3.0, 3.0))
    return World(s, shoreline_x=0.0, land_z=land, seabed_z=seabed)


def harbor(rng: np.random.Generator, n_auv: int = 2, include_air_ground: bool = True) -> Scenario:
    """Harbour scenario with the default heterogeneous team."""
    world = harbor_world(rng)
    RF, AC = LinkType.RF, LinkType.ACOUSTIC
    agents = [
        AgentConfig(
            0,
            "usv_0",
            Domain.SURFACE,
            rectangle((6, 66), (-20, 20), 0.0),
            1.5,
            sensors=("lidar", "camera", "sonar"),
            comm=(RF, AC),
            absolute_z="surface",
        ),
        AgentConfig(
            1,
            "auv_0",
            Domain.UNDERWATER,
            lawnmower((2, 64), (-14, 14), 7.0, -6.0),
            1.0,
            sensors=("sonar", "camera_underwater"),
            comm=(AC,),
            absolute_z="depth",
        ),
    ]
    if n_auv >= 2:
        agents.append(
            AgentConfig(
                2,
                "auv_1",
                Domain.UNDERWATER,
                lawnmower((40, 110), (-50, 6), 8.0, -8.0, along="y"),
                1.0,
                sensors=("sonar", "camera_underwater"),
                comm=(AC,),
                absolute_z="depth",
            )
        )
    if include_air_ground:
        agents += [
            AgentConfig(
                3,
                "uav_0",
                Domain.AERIAL,
                lawnmower((-40, 80), (-40, 50), 18.0, 15.0),
                3.0,
                sensors=("lidar_aerial", "camera"),
                comm=(RF,),
                absolute_z="baro",
            ),
            AgentConfig(
                4,
                "ugv_0",
                Domain.GROUND,
                rectangle((-24, -3), (-60, 60), world.land_z + 0.5),
                1.0,
                sensors=("lidar", "camera"),
                comm=(RF,),
            ),
        ]
    return Scenario("harbor", world, agents, anchor_id=0)


SCENARIOS = {"harbor": harbor}
