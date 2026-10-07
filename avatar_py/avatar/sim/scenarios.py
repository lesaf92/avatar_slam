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

import dataclasses
from dataclasses import dataclass, field

import numpy as np

from avatar.comm.channel import ACOUSTIC_DEFAULT, CHANNEL_PROFILES, RF_DEFAULT, ChannelModel
from avatar.sim.agents import PLATFORM_ODOMETRY, AgentConfig, lawnmower, rectangle
from avatar.sim.measurements import FrontEndErrors
from avatar.sim.trajectories import TrajectorySpec
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
    frontend_errors: FrontEndErrors = field(default_factory=FrontEndErrors)

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


def fleet_default_paths(land_z: float = 1.5) -> dict[str, TrajectorySpec]:
    """Default trajectories of the reference fleet (``harbor_fleet``)."""
    T = TrajectorySpec
    return {
        "ugv_0": T("rectangle", (-14.0, -50.0), land_z + 0.7, 0.0, 1.0,
                   {"length": 11.0, "width": 105.0}),
        # 2.5 m stand-off around pier A's pile rows, 3 m up (D435i range ≈ 6 m).
        "uav_0": T("rectangle", (2.0, -6.5), 3.0, 0.0, 1.0, {"length": 58.0, "width": 13.0}),
        "uuv_0": T("lawnmower", (2.0, -8.0), -4.0, 0.0, 0.5,
                   {"length": 58.0, "width": 16.0, "spacing": 8.0}),
        "uuv_1": T("rectangle", (8.0, -18.0), -5.0, 0.0, 0.5, {"length": 42.0, "width": 56.0}),
        "uuv_2": T("lawnmower", (10.0, 38.0), -3.0, 0.0, 0.5,
                   {"length": 35.0, "width": 14.0, "spacing": 7.0}),
        "gw_0": T("hold", (0.5, 0.0), 0.0),
        "usv_0": T("rectangle", (6.0, -20.0), 0.0, 0.0, 1.0, {"length": 60.0, "width": 40.0}),
    }  # fmt: skip


def harbor_fleet(
    rng: np.random.Generator,
    n_uuv: int = 2,
    acoustic: str = "m64",
    rf: str = "wifi_mesh",
    with_usv: bool = False,
    paths: dict[str, dict] | None = None,
    frontend_errors: dict | None = None,
    acoustic_bps: float | None = None,
    uav_lidar: bool = False,
    uuv_ping360: bool = False,
    acoustic_loss: float | None = None,
) -> Scenario:
    """Harbour world with the PI's **reference fleet** (ADR-0006, docs/hardware.md).

    * ``ugv_0``: Clearpath Husky, VLP-16 + D435i (level), Wi-Fi mesh. **Anchor.**
      It patrols the quay (land), so it sees pile tops and bollards.
    * ``uav_0``: Tarot 680 hexacopter + Cube, D435i pitched 30° down (≤ 6 m
      useful depth), barometer, Wi-Fi mesh. It circles pier A's pile rows at a
      2.5 m stand-off, 3 m up (the D435i sees nothing useful beyond ~6 m).
      ``uav_lidar=True`` adds a VLP-16-class LiDAR (stand-in for the optional
      Livox-class unit of decision D6, docs/hardware.md), which sees many piles
      per keyframe (LOG L31).
    * ``uuv_k``: BlueROV2, Micron Gemini 720s imaging sonar + low-light camera +
      Bar30 depth, DVL A50 dead reckoning. SLAM traffic goes **only** over the
      acoustic modem (the tether is for safety/logging, never for SLAM data).
      ``uuv_ping360=True`` adds a Blue Robotics Ping360 360° scanning sonar to every BlueROV2
      (an option of decision D14, T-S1-11).
    * ``gw_0``: quay-side surface gateway (topside modem + Wi-Fi), sensorless relay.
    * optional ``usv_0``: BlueBoat with D435i above and a Gemini below (bridge).

    ``acoustic``/``rf`` select entries of ``avatar.comm.channel.CHANNEL_PROFILES``.
    ``paths`` overrides trajectories per agent name with any
    :class:`~avatar.sim.trajectories.TrajectorySpec` field or shape parameter,
    e.g. ``{"uuv_0": {"kind": "figure8", "start": [20, 0], "z": -6, "length": 30}}``
    (defaults: :func:`fleet_default_paths`). An AUV path that reaches
    ``z > -0.3 m`` gives the vehicle an RF window only if it also has ``"rf"`` in
    ``paths[name]["comm"]``. ``frontend_errors`` sets
    :class:`~avatar.sim.measurements.FrontEndErrors` (clutter, identity switches;
    off by default). ``acoustic_bps`` overrides the acoustic profile's raw bit
    rate (bandwidth sweeps, T-C5-01); everything else of the profile is kept.
    ``acoustic_loss`` replaces its range-dependent packet loss with a loss probability that is
    the same at every range within the modem's reach (loss sweeps, T-C5-01).
    """
    errors = FrontEndErrors(**(frontend_errors or {}))
    world = harbor_world(rng)
    RF, AC = LinkType.RF, LinkType.ACOUSTIC
    odo = PLATFORM_ODOMETRY
    paths = paths or {}
    unknown = set(paths) - {"ugv_0", "uav_0", "gw_0", "usv_0"} - {f"uuv_{k}" for k in range(n_uuv)}
    if unknown:
        raise ValueError(f"paths given for unknown agents: {sorted(unknown)}")
    defaults = fleet_default_paths(world.land_z)

    def spec(name: str) -> tuple[TrajectorySpec, tuple[LinkType, ...] | None]:
        over = dict(paths.get(name, {}))
        comm = over.pop("comm", None)
        base = defaults.get(name) or defaults[f"uuv_{int(name.split('_')[1]) % 3}"]
        links = None
        if comm is not None:
            links = tuple({"rf": RF, "acoustic": AC}[c.lower()] for c in comm)
        return base.with_overrides(over), links

    def agent(aid, name, domain, sensors, comm, **kw) -> AgentConfig:
        sp, links = spec(name)
        return AgentConfig(
            aid, name, domain, sp.waypoints(), sp.speed_mps, sensors=sensors,
            comm=links or comm, loop=sp.loop, extra={"trajectory": sp}, **kw,
        )  # fmt: skip

    agents = [
        agent(0, "ugv_0", Domain.GROUND, ("vlp16", "d435i"), (RF,), odometry=odo["husky_lio"]),
        agent(1, "uav_0", Domain.AERIAL, ("d435i_down30", *(("vlp16",) if uav_lidar else ())),
              (RF,), absolute_z="baro", odometry=odo["tarot_vio"]),
    ]  # fmt: skip
    for k in range(n_uuv):
        agents.append(
            agent(2 + k, f"uuv_{k}", Domain.UNDERWATER,
                  ("gemini_720s", "bluerov2_camera", *(("ping360",) if uuv_ping360 else ())),
                  (AC,), absolute_z="bar30", odometry=odo["bluerov2_dvl"])
        )  # fmt: skip
    gw_id = 2 + n_uuv
    agents.append(agent(gw_id, "gw_0", Domain.SURFACE, (), (RF, AC), role="gateway"))
    if with_usv:
        agents.append(
            agent(gw_id + 1, "usv_0", Domain.SURFACE, ("d435i", "gemini_720s"), (RF, AC),
                  absolute_z="surface", odometry=odo["blueboat_vio"])
        )  # fmt: skip
    channels = {RF: CHANNEL_PROFILES[rf], AC: CHANNEL_PROFILES[acoustic]}
    if acoustic_bps is not None:
        channels[AC] = dataclasses.replace(channels[AC], bandwidth_bps=float(acoustic_bps))
    if acoustic_loss is not None:
        p = float(acoustic_loss)
        channels[AC] = dataclasses.replace(channels[AC], loss_near=p, loss_far=p)
    return Scenario(
        "harbor_fleet", world, agents, channels=channels, anchor_id=0, frontend_errors=errors
    )


SCENARIOS = {"harbor": harbor, "harbor_fleet": harbor_fleet}
