import numpy as np
import pytest

from avatar.comm.channel import ACOUSTIC_DEFAULT, RF_DEFAULT
from avatar.comm.network import Network
from avatar.sim.agents import AgentConfig, rectangle, sample_trajectory
from avatar.sim.scenarios import harbor
from avatar.sim.sensors import SENSOR_LIBRARY, detect
from avatar.sim.world import Structure, World
from avatar.types import Domain, LinkType, Medium


def test_spanning_structure_has_two_coaxial_parts():
    s = Structure(0, "pile", (3.0, 4.0), -10.0, 2.0, (0.8, 0.8))
    up, down = s.part(Medium.ABOVE), s.part(Medium.BELOW)
    assert s.spans_waterline
    assert np.allclose(up.position, [3, 4, 1.0]) and np.allclose(down.position, [3, 4, -5.0])
    assert up.extent[2] == pytest.approx(2.0) and down.extent[2] == pytest.approx(10.0)
    land = Structure(1, "bollard", (0, 0), 1.5, 2.3, (0.4, 0.4))
    assert land.part(Medium.BELOW) is None


def _world():
    return World(
        [
            Structure(0, "pile", (10.0, 0.0), -10.0, 2.0, (0.8, 0.8)),
            Structure(1, "rock", (8.0, 3.0), -12.0, -10.0, (2.0, 2.0)),
            Structure(2, "container", (-10.0, 0.0), 1.5, 4.1, (6.0, 2.5)),
        ]
    )


def _detect(sensor_name, pose, world, rng):
    s = SENSOR_LIBRARY[sensor_name]
    s = type(s)(**{**s.__dict__, "detection_prob": 1.0})
    ext = np.array([p.extent for p in world.parts])
    return detect(
        s,
        np.asarray(pose, float),
        world.part_positions,
        world.part_media,
        ext,
        [p.class_name for p in world.parts],
        np.zeros((len(world.parts), 0)),
        rng,
    )


def test_medium_gating_of_sensors(rng):
    w = _world()
    lidar_at_surface = _detect("lidar", [0, 0, 0.0, 0.0], w, rng)
    assert {w.parts[d.part_index].medium for d in lidar_at_surface} == {Medium.ABOVE}
    assert _detect("lidar", [0, 0, -5.0, 0.0], w, rng) == []  # submerged LiDAR sees nothing
    sonar_uw = _detect("sonar", [0, 0, -5.0, 0.0], w, rng)
    assert {w.parts[d.part_index].medium for d in sonar_uw} == {Medium.BELOW}
    assert _detect("sonar", [0, 0, 10.0, 0.0], w, rng) == []  # sonar in air sees nothing


def test_trajectory_speed_and_heading():
    cfg = AgentConfig(
        0, "usv_0", Domain.SURFACE, rectangle((0, 20), (0, 10), 0.0), 2.0, sensors=(), comm=()
    )
    t = np.arange(0, 10, 0.5)
    tr = sample_trajectory(cfg, t)
    assert np.allclose(np.linalg.norm(np.diff(tr[:, :3], axis=0), axis=1), 1.0)
    assert np.allclose(tr[:, 3], 0.0)  # first 20 m leg heads east


def test_network_medium_gating_latency_and_share():
    pos = {0: np.array([0, 0, 0.0]), 1: np.array([150.0, 0, -6.0]), 2: np.array([100.0, 0, 15.0])}
    net = Network(
        channels={LinkType.RF: RF_DEFAULT, LinkType.ACOUSTIC: ACOUSTIC_DEFAULT},
        memberships={
            0: (LinkType.RF, LinkType.ACOUSTIC),
            1: (LinkType.ACOUSTIC,),
            2: (LinkType.RF,),
        },
        position_fn=lambda a, t: pos[a],
        rng=np.random.default_rng(0),
    )
    assert net.share(LinkType.ACOUSTIC) == pytest.approx(0.5)
    for _ in range(50):
        net.send(0.0, 0, LinkType.ACOUSTIC, bytes(100))
    deliveries = net.pop_until(1e6)
    assert all(d.receiver == 1 for d in deliveries)  # UAV has no modem
    ok = len(deliveries)
    assert 30 < ok <= 50
    first = min(deliveries, key=lambda d: d.t_arrival)
    airtime = 8 * 100 / (1000 * 0.5)
    dist = float(np.linalg.norm(pos[0] - pos[1]))
    assert first.t_arrival == pytest.approx(airtime + 0.2 + dist / 1500)
    net.send(0.0, 1, LinkType.ACOUSTIC, bytes(10))
    net.send(0.0, 0, LinkType.RF, bytes(10))
    rf = [d for d in net.pop_until(1e7) if d.link_type == LinkType.RF]
    assert {d.receiver for d in rf} <= {2}  # submerged AUV never receives RF
    with pytest.raises(ValueError):
        net.send(0.0, 1, LinkType.RF, bytes(10))


def test_harbor_scenario_is_deterministic():
    a = harbor(np.random.default_rng(3))
    b = harbor(np.random.default_rng(3))
    assert len(a.world.structures) == len(b.world.structures)
    assert np.allclose(a.world.part_positions, b.world.part_positions)
    spanning = [s for s in a.world.structures if s.spans_waterline]
    assert len(spanning) >= 30  # piles, hulls, buoys


def test_scenario_presets_build_in_both_tiers():
    """T-S4-01: every preset of experiments/scenarios builds a Tier-1 simulation and a Tier-2
    world, and fleet_default.yaml is the default reference fleet."""
    from pathlib import Path

    import yaml

    from avatar.agent import AvatarParams
    from avatar.runner import make_sim
    from avatar.sim.scenarios import harbor_fleet
    from avatar.tier2.sdf import rig_sensors, world_sdf

    presets = sorted((Path(__file__).parents[2] / "experiments" / "scenarios").glob("*.yaml"))
    assert len(presets) >= 5
    for f in presets:
        doc = yaml.safe_load(f.read_text())
        sc, sim = make_sim(doc["scenario"], 0, 20.0, AvatarParams(), **doc["args"])
        rigs = {n: s for n, s in rig_sensors(sc).items() if s}
        gt = {a.config.name: a.gt for a in sim.agents.values()}
        assert "<world" in world_sdf(sc, {n: gt[n][0] for n in rigs}, "w"), f.name
    doc = yaml.safe_load((presets[0].parent / "fleet_default.yaml").read_text())
    a, b = (
        harbor_fleet(np.random.default_rng(0), **doc["args"]),
        harbor_fleet(np.random.default_rng(0)),
    )
    assert [(x.name, x.sensors, x.comm) for x in a.agents] == [
        (x.name, x.sensors, x.comm) for x in b.agents
    ]
