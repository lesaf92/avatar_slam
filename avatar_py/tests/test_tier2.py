"""Tier-2 (Gazebo) pipeline pieces that run without Gazebo: SDF, rays, front-end, dataset."""

from __future__ import annotations

import json
import os
import pickle
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.tier2.dataset import build_tier2_sim, frontend_sources
from avatar.tier2.frontend import FrontEndParams, Tracker, segment
from avatar.tier2.rays import body_to_world, depth_directions, scan_directions, sensor_points
from avatar.tier2.sdf import GZ_SENSORS, RaySensorSpec, bridge_yaml, rig_sensors, world_sdf
from avatar.types import Medium


def _cast_cylinder(dirs, origin, cx, cy, radius, z0, z1):
    """Analytic ranges of unit rays ``dirs (..., 3)`` from ``origin`` to a vertical cylinder."""
    d = dirs.reshape(-1, 3)
    ox, oy = origin[0] - cx, origin[1] - cy
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = 2 * (ox * d[:, 0] + oy * d[:, 1])
    c = ox**2 + oy**2 - radius**2
    disc = b**2 - 4 * a * c
    t = np.full(len(d), np.inf)
    ok = (disc >= 0) & (a > 1e-12)
    t_hit = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
    z = origin[2] + t_hit * d[ok, 2]
    good = (t_hit > 0) & (z >= z0) & (z <= z1)
    tt = np.full(ok.sum(), np.inf)
    tt[good] = t_hit[good]
    t[ok] = tt
    return t.reshape(dirs.shape[:-1])


def test_scan_directions_conventions():
    spec = RaySensorSpec("lidar", 5, 3, np.pi, np.deg2rad(20), 0.1, 10)
    d = scan_directions(spec)
    assert d.shape == (3, 5, 3)
    np.testing.assert_allclose(np.linalg.norm(d, axis=-1), 1.0)
    np.testing.assert_allclose(d[1, 2], [1, 0, 0], atol=1e-12)  # centre ray = boresight
    assert d[1, 0, 1] < 0 and d[1, 4, 1] > 0  # azimuth increases counter-clockwise
    assert d[0, 2, 2] < 0 and d[2, 2, 2] > 0  # rows go upwards


def test_depth_directions_and_pitch():
    spec = RaySensorSpec("depth", 4, 2, np.deg2rad(90), np.deg2rad(45), 0.1, 10, np.deg2rad(30))
    d = depth_directions(spec)
    assert d.shape == (2, 4, 3)
    assert d[0, 0, 1] > 0 and d[0, 3, 1] < 0  # left columns look left (+y)
    assert d[0, 0, 2] > 0 and d[1, 0, 2] < 0  # top rows look up
    # f = (W/2)/tan(hfov/2) = 2: outermost pixel centre at 1.5 px -> y = 0.75
    assert d[0, 0, 1] == pytest.approx(0.75)
    # A return straight along a boresight pitched 30° down: 2 m -> (1.732, 0, -1)
    one = RaySensorSpec("depth", 1, 1, np.deg2rad(60), np.deg2rad(60), 0.1, 10, np.deg2rad(30))
    p, valid = sensor_points(one, np.array([[2.0]]))
    assert valid.all()
    np.testing.assert_allclose(p[0], [2 * np.cos(np.pi / 6), 0.0, -2 * np.sin(np.pi / 6)])


def test_sensor_points_rejects_out_of_range_and_inf():
    spec = RaySensorSpec("lidar", 3, 1, np.pi / 2, 0.0, 0.5, 10.0)
    p, valid = sensor_points(spec, np.array([[np.inf, 0.2, 5.0]]))
    assert valid.tolist() == [[False, False, True]]
    np.testing.assert_allclose(np.linalg.norm(p, axis=1), [5.0])


def test_uav_lidar_variant_adds_a_lidar_rig_to_the_uav_only():
    base, _ = make_sim("harbor_fleet", 0, 2.0, AvatarParams())
    lidar, _ = make_sim("harbor_fleet", 0, 2.0, AvatarParams(), uav_lidar=True)
    assert rig_sensors(base)["uav_0"] == ["d435i_down30"]
    assert rig_sensors(lidar)["uav_0"] == ["d435i_down30", "vlp16"]
    assert {k: v for k, v in rig_sensors(lidar).items() if k != "uav_0"} == {
        k: v for k, v in rig_sensors(base).items() if k != "uav_0"
    }


def test_world_sdf_and_bridge_are_consistent():
    scenario, sim = make_sim("harbor_fleet", 0, 2.0, AvatarParams())
    poses = {a.config.name: a.gt[0] for a in sim.agents.values()}
    root = ET.fromstring(world_sdf(scenario, poses, "w"))
    models = {m.get("name") for m in root.iter("model")}
    assert sum(n.startswith("obj_") for n in models) == len(scenario.world.structures)
    rigs = rig_sensors(scenario)
    assert rigs["ugv_0"] == ["vlp16", "d435i"] and rigs["uuv_0"] == ["gemini_720s"]
    assert {"ugv_0", "uav_0", "uuv_0", "uuv_1", "land", "seabed"} <= models
    assert "gw_0" not in models  # sensorless gateway: no rig
    sensors = {s.get("name") for s in root.iter("sensor")}
    assert sensors == {"vlp16", "d435i", "d435i_down30", "gemini_720s"}
    yml = bridge_yaml(scenario, "w")
    assert "/avatar/uuv_1/gemini_720s" in yml and "/world/w/set_pose" in yml


def test_segment_finds_cylinder_centre_lidar():
    spec = GZ_SENSORS["vlp16"]
    dirs = scan_directions(spec)
    agent = np.array([0.0, 0.0, 2.0])
    ranges = _cast_cylinder(dirs, agent, 8.0, 3.0, 0.4, 0.0, 4.0)
    rng = np.random.default_rng(0)
    cl = segment(spec, ranges, 2.0, -12.0, FrontEndParams(), rng)
    assert len(cl) == 1
    np.testing.assert_allclose(cl[0].p_body[:2], [8.0, 3.0], atol=0.08)  # circle fit, not face
    assert not cl[0].one_sided
    assert cl[0].footprint[0] == pytest.approx(0.8, abs=0.2)


def test_optical_returns_below_waterline_are_removed():
    spec = GZ_SENSORS["vlp16"]
    dirs = scan_directions(spec)
    ranges = _cast_cylinder(dirs, np.array([0.0, 0.0, 0.5]), 6.0, 0.0, 0.4, -8.0, -0.5)
    assert np.isfinite(ranges).any()  # the rays do hit the submerged part...
    cl = segment(spec, ranges, 0.5, -12.0, FrontEndParams(), np.random.default_rng(0))
    assert cl == []  # ...but light does not cross the water surface


def test_sonar_cluster_ignores_elevation():
    spec = GZ_SENSORS["gemini_720s"]
    dirs = scan_directions(spec)
    ranges = _cast_cylinder(dirs, np.array([0.0, 0.0, -5.0]), 10.0, 1.0, 0.5, -12.0, 2.0)
    cl = segment(spec, ranges, -5.0, -12.0, FrontEndParams(), np.random.default_rng(1))
    assert len(cl) == 1
    assert cl[0].p_body[2] == 0.0  # fan centre: elevation is not measured
    # Horizontal range is slightly overestimated (slant range ≥ horizontal range).
    assert np.hypot(*cl[0].p_body[:2]) == pytest.approx(np.hypot(10.0, 1.0), abs=0.35)


def test_tracker_reuses_and_creates_tracks():
    tr = Tracker(1000, FrontEndParams())
    a, _ = tr.assign(0, Medium.ABOVE, np.array([5.0, 0.0, 1.0]), 0.1, set())
    b, _ = tr.assign(1, Medium.ABOVE, np.array([5.3, 0.1, 1.0]), 0.1, set())
    c, _ = tr.assign(1, Medium.ABOVE, np.array([15.0, 0.0, 1.0]), 0.1, set())
    d, _ = tr.assign(1, Medium.BELOW, np.array([5.0, 0.0, -3.0]), 0.1, set())
    assert a == b == 1000 and c == 1001 and d == 1002
    # One-to-one within a keyframe: a second detection cannot take the same track.
    taken: set[int] = set()
    e, _ = tr.assign(2, Medium.ABOVE, np.array([5.0, 0.0, 1.0]), 0.1, taken)
    f, _ = tr.assign(2, Medium.ABOVE, np.array([5.1, 0.0, 1.0]), 0.1, taken)
    assert e == 1000 and f not in (1000,)


def test_tracker_ambiguous_detection_is_dropped_not_duplicated():
    """Regression (docs/LOG.md L28): an ambiguous detection must not start a track.

    Starting one made the next detection between the pair ambiguous too, so a
    single crane leg collected 250 tracks on the Tier-2 harbour.
    """
    close = np.array([[5.0, 0.0, 1.0], [6.2, 0.0, 1.0]])  # two tracks 1.2 m apart
    jitter = np.random.default_rng(0).uniform(-0.4, 0.4, size=(40, 2))  # detections between
    counts = {}
    for mode in ("drop", "new"):
        tr = Tracker(0, FrontEndParams(track_on_ambiguity=mode))
        for k, p in enumerate(close):
            tr.assign(k, Medium.ABOVE, p, 0.1, set())
        for k, j in enumerate(jitter):
            tr.assign(2 + k, Medium.ABOVE, np.array([5.6 + j[0], j[1], 1.0]), 0.1, set())
        counts[mode] = (len(tr.tracks), tr.n_dropped)
    assert counts["drop"][0] <= 4 and counts["drop"][1] > 0
    assert counts["new"][0] > 3 * counts["drop"][0] and counts["new"][1] == 0
    tr = Tracker(0, FrontEndParams())
    for k, p in enumerate(close):
        tr.assign(k, Medium.ABOVE, p, 0.1, set())
    assert tr.assign(2, Medium.ABOVE, np.array([5.6, 0.0, 1.0]), 0.1, set()) is None  # a tie
    assert tr.assign(3, Medium.ABOVE, np.array([5.02, 0.0, 1.0]), 0.1, set()) is not None


def _tracker_with(points: np.ndarray, travelled_m: float) -> Tracker:
    """Registration tracker holding ``points``; ``travelled_m`` widens its search radius."""
    tr = Tracker(0, FrontEndParams(tracking="registration"))
    taken: set[int] = set()
    for p in points:
        tr.assign(0, Medium.ABOVE, p, 0.1, taken)
    tr.step(travelled_m)
    return tr


def test_registration_recovers_a_drift_offset():
    tracks = np.array([[5.0, 0.0, 1.0], [9.0, 3.0, 1.0], [14.0, -2.0, 1.0], [20.0, 6.0, 1.0]])
    tr = _tracker_with(tracks, travelled_m=100.0)  # search radius 1 + 0.02 * 100 = 3 m
    drift = np.array([1.4, -0.8, 0.0])
    off = tr.register([Medium.ABOVE] * 3, tracks[:3] + drift, np.full(3, 0.1))
    np.testing.assert_allclose(off, -drift[:2], atol=1e-9)
    np.testing.assert_allclose(tr.correction_xy, -drift[:2], atol=1e-9)
    assert tr.n_registrations == 1


def test_registration_refuses_a_tie_along_a_regular_row():
    """Two piles of a regular row match the row shifted by one spacing: no unique offset."""
    row = np.array([[6.0 * i, 0.0, 1.0] for i in range(5)])
    tr = _tracker_with(row, travelled_m=300.0)  # radius min(1 + 6, 8) m reaches the neighbours
    off = tr.register([Medium.ABOVE] * 2, row[1:3] + np.array([0.3, 0.0, 0.0]), np.full(2, 0.1))
    np.testing.assert_array_equal(off, np.zeros(2))
    assert tr.n_registrations == 0 and tr.n_ambiguous == 1


def test_tracker_coaxial_object_key():
    tr = Tracker(0, FrontEndParams())
    _, k1 = tr.assign(0, Medium.ABOVE, np.array([5.0, 0.0, 1.0]), 0.1, set())
    _, k2 = tr.assign(0, Medium.BELOW, np.array([5.2, 0.1, -3.0]), 0.1, set())
    assert k1 == k2


def test_build_tier2_sim_keeps_tier1_odometry(tmp_path):
    params = AvatarParams()
    scenario, sim = make_sim("harbor_fleet", 3, 4.0, params)
    names = {a.config.name: a for a in sim.agents.values()}
    rigs = {n: s for n, s in rig_sensors(scenario).items() if s}
    arrays = {}
    for n, ss in rigs.items():
        for s in ss:
            spec = GZ_SENSORS[s]
            arrays[f"{n}/{s}"] = np.full(
                (len(names[n].times), spec.v_samples, spec.h_samples), np.inf, np.float16
            )
    np.savez_compressed(tmp_path / "raw.npz", **arrays)
    meta = {"scenario": "harbor_fleet", "scenario_args": {}, "seed": 3, "duration_s": 4.0,
            "rigs": rigs}  # fmt: skip
    (tmp_path / "meta.json").write_text(json.dumps(meta))
    _, sim2, stats = build_tier2_sim(tmp_path, params)
    for i, ad in sim.agents.items():
        ad2 = sim2.agents[i]
        np.testing.assert_array_equal(ad.gt, ad2.gt)
        for kf, kf2 in zip(ad.keyframes, ad2.keyframes, strict=True):
            if kf.odom is not None:
                np.testing.assert_array_equal(kf.odom, kf2.odom)
            assert kf.abs_z == kf2.abs_z
            if ad.config.role == "slam":
                assert kf2.detections == []  # no returns -> no detections
    assert stats["ugv_0"]["clusters"] == 0
    cache = tmp_path / "detections_oracle.pkl"
    key0 = pickle.loads(cache.read_bytes())["key"]
    os.utime(tmp_path / "raw.npz", ns=(1, 1))  # a re-recording replaces raw.npz
    build_tier2_sim(tmp_path, params)
    assert pickle.loads(cache.read_bytes())["key"] != key0
    # a non-default front-end variant gets its own file and leaves the default one alone
    build_tier2_sim(tmp_path, params, FrontEndParams(ignore_sensors=("vlp16",)))
    assert len(list(tmp_path.glob("detections_oracle_*.pkl"))) == 1
    assert pickle.loads(cache.read_bytes())["key"] != key0


def test_detection_cache_key_covers_frontend_imports():
    # A front-end dependency outside the key would let a code change reuse stale detections.
    import ast
    from pathlib import Path

    from avatar.tier2 import frontend

    keyed = {p.resolve() for p in frontend_sources()}
    root = Path(frontend.__file__).resolve().parents[2]
    tree = ast.parse(Path(frontend.__file__).read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("avatar."):
            assert root / (node.module.replace(".", "/") + ".py") in keyed, node.module


def test_body_to_world():
    p = body_to_world(np.array([1.0, 2.0, 3.0, np.pi / 2]), np.array([[1.0, 0.0, 0.0]]))
    np.testing.assert_allclose(p, [[1.0, 3.0, 3.0]], atol=1e-12)


def test_ping360_sweeps_disjoint_sectors_that_cover_one_turn():
    """T-S1-11: with a 4 s turn and 1 s keyframes, keyframes 1-4 keep four disjoint quarters of
    the full-turn scan; together they are the whole turn, and keyframe 5 starts it again."""
    from avatar.tier2.frontend import swept_sector

    spec = GZ_SENSORS["ping360"]
    scan = np.full((spec.v_samples, spec.h_samples), 10.0)
    times = np.arange(6, dtype=float)
    kept = [np.isfinite(swept_sector(spec, scan, times, k, 4.0)[0]) for k in range(1, 6)]
    assert all(abs(m.sum() - spec.h_samples / 4) <= 2 for m in kept[:4])
    assert not np.any(kept[0] & kept[1]) and np.all(kept[0] | kept[1] | kept[2] | kept[3])
    np.testing.assert_array_equal(kept[4], kept[0])


def test_harbor_fleet_ping360_option():
    from avatar.sim.scenarios import harbor_fleet

    plain = {a.name: a.sensors for a in harbor_fleet(np.random.default_rng(0)).agents}
    with360 = {
        a.name: a.sensors for a in harbor_fleet(np.random.default_rng(0), uuv_ping360=True).agents
    }
    assert "ping360" not in plain["uuv_1"]
    assert with360["uuv_0"][-1] == with360["uuv_1"][-1] == "ping360"
    assert with360["ugv_0"] == plain["ugv_0"]
    assert {n for n, s in with360.items() if "ping360" in s} == {"uuv_0", "uuv_1"}


def test_sonar_world_renders_the_ping360_as_four_yawed_fans():
    from avatar.sim.scenarios import harbor_fleet
    from avatar.tier2.sdf import sonar_world_sdf

    sc = harbor_fleet(np.random.default_rng(0), uuv_ping360=True)
    poses = {a.name: np.zeros(4) for a in sc.agents}
    both = sonar_world_sdf(sc, poses, "w")
    only = sonar_world_sdf(sc, poses, "w", only=["ping360"])
    assert both.count('name="ping360_') == 8 and both.count('name="gemini_720s"') == 2
    assert only.count('name="ping360_') == 8 and "gemini_720s" not in only
    # one model per fan, turned by the fan's yaw (DAVE ignores a sensor's own pose)
    assert only.count('<model name="uuv_1__ping360_') == 4 and '<model name="uuv_1">' not in only
    assert '<model name="uuv_1">' in both
    assert only.count(" 0 0 1.5708</pose>") == 2  # the +90° fans of both BlueROV2s
