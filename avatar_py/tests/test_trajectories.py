"""Trajectory library, per-agent overrides, turn-dependent odometry, surfacing."""

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.cli import _parse_value, _scenario_kwargs
from avatar.runner import make_sim, run_decentralized
from avatar.sim.agents import AgentConfig, OdometryNoise, sample_trajectory
from avatar.sim.scenarios import harbor_fleet
from avatar.sim.trajectories import TRAJECTORY_LIBRARY, TrajectorySpec
from avatar.types import Domain, LinkType


@pytest.mark.parametrize("kind", sorted(set(TRAJECTORY_LIBRARY) - {"csv"}))
def test_every_kind_builds_a_followable_path(kind):
    spec = TrajectorySpec(kind, start=(5.0, -3.0), z=-2.0, speed_mps=0.7)
    wp = spec.waypoints()
    assert wp.ndim == 2 and wp.shape[1] == 3 and len(wp) >= 2
    assert np.allclose(wp[0, :2], [5.0, -3.0])  # starts where asked
    cfg = AgentConfig(1, "uuv_0", Domain.UNDERWATER, wp, spec.speed_mps, sensors=(), comm=())
    tr = sample_trajectory(cfg, np.arange(0.0, 60.0))
    assert np.all(np.isfinite(tr))
    step = np.linalg.norm(np.diff(tr[:, :3], axis=0), axis=1)
    assert np.all(step <= spec.speed_mps + 1e-6)  # never faster than commanded


def test_heading_rotates_the_pattern_and_heights_are_respected():
    base = TrajectorySpec("line", start=(0.0, 0.0), z=3.0, params={"length": 10.0})
    rot = TrajectorySpec("line", start=(0.0, 0.0), z=3.0, heading_deg=90.0,
                         params={"length": 10.0})  # fmt: skip
    assert np.allclose(base.waypoints()[-1], [10.0, 0.0, 3.0])
    assert np.allclose(rot.waypoints()[-1], [0.0, 10.0, 3.0], atol=1e-9)
    helix = TrajectorySpec("helix", z=2.0, params={"z_top": 7.0, "turns": 2}).waypoints()
    assert helix[:, 2].min() == pytest.approx(2.0) and helix[:, 2].max() == pytest.approx(
        7.0, abs=0.05
    )
    yoyo = TrajectorySpec("yoyo", z=-8.0, params={"z_top": 0.0}).waypoints()
    assert yoyo[:, 2].min() == pytest.approx(-8.0) and yoyo[:, 2].max() == pytest.approx(
        0.0, abs=1e-6
    )


def test_overrides_and_validation():
    s = TrajectorySpec("rectangle", (1.0, 2.0), 0.0, params={"length": 5.0, "width": 3.0})
    o = s.with_overrides({"start": [7, 8], "width": 9.0})
    assert o.start == (7.0, 8.0) and o.params == {"length": 5.0, "width": 9.0}
    k = s.with_overrides({"kind": "circle", "radius": 4.0})
    assert k.kind == "circle" and k.params == {"radius": 4.0}  # new kind: own defaults
    with pytest.raises(ValueError):
        TrajectorySpec("teleport")


def test_csv_replay(tmp_path):
    f = tmp_path / "path.csv"
    f.write_text("x,y,z\n0,0,-1\n10,0,-2\n10,5,-3\n")
    wp = TrajectorySpec("csv", start=(1.0, 1.0), z=-4.0, params={"file": str(f)}).waypoints()
    assert np.allclose(wp, [[1, 1, -4], [11, 1, -5], [11, 6, -6]])


def test_harbor_fleet_path_overrides_and_unknown_agent():
    sc = harbor_fleet(np.random.default_rng(0), paths={
        "uuv_0": {"kind": "figure8", "start": [30, 0], "z": -6.0, "length": 30},
        "uuv_1": {"comm": ["acoustic", "rf"]},
    })  # fmt: skip
    uuv0 = next(a for a in sc.agents if a.name == "uuv_0")
    uuv1 = next(a for a in sc.agents if a.name == "uuv_1")
    assert uuv0.extra["trajectory"].kind == "figure8"
    assert np.allclose(uuv0.waypoints[0], [30, 0, -6.0])
    assert set(uuv1.comm) == {LinkType.ACOUSTIC, LinkType.RF}
    with pytest.raises(ValueError):
        harbor_fleet(np.random.default_rng(0), paths={"uuv_9": {"kind": "circle"}})


def test_cli_nested_scenario_args():
    from types import SimpleNamespace

    args = SimpleNamespace(
        scenario_arg=["paths.uuv_0.kind=circle", "paths.uuv_0.start=3,4", "n_uuv=3"]
    )
    kw = _scenario_kwargs(args)
    assert kw == {"paths": {"uuv_0": {"kind": "circle", "start": [3, 4]}}, "n_uuv": 3}
    assert _parse_value("true") is True and _parse_value("0.5") == 0.5


def test_turns_cost_heading_accuracy():
    n = OdometryNoise(sigma_yaw_per_rad=0.02)
    assert n.sigmas(1.0, turn_rad=np.pi / 2)[3] > 5 * n.sigmas(1.0, turn_rad=0.0)[3]


def test_surfacing_window_uses_rf_only_at_the_surface():
    params = AvatarParams()
    paths = {
        "uuv_0": {"kind": "yoyo", "start": [2, 0], "z": -8.0, "z_top": 0.0, "length": 58,
                  "period": 40, "comm": ["acoustic", "rf"]},
    }  # fmt: skip
    sc, sim = make_sim("harbor_fleet", 0, 200.0, params, paths=paths)
    res = run_decentralized(sc, sim, params, 0)
    uuv0 = next(a.agent_id for a in sc.agents if a.name == "uuv_0")
    rf_tx = [e for e in res.comm_events if e["from"] == uuv0 and e["link"] == "RF"]
    assert rf_tx, "the surfaced BlueROV2 should transmit over RF"
    times = sim.agents[uuv0].times
    for e in rf_tx:
        k = int(np.searchsorted(times, e["t"], side="right") - 1)
        assert sim.agents[uuv0].gt[k, 2] >= -0.3  # only while surfaced
