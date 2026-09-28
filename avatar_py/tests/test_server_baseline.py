"""A&B-style centralized server baseline (T-E2-05)."""

import dataclasses

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.baselines.centralized_server import (
    ABS_Z_B,
    DET_B,
    KF_BASE_B,
    ServerParams,
    compose_keyframes,
    keyframe_size_B,
    run_server,
)
from avatar.geometry import compose
from avatar.runner import make_sim, run_centralized
from avatar.types import LinkType


@pytest.fixture(scope="module")
def fleet():
    params = AvatarParams()
    sc, sim = make_sim("harbor_fleet", 0, 300.0, params)
    return params, sc, sim


def test_composed_keyframe_chains_odometry_and_keeps_last_detections(fleet):
    _, _, sim = fleet
    kfs = sim.agents[2].keyframes[5:15]
    ckf = compose_keyframes(kfs, descriptor_dim=0)
    want = np.zeros(4)
    for kf in kfs:
        want = compose(want, kf.odom)
    assert np.allclose(ckf.odom, want)
    s2 = sum(np.asarray(kf.odom_sigmas) ** 2 for kf in kfs)
    assert np.allclose(ckf.odom_sigmas, np.sqrt(s2))
    assert len(ckf.detections) == len(kfs[-1].detections)
    assert all(not np.any(d.descriptor) for d in ckf.detections)  # D = 0 on acoustic
    n = len(ckf.detections)
    z = ABS_Z_B if ckf.abs_z is not None else 0
    assert keyframe_size_B(ckf, 8) == KF_BASE_B + z + n * (DET_B + 8)


def test_ideal_links_match_the_oracle(fleet):
    params, sc, sim = fleet
    sc = dataclasses.replace(sc, channels=dict(sc.channels))
    ac = sc.channels[LinkType.ACOUSTIC]
    sc.channels[LinkType.ACOUSTIC] = dataclasses.replace(
        ac, bandwidth_bps=1e7, mtu_B=1400, loss_near=0.0, loss_far=0.0
    )
    m = run_server(sc, sim, params, 0, ServerParams(acoustic_stride=1)).metrics
    oracle = run_centralized(sc, sim, params, 0).metrics["ate_team_m"]
    assert not m["disconnected"]
    assert min(m["delivered_fraction"].values()) > 0.95
    assert m["ate_team_m"] < 1.3 * oracle + 0.01  # estimated association ≈ ground truth


def test_every_acoustic_byte_is_within_the_budget(fleet):
    params, sc, sim = fleet
    res = run_server(sc, sim, params, 0)
    ch = sc.channels[LinkType.ACOUSTIC]
    senders = [a for a in sc.agents if LinkType.ACOUSTIC in a.comm and a.role == "slam"]
    share = 1.0 / sum(LinkType.ACOUSTIC in a.comm for a in sc.agents)
    per_node = ch.budget_B(sim.duration_s, share) + max(
        4 * ch.budget_B(params.exchange_period_s, share), ch.mtu_B
    )
    assert res.metrics["comm"]["ACOUSTIC"]["bytes_sent"] <= len(senders) * per_node
    # raw keyframes do not fit 64 bps: the server lags far behind the AUVs
    uuvs = [a.agent_id for a in sc.agents if a.name.startswith("uuv")]
    assert max(res.metrics["delivered_fraction"][i] for i in uuvs) < 0.5
