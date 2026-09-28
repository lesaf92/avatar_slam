"""Reference fleet: sensor models, gateway relay, token buckets (ADR-0006)."""

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.comm import codec
from avatar.comm.channel import ACOUSTIC_M64, CHANNEL_PROFILES
from avatar.comm.gateway import Gateway
from avatar.runner import make_sim, run_decentralized
from avatar.semantics import CLASS_ID
from avatar.sim.agents import AgentConfig, sample_trajectory
from avatar.sim.sensors import SENSOR_LIBRARY, detect
from avatar.types import Domain, LandmarkFlags, LinkType


def _detect_one(sensor_name, pose, part_pos, part_ext, medium=0):
    s = SENSOR_LIBRARY[sensor_name]
    s = type(s)(**{**s.__dict__, "detection_prob": 1.0})
    return detect(
        s,
        np.asarray(pose, float),
        np.atleast_2d(part_pos).astype(float),
        np.array([medium]),
        np.atleast_2d(part_ext).astype(float),
        ["pile"],
        np.zeros((1, 0)),
        np.random.default_rng(0),
    )


def test_vlp16_vertical_fan_misses_close_low_objects():
    # Sensor 2.2 m up; a 0.8 m bollard part centred at z = 1.9 m.
    far = _detect_one("vlp16", [0, 0, 2.2, 0], [10.0, 0.0, 1.9], [0.4, 0.4, 0.8])
    near_low = _detect_one("vlp16", [0, 0, 2.2, 0], [1.2, 0.0, 0.2], [0.4, 0.4, 0.4])
    assert len(far) == 1 and near_low == []


def test_d435i_range_and_quadratic_noise():
    s = SENSOR_LIBRARY["d435i"]
    assert _detect_one("d435i", [0, 0, 1, 0], [7.0, 0.0, 1.0], [1, 1, 1]) == []
    assert s.sigmas(4.0)[0] > 2.5 * s.sigmas(2.0)[0] - 2 * s.sigma_h_base_m  # ∝ r²
    assert len(_detect_one("d435i", [0, 0, 1, 0], [3.0, 0.5, 1.0], [1, 1, 1])) == 1


def test_gemini_is_underwater_only_with_90_degree_fov():
    below = 1
    ahead = _detect_one("gemini_720s", [0, 0, -4, 0], [10.0, 0.0, -5.0], [1, 1, 8], below)
    side = _detect_one("gemini_720s", [0, 0, -4, 0], [0.0, 10.0, -5.0], [1, 1, 8], below)
    in_air = _detect_one("gemini_720s", [0, 0, 5, 0], [10.0, 0.0, -5.0], [1, 1, 8], below)
    assert len(ahead) == 1 and side == [] and in_air == []


def test_static_node_trajectory():
    cfg = AgentConfig(
        9, "gw_0", Domain.SURFACE, np.array([(0.5, 0.0, 0.0)] * 2), 1.0,
        sensors=(), comm=(LinkType.RF,), role="gateway",
    )  # fmt: skip
    tr = sample_trajectory(cfg, np.arange(0, 10.0))
    assert np.allclose(tr, [[0.5, 0.0, 0.0, 0.0]] * 10)
    with pytest.raises(ValueError):
        AgentConfig(
            9, "gw_1", Domain.SURFACE, np.zeros((2, 3)), 1.0,
            sensors=("vlp16",), comm=(), role="gateway",
        )  # fmt: skip


def _digest(sender, seq, records, dim=0):
    return codec.encode(codec.LandmarkDigest(sender, seq, 0, 1, dim, tuple(records)))


def _rec(lid, cls, flags, n_obs=5, dim=0):
    return codec.LandmarkRecord(
        lid, (float(lid), 0.0, 1.0), 0.1, 0.2, (0.8, 0.8, 2.0), cls, flags, n_obs, (0.5,) * dim
    )


def test_gateway_forwards_acoustic_to_rf_unchanged_and_dedups():
    gw = Gateway(4)
    pkt = _digest(2, 7, [_rec(1, 0, int(LandmarkFlags.BELOW | LandmarkFlags.SONAR))])
    gw.on_packet(LinkType.ACOUSTIC, pkt)
    gw.on_packet(LinkType.ACOUSTIC, pkt)  # duplicate
    out = gw.build_packets(0.0, LinkType.RF, 10**6, 1400)
    assert out == [pkt] and gw.stats["duplicates"] == 1
    assert gw.build_packets(0.0, LinkType.ACOUSTIC, 10**6, 64) == []  # never echoed back


def test_gateway_reencodes_rf_to_acoustic_with_priority_budget_and_origin():
    gw = Gateway(4)
    above = int(LandmarkFlags.ABOVE | LandmarkFlags.LIDAR)
    recs = [_rec(10, CLASS_ID["container"], above, n_obs=50, dim=8)]  # cannot cross
    recs += [_rec(20 + k, CLASS_ID["pile"], above, n_obs=3, dim=8) for k in range(5)]
    gw.on_packet(LinkType.RF, _digest(0, 1, recs, dim=8))
    fa = codec.encode(codec.FrameAlignment(0, 2, 0, 1, 9, 1, 2, 0, 0.1, 0.1, 0.1, 0.01))
    gw.on_packet(LinkType.RF, fa)
    budget = codec.digest_size(3, 0)  # room for exactly three records
    out = gw.build_packets(0.0, LinkType.ACOUSTIC, budget, ACOUSTIC_M64.mtu_B)
    assert sum(len(p) for p in out) <= budget
    msgs = [codec.decode(p) for p in out]
    assert all(isinstance(m, codec.LandmarkDigest) for m in msgs)  # no FRAME_ALIGNMENT
    assert all(m.sender_id == 0 and m.descriptor_dim == 0 for m in msgs)  # origin kept
    sent = [r.landmark_id for m in msgs for r in m.records]
    assert len(sent) == 3 and 10 not in sent  # crossing-capable piles first
    more = gw.build_packets(1.0, LinkType.ACOUSTIC, 10**6, ACOUSTIC_M64.mtu_B)
    rest = [r.landmark_id for p in more for r in codec.decode(p).records]
    assert sorted(sent + rest) == [10, 20, 21, 22, 23, 24]  # each forwarded once


@pytest.fixture(scope="module")
def fleet_run():
    params = AvatarParams()
    scenario, sim = make_sim("harbor_fleet", 0, 300.0, params)
    return scenario, run_decentralized(scenario, sim, params, 0).metrics


def test_fleet_acoustic_traffic_respects_modem_capacity(fleet_run):
    scenario, m = fleet_run
    ch = CHANNEL_PROFILES["m64"]
    nodes = [a for a in scenario.agents if LinkType.ACOUSTIC in a.comm]
    duration = 300.0
    per_node = ch.bandwidth_bps / len(nodes) * ch.utilization * duration / 8.0
    cap = max(4 * ch.budget_B(20.0, 1 / len(nodes)), ch.mtu_B)
    assert m["comm"]["ACOUSTIC"]["bytes_sent"] <= len(nodes) * (per_node + cap)
    assert m["comm"]["ACOUSTIC"]["bytes_sent"] > 0


def test_fleet_gateway_bridges_media(fleet_run):
    _, m = fleet_run
    (gw_stats,) = m["gateways"].values()
    assert gw_stats["forwarded_to_rf"] > 0 and gw_stats["records_to_acoustic"] > 0
    # Some air/ground agent aligned a BlueROV2 through the relay (cross-medium only)...
    uuvs = {i for i, n in m["agents"].items() if n.startswith("uuv")}
    air_ground = set(m["agents"]) - uuvs
    assert any(uuvs & set(m["alignments"][i]) for i in air_ground)
    # ...and the anchor's frame chain covers the whole team within the mission.
    assert m["team_connected_s"] is not None and m["disconnected"] == []
    assert m["decode_errors"] == 0
