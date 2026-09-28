"""Integration tests on a short harbour mission (kept small for CI)."""

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_decentralized, run_independent

DURATION_S = 90.0


@pytest.fixture(scope="module")
def sim():
    params = AvatarParams()
    return (*make_sim("harbor", 0, DURATION_S, params), params)


def test_decentralized_team_is_connected_and_consistent(sim):
    scenario, data, params = sim
    res = run_decentralized(scenario, data, params, seed=0)
    m = res.metrics
    assert m["disconnected"] == []
    assert m["decode_errors"] == 0
    # The local-frame error also contains the (unobservable) heading drift an agent
    # accumulates between its origin and the first overlap, e.g. auv_1 starts 50 m
    # from the pier. Team ATE is the primary consistency metric (see docs/LOG.md).
    for err in m["frame_error"].values():
        assert err["xy_m"] < 1.5 and err["yaw_deg"] < 5.0
    assert m["ate_team_m"] < 0.3
    assert m["comm"]["ACOUSTIC"]["bytes_sent"] > 0 and m["comm"]["RF"]["bytes_sent"] > 0


def test_centralized_oracle_is_at_least_as_good_as_decentralized_team(sim):
    scenario, data, params = sim
    cen = run_centralized(scenario, data, params, seed=0).metrics
    dec = run_decentralized(scenario, data, params, seed=0).metrics
    assert cen["ate_team_m"] <= dec["ate_team_m"] + 0.02


def test_independent_baseline_runs(sim):
    scenario, data, params = sim
    m = run_independent(scenario, data, params, seed=0).metrics
    assert set(m["ate_local_m"]) == {a.agent_id for a in scenario.agents}
    assert all(np.isfinite(v) for v in m["ate_local_m"].values())


def test_no_ground_truth_leak_and_association_precision(sim):
    """Inter-agent pairs must be correct, and ids on the wire must be private."""
    from avatar.agent import AvatarAgent
    from avatar.comm import codec
    from avatar.types import LinkType

    scenario, data, params = sim
    obj_ids = np.array([p.object_id for p in data.world.parts])
    agents = {}
    for cfg in scenario.agents:
        ag = AvatarAgent(
            cfg,
            params,
            obj_ids,
            float(data.agents[cfg.agent_id].gt[0, 2]),
            np.random.default_rng(cfg.agent_id),
        )
        for kf in data.agents[cfg.agent_id].keyframes:
            ag.on_keyframe(kf)
        ag.solve_local()
        agents[cfg.agent_id] = ag
    # Digests are broadcasts: build each sender's packets once, deliver to everyone.
    packets = {i: ag.build_digests(0.0, LinkType.RF, 10**6, 1400) for i, ag in agents.items()}
    for rx in agents.values():
        for tx, pkts in packets.items():
            if tx != rx.id:
                for pkt in pkts:
                    rx.on_packet(pkt)
        rx.update_alignments()
    part_of = {
        ag.id: {lid: part for part, lid in ag._lid_of_part.items()} for ag in agents.values()
    }
    n_pairs = n_correct = 0
    for ag in agents.values():
        for sender, pairs in ag.alignment_ids.items():
            for my_lid, rlid, _ in pairs:
                n_pairs += 1
                a = data.world.parts[part_of[ag.id][my_lid]].object_id
                b = data.world.parts[part_of[sender][rlid]].object_id
                n_correct += a == b
    assert n_pairs > 50
    assert n_correct / n_pairs >= 0.95
    # Private ids: the wire carries lids from a random mapping, not part/object indices.
    lids = [r.landmark_id for p in packets[0] for r in codec.decode(p).records]
    assert lids and not set(lids) <= set(range(len(data.world.parts)))
