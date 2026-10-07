"""Team frame-graph cycle consistency (T-X2-02)."""

import numpy as np
import pytest

from avatar.agent import AvatarParams
from avatar.frontend.frame_consistency import (
    CycleGate,
    FrameEdge,
    consistent_subset,
    cycle_error,
    is_consistent,
)
from avatar.geometry import compose, inverse
from avatar.runner import make_sim, run_decentralized

# Ground-truth world_from_agent frames of four agents [x, y, z, yaw].
WORLD = {
    0: np.array([0.0, 0.0, 0.0, 0.0]),
    1: np.array([30.0, -5.0, 3.0, 0.7]),
    2: np.array([10.0, 20.0, -4.0, -1.2]),
    3: np.array([-15.0, 8.0, -2.0, 2.5]),
}


def true_edge(a: int, b: int, n: int = 20, s: float = 0.1, sy: float = 0.005) -> FrameEdge:
    T = compose(inverse(WORLD[a]), WORLD[b])
    return FrameEdge(a, b, T, s, sy, n)


def test_true_cycles_close_exactly():
    path = [(true_edge(0, 1), True), (true_edge(2, 1), False)]  # 0 → 1 → 2
    dxy, dyaw = cycle_error(true_edge(0, 2), path)
    assert dxy == pytest.approx(0.0, abs=1e-9) and dyaw == pytest.approx(0.0, abs=1e-9)
    assert is_consistent(true_edge(0, 2), path)


def test_weak_wrong_edge_is_rejected_and_strong_ones_kept():
    bad_T = compose(true_edge(0, 2).T, np.array([6.0, -3.0, 0.0, 0.2]))  # aliasing offset
    bad = FrameEdge(0, 2, bad_T, 0.1, 0.005, 8)
    edges = [true_edge(0, 1, 30), true_edge(1, 2, 25), bad, true_edge(3, 0, 12)]
    accepted, rejected = consistent_subset(edges)
    assert rejected == [bad]
    assert len(accepted) == 3  # the dangling 3–0 edge has no cycle: kept


def test_parallel_estimates_form_a_two_cycle():
    good = true_edge(0, 1, 20)
    rev = true_edge(1, 0, 15)  # the other agent's own estimate, other direction
    wrong = FrameEdge(1, 0, compose(rev.T, np.array([0.0, 4.0, 0.0, 0.0])), 0.1, 0.005, 10)
    accepted, rejected = consistent_subset([good, rev, wrong])
    assert rejected == [wrong] and set(map(id, accepted)) == {id(good), id(rev)}


def test_gate_scales_with_uncertainty():
    # A 2 m discrepancy is rejected for precise edges, accepted for loose ones.
    off = compose(true_edge(0, 2).T, np.array([2.0, 0.0, 0.0, 0.0]))
    path = [(true_edge(0, 1), True), (true_edge(1, 2), True)]
    assert not is_consistent(FrameEdge(0, 2, off, 0.1, 0.001, 8), path)
    assert is_consistent(FrameEdge(0, 2, off, 1.0, 0.001, 8), path)
    assert is_consistent(FrameEdge(0, 2, off, 0.1, 0.001, 8), path, CycleGate(min_xy_m=2.5))


def test_ties_and_order_do_not_change_the_result():
    bad = FrameEdge(0, 2, compose(true_edge(0, 2).T, np.array([5.0, 5.0, 0, 0])), 0.1, 0.005, 8)
    edges = [true_edge(0, 1, 30), true_edge(1, 2, 25), bad]
    rng = np.random.default_rng(0)
    for _ in range(5):
        perm = [edges[i] for i in rng.permutation(3)]
        assert consistent_subset(perm)[1] == [bad]


@pytest.mark.parametrize("reverse", ["disagrees", "missing", "agrees"])
def test_veto_drops_a_weak_estimate_only_when_its_reverse_disagrees(reverse):
    # T-F3-05 / D13 default: a weak (< 8 inliers) estimate whose reverse direction exists and
    # disagrees is used in neither direction; without a reverse it is used (LOG L37, L39).
    from avatar.agent import AvatarAgent, FrameEstimate
    from avatar.frontend.association import Alignment

    params = AvatarParams()
    assert (params.confirm_weak_inliers, params.confirm_weak_policy) == (8, "veto")
    sc, _ = make_sim("harbor_fleet", 0, 4.0, params)
    agent = AvatarAgent(sc.agents[0], params, np.zeros(0, np.int64), 0.0, np.random.default_rng(0))
    flipped = compose(true_edge(0, 1).T, np.array([0.0, 0.0, 0.0, np.pi]))  # the L33 180° flip
    pairs = tuple((i, i, False) for i in range(5))
    agent.alignments = {1: Alignment(flipped, pairs, 0.1, 0.1, 0.1, 0.005)}
    if reverse != "missing":
        T = true_edge(1, 0).T if reverse == "disagrees" else inverse(flipped)
        agent.frames_about_me = {1: FrameEstimate(T, 0.1, 0.1, 0.005, 5, 1)}
    agent.check_cycles()
    own_dropped = 1 in agent.vetoed | agent.unconfirmed
    assert own_dropped == (reverse == "disagrees")
    assert (1 in agent.rejected_about_me) == (reverse == "disagrees")


def test_cycle_check_keeps_the_fleet_run_intact():
    # No wrong alignments in this run: the check must not veto correct ones.
    params = AvatarParams()
    sc, sim = make_sim("harbor_fleet", 0, 300.0, params)
    res = run_decentralized(sc, sim, params, 0)
    assert res.metrics["team_connected_s"] is not None
    assert res.metrics["vetoed_alignments"] == 0


def test_frame_graph_fusion_is_exact_on_exact_edges_and_averages_noisy_ones():
    from avatar.eval.metrics import chain_frames
    from avatar.frontend.frame_consistency import optimize_frame_graph

    edges = [true_edge(0, 1), true_edge(1, 2), true_edge(0, 2), true_edge(3, 0)]
    init = chain_frames(0, {(e.a, e.b): (e.T, e.sigma_xy) for e in edges})
    out = optimize_frame_graph(0, edges, init)
    for j in range(4):
        assert np.allclose(out[j], compose(inverse(WORLD[0]), WORLD[j]), atol=1e-6)
    # A biased direct edge (preferred by the least-σ chain) is pulled toward the
    # two-hop evidence and a reverse estimate of equal weight.
    bias = np.array([1.0, -0.6, 0.0, 0.02])
    bad = FrameEdge(0, 2, compose(true_edge(0, 2).T, bias), 0.05, 0.005, 15)
    rev = true_edge(2, 0, s=0.05, sy=0.005)
    edges = [true_edge(0, 1, s=0.2, sy=0.01), true_edge(1, 2, s=0.2, sy=0.01), bad, rev]
    init = chain_frames(0, {(e.a, e.b): (e.T, e.sigma_xy) for e in edges})
    fused = optimize_frame_graph(0, edges, init)
    truth = compose(inverse(WORLD[0]), WORLD[2])
    err_chain = np.hypot(*(init[2] - truth)[:2])
    err_fused = np.hypot(*(fused[2] - truth)[:2])
    assert err_fused < 0.6 * err_chain


def test_windowed_association_never_loses_pairs():
    # Windows add pairs from sub-maps; the merged pairing is one-to-one.
    base = AvatarParams()
    win = AvatarParams(align_window_kf=120)
    sc, sim = make_sim("harbor_fleet", 0, 300.0, base)
    import avatar.runner as R

    got = {}
    orig = R._make_agents
    for name, params in (("base", base), ("win", win)):
        R._make_agents = lambda *a, _n=name, **k: got.setdefault(_n, orig(*a, **k))
        try:
            run_decentralized(sc, sim, params, 0)
        finally:
            R._make_agents = orig
    for i, ag in got["win"].items():
        for pairs in ag.alignment_ids.values():
            mine = [p[0] for p in pairs]
            rem = [p[1] for p in pairs]
            assert len(set(mine)) == len(mine) and len(set(rem)) == len(rem)
        n_base = sum(len(v) for v in got["base"][i].alignment_ids.values())
        assert sum(len(v) for v in ag.alignment_ids.values()) >= n_base


def test_frame_links_couple_neighbour_frames_without_changing_rigid_runs():
    # On a run without drift problems the links must not hurt (tolerance 5 %).
    base = AvatarParams()
    links = AvatarParams(frame_links_in_fused=True)
    sc, sim = make_sim("harbor_fleet", 0, 300.0, base)
    a = run_decentralized(sc, sim, base, 0).metrics["ate_team_m"]
    b = run_decentralized(sc, sim, links, 0).metrics["ate_team_m"]
    assert b <= 1.05 * a + 1e-3
