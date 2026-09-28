"""Front-end error model (T-S1-04): clutter, identity switches, robust observations."""

import numpy as np

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_independent


def _dets(sim):
    return [d for a in sim.agents.values() for kf in a.keyframes for d in kf.detections]


def test_errors_off_by_default_and_on_a_separate_stream():
    params = AvatarParams()
    _, base = make_sim("harbor_fleet", 3, 60.0, params)
    _, noisy = make_sim(
        "harbor_fleet", 3, 60.0, params,
        frontend_errors={"id_switch_prob": 0.2, "clutter_per_kf": 1.0},
    )  # fmt: skip
    assert all(d.true_part_index is None for d in _dets(base))
    # everything except the injected errors is identical (separate random stream)
    for i, a in base.agents.items():
        b = noisy.agents[i]
        assert np.array_equal(a.gt, b.gt)
        for ka, kb in zip(a.keyframes, b.keyframes, strict=True):
            assert (ka.odom is None and kb.odom is None) or np.array_equal(ka.odom, kb.odom)
            real = [d for d in kb.detections if d.true_part_index != -1]
            assert len(real) == len(ka.detections)
            for da, db in zip(ka.detections, real, strict=True):
                assert np.array_equal(da.p_body, db.p_body)
                true = db.part_index if db.true_part_index is None else db.true_part_index
                assert true == da.part_index


def test_switches_stay_nearby_in_the_same_medium_and_clutter_is_unique():
    params = AvatarParams()
    _, sim = make_sim(
        "harbor_fleet", 1, 120.0, params,
        frontend_errors={"id_switch_prob": 0.3, "clutter_per_kf": 0.5, "id_switch_radius_m": 8.0},
    )  # fmt: skip
    parts = sim.world.parts
    pos = sim.world.part_positions
    switched = [d for d in _dets(sim) if d.true_part_index not in (None, -1)]
    clutter = [d for d in _dets(sim) if d.true_part_index == -1]
    assert switched and clutter
    for d in switched:
        assert d.part_index != d.true_part_index
        assert parts[d.part_index].medium == parts[d.true_part_index].medium
        assert np.linalg.norm(pos[d.part_index] - pos[d.true_part_index]) <= 8.0 + 1e-9
    ids = [d.part_index for d in clutter]
    assert len(set(ids)) == len(ids) and min(ids) >= len(parts)


def test_robust_kernel_contains_association_errors():
    errors = {"id_switch_prob": 0.1, "clutter_per_kf": 0.5}
    plain = AvatarParams()
    robust = AvatarParams(point_obs_robust_k=3.0)
    sc, sim = make_sim("harbor_fleet", 0, 200.0, plain, frontend_errors=errors)
    ate_plain = run_independent(sc, sim, plain, 0).metrics["ate_local_m"]
    ate_robust = run_independent(sc, sim, robust, 0).metrics["ate_local_m"]
    uuv = next(a.agent_id for a in sc.agents if a.name == "uuv_0")
    assert ate_robust[uuv] < 0.5 * ate_plain[uuv]
    # the oracle uses ground-truth association: clutter and switches do not reach it
    _, clean = make_sim("harbor_fleet", 0, 200.0, plain)
    a = run_centralized(sc, sim, plain, 0).metrics["ate_team_m"]
    b = run_centralized(sc, clean, plain, 0).metrics["ate_team_m"]
    assert a == b
