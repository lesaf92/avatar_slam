"""Front-end error model (T-S1-04): clutter, identity switches, robust observations."""

from itertools import pairwise

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
    plain = AvatarParams(point_obs_gnc=None)  # no robust kernel at all
    robust = AvatarParams(point_obs_gnc=None, point_obs_robust_k=3.0)
    gnc = AvatarParams()  # default: GNC-TLS
    sc, sim = make_sim("harbor_fleet", 0, 200.0, plain, frontend_errors=errors)
    ate_plain = run_independent(sc, sim, plain, 0).metrics["ate_local_m"]
    ate_robust = run_independent(sc, sim, robust, 0).metrics["ate_local_m"]
    ate_gnc = run_independent(sc, sim, gnc, 0).metrics["ate_local_m"]
    uuv = next(a.agent_id for a in sc.agents if a.name == "uuv_0")
    uav = next(a.agent_id for a in sc.agents if a.name == "uav_0")
    assert ate_robust[uuv] < 0.5 * ate_plain[uuv]
    assert ate_gnc[uav] < ate_robust[uav]  # GNC beats Huber where errors dominate (L25)
    # the oracle uses ground-truth association: clutter and switches do not reach it
    _, clean = make_sim("harbor_fleet", 0, 200.0, plain)
    a = run_centralized(sc, sim, plain, 0).metrics["ate_team_m"]
    b = run_centralized(sc, clean, plain, 0).metrics["ate_team_m"]
    assert a == b


def test_persistent_switches_come_in_bursts_at_the_stated_rate():
    """With id_switch_persist_kf = n a part keeps its wrong identity (per sensor) for n
    keyframes; the share of wrong attributions is close to p n / (1 + p n)."""
    params = AvatarParams()
    p, n = 0.01, 20
    _, sim = make_sim(
        "harbor_fleet", 2, 300.0, params,
        frontend_errors={"id_switch_prob": p, "id_switch_persist_kf": n},
    )  # fmt: skip
    # bursts: for one agent and part, runs of identical wrong identity
    wrong_by_part: dict = {}
    total = wrong_total = 0
    for a in sim.agents.values():
        if a.config.role != "slam":
            continue
        for k, kf in enumerate(a.keyframes):
            for d in kf.detections:
                if d.true_part_index is None:
                    total += 1
                    continue
                wrong_total += 1
                total += 1
                wrong_by_part.setdefault((a.config.agent_id, d.true_part_index), []).append(
                    (k, d.part_index)
                )
    assert wrong_total > 0
    runs = []
    for seq in wrong_by_part.values():
        run = 1
        for (k0, w0), (k1, w1) in pairwise(seq):
            if k1 - k0 <= 2 and w0 == w1:
                run += 1
            else:
                runs.append(run)
                run = 1
        runs.append(run)
    assert max(runs) >= n // 2  # long runs of one wrong identity
    assert 0.3 * p * n / (1 + p * n) < wrong_total / total < 3.0 * p * n / (1 + p * n)


def test_persistence_off_keeps_the_independent_model_bit_for_bit():
    params = AvatarParams()
    kw = {"id_switch_prob": 0.1, "clutter_per_kf": 0.5}
    _, a = make_sim("harbor_fleet", 5, 60.0, params, frontend_errors=kw)
    _, b = make_sim(
        "harbor_fleet", 5, 60.0, params, frontend_errors={**kw, "id_switch_persist_kf": 0}
    )
    for i in a.agents:
        for ka, kb in zip(a.agents[i].keyframes, b.agents[i].keyframes, strict=True):
            assert [d.part_index for d in ka.detections] == [d.part_index for d in kb.detections]


def test_splits_give_revisited_parts_a_new_identity_and_keep_it():
    params = AvatarParams()
    _, sim = make_sim(
        "harbor_fleet", 4, 400.0, params,
        frontend_errors={"id_split_prob": 1.0, "id_split_gap_kf": 20},
    )  # fmt: skip
    n_parts = len(sim.world.parts)
    checked = 0
    for a in sim.agents.values():
        if a.config.role != "slam":
            continue
        seen: dict = {}
        for k, kf in enumerate(a.keyframes):
            for d in kf.detections:
                true = d.part_index if d.true_part_index is None else d.true_part_index
                key = (int(d.modality), true)  # the state is kept per sensor and part
                if key in seen and k - seen[key][0] >= 20:
                    assert d.part_index >= n_parts and d.true_part_index == true  # split
                    checked += 1
                elif key in seen:
                    assert d.part_index == seen[key][1]  # same identity between revisits
                seen[key] = (k, d.part_index)
    assert checked > 0


def test_splits_off_leave_measurements_and_ids_unchanged():
    params = AvatarParams()
    _, a = make_sim("harbor_fleet", 4, 60.0, params, frontend_errors={"clutter_per_kf": 0.5})
    _, b = make_sim(
        "harbor_fleet", 4, 60.0, params,
        frontend_errors={"clutter_per_kf": 0.5, "id_split_prob": 0.0},
    )  # fmt: skip
    for i in a.agents:
        for ka, kb in zip(a.agents[i].keyframes, b.agents[i].keyframes, strict=True):
            assert [d.part_index for d in ka.detections] == [d.part_index for d in kb.detections]
