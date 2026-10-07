"""EKF landmark tracker (T-F3-02): filter algebra, gating, ambiguity, no runaway."""

from __future__ import annotations

import numpy as np
import pytest

from avatar.frontend.ekf_tracker import (
    CONFIRMED,
    TENTATIVE,
    EkfTracker,
    EkfTrackerParams,
    PlatformPrior,
)
from avatar.types import Medium

A, B = Medium.ABOVE, Medium.BELOW


def _tracker(prior: PlatformPrior | None = None, **kw) -> EkfTracker:
    return EkfTracker(1000, EkfTrackerParams(**kw), prior)


def _set_pose_cov(tr: EkfTracker, sigma_xy: float, sigma_yaw: float = 1e-4) -> None:
    tr.P = np.diag([sigma_xy**2, sigma_xy**2, sigma_yaw**2, 0.0, 0.0, 0.0])


def _add(tr: EkfTracker, medium: Medium, xy: tuple[float, float], sigma: float = 0.05) -> int:
    """A confirmed landmark at ``xy``."""
    j = tr._new_landmark(medium, np.array(xy, dtype=float), sigma**2 * np.eye(2))
    tr._status[j] = CONFIRMED
    return j


def _detect(tr: EkfTracker, world_xy, sigma: float = 0.05, medium: Medium = A):
    """Associate one detection at ``world_xy`` (pose at the origin, heading 0)."""
    p = np.array([[world_xy[0], world_xy[1], 0.0]])
    return tr.associate([medium], p, np.full((1, 3), sigma))[0]


def test_predict_covariance_matches_finite_difference_jacobian():
    """P' = F P Fᵀ with F from central differences of the state transition (Q = 0)."""
    prior = PlatformPrior(1.5e-3, 0.01, 0.005)
    odom = np.array([0.8, -0.3, 0.05, 0.12])
    x0 = np.array([3.0, -2.0, 0.7, 1.0e-3, 0.004, -0.002])

    def transition(x: np.ndarray) -> np.ndarray:
        t = _tracker(prior)
        t.x, t.P = x.copy(), np.zeros((6, 6))
        t.predict(odom, np.zeros(4))
        return t.x

    F = np.zeros((6, 6))
    for i in range(6):
        e = np.zeros(6)
        e[i] = 1e-6
        F[:, i] = (transition(x0 + e) - transition(x0 - e)) / 2e-6
    L = np.random.default_rng(0).normal(size=(6, 6))
    P0 = L @ L.T * 1e-3
    t = _tracker(prior)
    t.x, t.P = x0.copy(), P0.copy()
    t.predict(odom, np.zeros(4))
    np.testing.assert_allclose(t.P, F @ P0 @ F.T, atol=1e-9)


def test_predict_grows_position_covariance_with_odometry_noise():
    t = _tracker(odom_noise_scale=1.0)
    for _ in range(100):
        t.predict(np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.03, 0.03, 0.01, 0.002]))
    # Random walk along and across the track (yaw noise adds a cross-track term).
    assert t.P[0, 0] == pytest.approx(100 * 0.03**2, rel=1e-3)
    assert t.P[1, 1] > 100 * 0.03**2
    assert t.P[2, 2] == pytest.approx(100 * 0.002**2 + t.params.start_sigma_rad**2, rel=1e-3)


def test_pose_update_matches_closed_form_kalman_filter():
    """With θ certain the measurement is linear in the position: compare with P S⁻¹ ν."""
    t = _tracker(map_sigma_floor_m=0.0)
    t.P = np.diag([0.5**2, 0.5**2, 1e-14, 0.0, 0.0, 0.0])
    m, C = np.array([10.0, 2.0]), 0.2**2 * np.eye(2)
    t._status[t._new_landmark(A, m, C)] = CONFIRMED
    p_body, sig = np.array([9.6, 2.3, 0.0]), 0.1
    out = t.associate([A], p_body[None], np.full((1, 3), sig))
    assert out[0] == (1000, 1000)
    Pp, R = 0.5**2 * np.eye(2), sig**2 * np.eye(2)
    S = Pp + C + R
    K = Pp @ np.linalg.inv(S)
    np.testing.assert_allclose(t.x[:2], K @ (m - p_body[:2]), atol=1e-9)
    np.testing.assert_allclose(t.P[:2, :2], (np.eye(2) - K) @ Pp, atol=1e-9)


def test_confident_match_and_new_landmark():
    t = _tracker()
    _set_pose_cov(t, 0.1)
    _add(t, A, (5.0, 0.0))
    assert _detect(t, (5.1, 0.05)) == (1000, 1000)
    assert t.n_landmarks == 1
    new = _detect(t, (12.0, 3.0))
    assert new == (1001, 1001) and t.n_landmarks == 2
    # A landmark of another medium at the same place is a different landmark.
    other = _detect(t, (5.1, 0.05), medium=B)
    assert other[0] == 1002


def test_two_detections_of_one_landmark_in_a_keyframe_share_it():
    """Two sensors of one robot (LiDAR and depth camera) see the same pile."""
    t = _tracker()
    _set_pose_cov(t, 0.1)
    _add(t, A, (5.0, 0.0))
    p = np.array([[5.02, 0.0, 0.0], [4.97, 0.03, 0.0]])
    out = t.associate([A, A], p, np.full((2, 3), 0.05))
    assert out[0] == out[1] == (1000, 1000)


def test_ambiguous_detection_is_dropped_and_starts_no_track():
    """Landmarks 6 m apart: with a confident pose the match is clear; with an uncertain
    pose (σ 4 m) a detection between them is dropped, not duplicated (LOG L28)."""
    t = _tracker()
    _set_pose_cov(t, 0.2)
    _add(t, A, (5.0, 0.0))
    _add(t, A, (11.0, 0.0))
    assert _detect(t, (5.1, 0.0)) == (1000, 1000)
    t.P = np.diag([4.0**2, 4.0**2, 1e-4, 0, 0, 0])
    assert _detect(t, (7.6, 0.0)) is None
    assert t.n_landmarks == 2 and t.n_dropped == 1


def test_no_runaway_for_jittered_detections_between_close_landmarks():
    """The failure of the dead-reckoning tracker: detections between two landmarks 1.2 m
    apart spawned ever more tracks. Here every one of them is matched or dropped."""
    t = _tracker()
    _set_pose_cov(t, 0.1)
    _add(t, A, (5.0, 0.0))
    _add(t, A, (6.2, 0.0))
    jitter = np.random.default_rng(0).uniform(-0.4, 0.4, size=(60, 2))
    for j in jitter:
        _detect(t, (5.6 + j[0], j[1]))
        _set_pose_cov(t, 0.1)  # keep the pose prior tight so the test isolates association
    assert t.n_landmarks == 2 and t.n_dropped > 0


def test_intra_agent_coaxial_key():
    t = _tracker()
    _set_pose_cov(t, 0.1)
    above = _detect(t, (5.0, 0.0), medium=A)
    below = _detect(t, (5.3, 0.1), medium=B)
    far = _detect(t, (20.0, 0.0), medium=B)
    assert above[1] == below[1] and far[1] != above[1]


def test_pose_update_pulls_the_estimate_towards_the_map():
    """A robot that thinks it is 1 m off is corrected by a re-observed landmark."""
    t = _tracker()
    _set_pose_cov(t, 0.1)
    _add(t, A, (10.0, 0.0), sigma=0.02)
    t.x[0] = 1.0  # believed position is 1 m ahead of the truth
    t.P[0, 0] = t.P[1, 1] = 1.0**2  # ... and the filter knows it may be that far off
    # The landmark is truly 10 m ahead of the true pose (0, 0); the robot thinks it is at x = 1.
    out = t.associate([A], np.array([[10.0, 0.0, 0.0]]), np.full((1, 3), 0.05))
    assert out[0] == (1000, 1000)
    assert abs(t.x[0]) < 0.35 and t.P[0, 0] < 0.5**2


def _run_revisit(scale_error: float) -> tuple[int, int]:
    """Out along a pier of piles, 340 m of empty water, and back. Returns (wrong, landmarks).

    The truth moves 1 m per keyframe along +x (heading 0) and then back along -x
    (heading π); the odometry is ``1 + scale_error`` per metre, so the raw
    dead-reckoning error at the return is several pile spacings.
    """
    piles = np.array([[6.0 * i, y] for i in range(11) for y in (-4.0, 4.0)])
    n_out, n_far = 60, 340
    xs = list(range(0, n_out + n_far + 1)) + list(range(n_out + n_far - 1, -1, -1))
    heads = [0.0] * (n_out + n_far + 1) + [np.pi] * (len(xs) - (n_out + n_far + 1))
    rng = np.random.default_rng(1)
    tr = EkfTracker(0, EkfTrackerParams(), PlatformPrior(0.0, 0.01, 0.0))
    first_true: dict[int, int] = {}
    wrong = 0
    for k, (x, th) in enumerate(zip(xs, heads, strict=True)):
        if k > 0:
            turn = th - heads[k - 1]
            odom = np.array([(1.0 + scale_error) * (1.0 if turn == 0 else 0.0), 0.0, 0.0, turn])
            tr.predict(odom, np.array([0.03, 0.03, 0.01, 0.002]))
        c, s = np.cos(th), np.sin(th)
        pos = np.array([float(x), 0.0])
        rel = piles - pos
        body = np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]], axis=1)
        vis = np.where((np.hypot(body[:, 0], body[:, 1]) < 15.0) & (body[:, 0] > 0.0))[0]
        if len(vis) == 0:
            continue
        pb = np.hstack([body[vis] + rng.normal(0, 0.05, (len(vis), 2)), np.zeros((len(vis), 1))])
        ids = tr.associate([A] * len(vis), pb, np.full((len(vis), 3), 0.05))
        for true_pile, got in zip(vis, ids, strict=True):
            if got is None:
                continue
            if first_true.setdefault(got[0], int(true_pile)) != int(true_pile):
                wrong += 1
    return wrong, tr.n_landmarks


def test_no_wrong_association_after_a_long_featureless_leg():
    """1.5 % scale error over 400 m is 6 m of raw drift, the pile spacing. The
    dead-reckoning-frame tracker makes 4 wrong associations and 30 tracks for the
    22 piles in this run; the EKF tracker must make none."""
    wrong, n_landmarks = _run_revisit(0.015)
    assert wrong == 0
    assert n_landmarks <= 26  # 22 piles; a few duplicates would be tolerable, a runaway not


def test_tentative_landmarks_correct_the_pose_weakly_and_never_the_biases():
    """A landmark with two earlier observations may correct the pose, with inflated noise,
    but cannot change the bias states; a confirmed one corrects more."""
    prior = PlatformPrior(1e-3, 0.01, 0.005)
    results = {}
    for status in (TENTATIVE, CONFIRMED):
        t = _tracker(prior)
        t.P[:2, :2] = 0.5**2 * np.eye(2)
        t.P[0, 4] = t.P[4, 0] = 1e-3  # position error correlated with the scale error
        j = t._new_landmark(A, np.array([10.0, 0.0]), 0.05**2 * np.eye(2), 0.05**2)
        t._nobs[j] = 2
        t._status[j] = status
        t.x[0] = 1.0  # the robot believes it is 1 m ahead of where it is ...
        t._cum_pos = 1.0  # ... which the process noise since the landmark's birth allows
        out = t.associate([A], np.array([[10.0, 0.0, 0.0]]), np.full((1, 3), 0.05))
        assert out[0] is not None
        results[status] = t
    weak, full = results[TENTATIVE], results[CONFIRMED]
    assert 0.0 < weak.x[0] < 1.0 and full.x[0] < weak.x[0]  # both corrected, tentative less
    np.testing.assert_array_equal(weak.x[3:], 0.0)  # biases untouched
    assert (
        full.x[3] != 0.0 or full.x[4] != 0.0 or full.x[5] != 0.0
    )  # the confirmed one informs them


def test_landmark_with_one_observation_does_not_correct_the_pose():
    t = _tracker()
    t.P[:2, :2] = 0.5**2 * np.eye(2)
    t._new_landmark(A, np.array([10.0, 0.0]), 0.05**2 * np.eye(2), 0.05**2)
    t.x[0] = 1.0
    out = t.associate([A], np.array([[10.0, 0.0, 0.0]]), np.full((1, 3), 0.05))
    assert out[0] is not None and t.x[0] == 1.0 and t.n_updates == 0


def test_far_match_is_a_new_landmark_when_the_pose_is_uncertain():
    """8 m from a landmark, a pose σ of 4 m makes the Gaussian match no likelier than a
    landmark not seen before: start a new one (a duplicate is cheap, a wrong match is
    not). With a confident pose the same detection near the landmark is matched."""
    t = _tracker()
    _add(t, A, (5.0, 0.0))
    _set_pose_cov(t, 4.0)
    out = _detect(t, (13.0, 0.0))
    assert out == (1001, 1001) and t.n_landmarks == 2
    t2 = _tracker()
    _add(t2, A, (5.0, 0.0))
    _set_pose_cov(t2, 0.2)
    assert _detect(t2, (5.1, 0.0)) == (1000, 1000) and t2.n_landmarks == 1


def test_static_landmark_is_confirmed_after_four_observations():
    t = _tracker()
    _set_pose_cov(t, 0.05)
    for _ in range(4):
        t.predict(np.array([0.0, 0.0, 0.0, 0.0]), np.array([0.01, 0.01, 0.01, 0.002]))
        out = _detect(t, (12.0, 3.0))
        assert out is not None
    assert t._status[0] == CONFIRMED and t.released(0)


def test_body_fixed_cluster_is_flagged_and_never_moves_the_pose_or_the_biases():
    """A wall seen along the track: its cluster stays at a fixed body-frame position while
    the robot moves 0.5 m per keyframe. Every incarnation must fail the birth test on its
    third observation, before it can update anything, and be dropped (LOG L28: it used
    to push the scale state to 3 %)."""
    prior = PlatformPrior(0.0, 0.01, 0.0)
    t = _tracker(prior)
    outs = []
    for k in range(40):
        if k:
            t.predict(np.array([0.5, 0.0, 0.0, 0.0]), np.array([0.02, 0.02, 0.01, 0.002]))
        outs.append(t.associate([A], np.array([[28.9, 0.0, 0.0]]), np.full((1, 3), 0.09))[0])
    assert t.n_updates == 0
    assert t.x[0] == pytest.approx(0.5 * 39, abs=1e-9)  # dead reckoning only
    np.testing.assert_array_equal(t.x[3:], 0.0)
    # The cluster never gets a landmark that survives to release: each incarnation is
    # flagged, or replaced by a new one before its third observation.
    assert not any(t.released(j) for j in range(t.n_landmarks))
    assert len(outs) == 40


def test_birth_streak_restarts_after_a_long_gap():
    t = _tracker()
    _set_pose_cov(t, 0.05)
    for _ in range(2):
        t.predict(np.zeros(4), np.array([0.01, 0.01, 0.01, 0.002]))
        _detect(t, (12.0, 3.0))
    for _ in range(10):  # not seen for 10 keyframes
        t.predict(np.zeros(4), np.array([0.01, 0.01, 0.01, 0.002]))
    _detect(t, (12.0, 3.0))
    assert t._nobs[0] == 1 and t._status[0] == TENTATIVE


def _row_tracker(xs: list[float], **kw) -> EkfTracker:
    """Confirmed landmarks on a row (y = 0) and a pose that is uncertain by 4 m."""
    t = _tracker(map_sigma_floor_m=0.1, **kw)
    for x in xs:
        _add(t, A, (x, 0.0), sigma=0.05)
    _set_pose_cov(t, 4.0)
    return t


def _frame(t: EkfTracker, xs: list[float]):
    n = len(xs)
    p = np.array([[x, 0.0, 0.0] for x in xs])
    return t.associate([A] * n, p, np.full((n, 3), 0.05))


def test_joint_pairing_resolves_detections_that_are_each_ambiguous():
    """Three piles seen in one frame. With a 4 m pose σ each detection alone fits two piles
    of an irregular row about equally well; together only one pairing is consistent."""
    row = [0.0, 6.0, 12.6, 18.5, 24.9]  # irregular spacing 6.0, 6.6, 5.9, 6.4
    seen = [5.7, 12.3, 18.2]  # landmarks 1, 2, 3 as seen from a pose 0.3 m off
    out_off = _frame(_row_tracker(row, joint_pairing=False), seen)
    assert all(o is None for o in out_off)  # ambiguous singly: dropped
    t = _row_tracker(row)
    out = _frame(t, seen)
    assert [o[0] for o in out] == [1001, 1002, 1003]
    assert t.n_joint == 1 and t.n_landmarks == 5  # no new landmark


def test_joint_pairing_refuses_a_tie_on_a_regular_row():
    """On a perfectly regular row the pairing shifted by one pile is equally good: no
    decision, so no match and no new landmark."""
    row = [0.0, 6.0, 12.0, 18.0, 24.0]
    t = _row_tracker(row)
    out = _frame(t, [5.7, 11.7, 17.7])
    assert all(o is None for o in out)
    assert t.n_joint == 0 and t.n_landmarks == 5


def _sparse_fixes(t: EkfTracker, xs: list[float]) -> list:
    """One pile per keyframe (a sparse sensor), the robot not moving. Returns every fix's
    final landmark id (``None``: dropped or still pending), resolved ones included."""
    outs = []
    for x in xs:
        t.predict(np.zeros(4), np.full(4, 1e-4))
        outs.append(_frame(t, [x])[0])
        for k, _, res in t.resolved:
            outs[k - 1] = res
    return [None if o is None else o[0] for o in outs]


def test_window_pairs_sparse_fixes_across_keyframes():
    """T-F3-07: the three piles of the joint-pairing test, now seen one per keyframe. Alone
    each fix is ambiguous under the 4 m pose σ (dropped without a window); kept pending,
    they are paired jointly once the third arrives, and the earlier ones are resolved."""
    row = [0.0, 6.0, 12.6, 18.5, 24.9]
    seen = [5.7, 12.3, 18.2]
    assert _sparse_fixes(_row_tracker(row), seen) == [None, None, None]
    t = _row_tracker(row, window_frames=3)
    assert _sparse_fixes(t, seen) == [1001, 1002, 1003]
    assert t.n_landmarks == 5 and not t._pending  # no duplicate, nothing left waiting


def test_window_expires_and_flush_decides_the_rest():
    """A pending fix that nothing resolves is decided as without a window after
    ``window_frames`` keyframes (here: ambiguous, dropped); ``flush`` decides what is left."""
    row = [0.0, 6.0, 12.6, 18.5, 24.9]
    t = _row_tracker(row, window_frames=2)
    _sparse_fixes(t, [5.7])
    assert len(t._pending) == 1
    for _ in range(2):
        t.predict(np.zeros(4), np.full(4, 1e-4))
        assert t.associate([], np.zeros((0, 3)), np.zeros((0, 3))) == []
    assert not t._pending and t.resolved == [(1, 0, None)] and t.n_dropped == 1
    _sparse_fixes(t, [12.3])
    t.flush()
    assert not t._pending and t.resolved == [(4, 0, None)] and t.n_landmarks == 5


def test_pending_detections_follow_the_odometry():
    """A pending detection is carried into the new body frame (1 m forward, then a 90-degree
    left turn: a point 5 m ahead ends 4 m to the right) and its variance grows."""
    t = _tracker(window_frames=5)
    t._pending = [dict(k=0, slot=0, medium=A, p=np.array([5.0, 0.0, 0.0]),
                       sig=np.full(3, 0.05), var=0.0, truth=None)]  # fmt: skip
    t.predict(np.array([1.0, 0.0, 0.0, np.pi / 2]), np.array([0.01, 0.01, 0.01, 0.002]))
    np.testing.assert_allclose(t._pending[0]["p"][:2], [0.0, -4.0], atol=1e-12)
    assert t._pending[0]["var"] > 0.0
