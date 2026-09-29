"""EKF landmark tracker: intra-agent data association with covariance gating (T-F3-02).

The dead-reckoning-frame trackers of :mod:`avatar.tier2.frontend` gate on a
distance that grows with the path travelled, so once an underwater robot's raw
drift approaches the spacing of similar piles they confuse neighbours
(docs/LOG.md L28). This tracker instead keeps an extended Kalman filter of the
robot's own pose **and the odometry biases the platform spec puts priors on**
(heading bias [rad/m], translation scale error, gyro scale error), so the
pose covariance says how far the robot can really be from where it thinks it
is. Every landmark carries its own position covariance. A detection is
compared with a landmark by the Mahalanobis distance of their horizontal
positions under the sum of

* the landmark covariance (plus a model-error floor: extended objects, the
  labelling radius),
* the detection's covariance (range-dependent sensor σ, rotated into the map), and
* the pose covariance propagated through the measurement.

A landmark is **tentative** until it has proved static: over its first
``confirm_frames`` observations the detection's world position must stay within
the process noise accumulated since its birth (a body-fixed cluster, such as a
wall seen along the track, slides with the robot and fails this test). Only
confirmed landmarks correct the pose, and detections of landmarks that never
confirm, or that fail the test, are not released (see :meth:`released`).

A detection is matched to a landmark only if the match is clearly likelier
than (a) the runner-up landmark and (b) a new landmark: the log-likelihood
includes the covariance determinant, so a match under large pose uncertainty
loses to "new". Among landmarks, ties are dropped.

Association is joint per keyframe, in rounds: confident matches update the
pose first (sequential EKF updates, each re-gated), which shrinks the pose
covariance and lets detections that were ambiguous or outside the gate be
re-evaluated. A detection that is still ambiguous between two landmarks is
**dropped**: starting a track from it made the previous tracker run away (LOG L28).
Detections that no landmark explains clearly better than "new" start landmarks.

States: ``x = [px, py, θ, b, s, g]``, with the pose in the agent's map frame
(origin and heading at the first keyframe), ``b`` the heading bias [rad/m],
``s`` the translation scale error and ``g`` the gyro scale error. The odometry
model matches the simulator and the estimator (``docs/conventions.md``):
``p' = p + (1 - s) R(θ) d``, ``θ' = θ + (Δθ - b |d|)(1 - g)``.

Not modelled: correlations between the pose and the landmarks (the filter is a
"naive" EKF-SLAM front-end, overconfident in that respect; ``odom_noise_scale``
and ``map_sigma_floor_m`` compensate), and the vertical axis (gating is
horizontal; a landmark's height is the factor graph's business).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.types import Medium

FloatArray = NDArray[np.float64]

N_STATE = 6  # px, py, theta, heading bias, scale error, gyro scale error
POS, YAW, BIAS, SCALE, GYRO = slice(0, 2), 2, 3, 4, 5
TENTATIVE, CONFIRMED, BAD = 0, 1, 2


@dataclass(frozen=True)
class EkfTrackerParams:
    """Tuning of :class:`EkfTracker` (SI units)."""

    # Gate on the squared Mahalanobis distance of horizontal positions
    # (χ² with 2 dof: 11.8 ≈ 99.7 %).
    gate_chi2: float = 11.8
    # A second landmark within this many χ² units of the best makes a detection
    # ambiguous (likelihood ratio exp(-margin / 2); 6 ≈ 1:20).
    ambiguity_margin: float = 6.0
    # Inflation of the reported per-increment odometry σ. The reported values
    # cover the random terms only; systematic terms are states.
    odom_noise_scale: float = 1.5
    # Position error [m] of a landmark that no covariance captures (partial
    # views of extended objects, label radius). Added to every innovation.
    map_sigma_floor_m: float = 0.3
    # Rounds of "update the pose with confident matches, then re-evaluate".
    max_rounds: int = 3
    # Birth test of a landmark (see the module docstring): observations that
    # confirm it, the largest gap [keyframes] that keeps its streak, the gate on
    # |Δ|²/σ² (χ² with 2 dof, 99.9 %) and the position error [m] the test tolerates
    # on top of the sensor and process noise.
    confirm_frames: int = 4
    max_gap_frames: int = 5
    static_chi2: float = 13.8
    static_floor_m: float = 0.1
    # Tentative landmarks correct the pose with this multiple of the measurement
    # variance and never the bias states (a sparse sensor, like a UAV's depth
    # camera, sees a pile for one to three frames and would never confirm it).
    tentative_inflation: float = 4.0
    # ... and only once the landmark has this many earlier observations in its streak
    # (the static test flags a body-fixed cluster on its third, before it can update).
    tentative_min_obs: int = 2
    # Expected density [1/m²] of landmarks (and clutter) in a fresh detection's
    # neighbourhood: the "new landmark" hypothesis a match must beat. When the pose
    # is so uncertain that a Gaussian match is no likelier than a landmark not seen
    # before, the detection starts a new landmark (a duplicate costs less than a
    # wrong match).
    landmark_density_per_m2: float = 0.03
    # Log-likelihood a match must exceed the new-landmark hypothesis by.
    new_landmark_margin: float = 1.0
    # A new landmark of one medium within this distance of one of the other
    # medium is the same structure (intra-agent coaxial link).
    coaxial_link_m: float = 1.0
    # Prior standard deviation of the start pose [m, rad] (agent's map origin).
    start_sigma_m: float = 1e-3
    start_sigma_rad: float = 1e-3


@dataclass(frozen=True)
class PlatformPrior:
    """Prior σ of the odometry biases (from the platform spec, as the estimator's)."""

    heading_bias_rad_per_m: float = 0.0
    scale: float = 0.0
    gyro_scale: float = 0.0


def _rot(theta: float) -> tuple[FloatArray, FloatArray]:
    """``R(θ)`` and its derivative ``dR/dθ`` (2 x 2)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]]), np.array([[-s, -c], [c, -s]])


class EkfTracker:
    """Per-agent landmark-part tracker with pose and landmark covariances.

    ``predict`` once per keyframe (from the second on), then ``associate`` the
    keyframe's detections. Landmark ids returned are ``id_offset + index``.
    """

    def __init__(
        self, id_offset: int, params: EkfTrackerParams, prior: PlatformPrior | None = None
    ) -> None:
        self.params = params
        self.offset = id_offset
        prior = prior or PlatformPrior()
        self.x = np.zeros(N_STATE)
        self.P = np.diag(
            [
                params.start_sigma_m**2,
                params.start_sigma_m**2,
                params.start_sigma_rad**2,
                prior.heading_bias_rad_per_m**2,
                prior.scale**2,
                prior.gyro_scale**2,
            ]
        )
        cap = 64
        self._m = np.zeros((cap, 2))  # landmark positions [m], agent map frame
        self._C = np.zeros((cap, 2, 2))  # their covariances [m²]
        self._medium = np.zeros(cap, dtype=np.int64)
        self._key = np.zeros(cap, dtype=np.int64)  # object key (structure) per landmark
        self._truth = np.full(cap, -2, dtype=np.int64)  # diagnostic label (see ``associate``)
        self._status = np.zeros(cap, dtype=np.int64)  # TENTATIVE / CONFIRMED / BAD
        self._nobs = np.zeros(cap, dtype=np.int64)  # observations in the current birth streak
        self._last_k = np.zeros(cap, dtype=np.int64)
        self._first_q = np.zeros((cap, 2))  # world position of the first observation
        self._first = np.zeros((cap, 3))  # its sensor variance, cumulative process σ² (pos, yaw)
        self.k = 0  # keyframe index (advanced by ``predict``)
        self._cum_pos = 0.0  # cumulative per-axis process variance [m²]
        self._cum_yaw = 0.0  # cumulative yaw process variance [rad²]
        self.n_landmarks = 0
        self.n_dropped = 0  # detections dropped as ambiguous
        self.n_updates = 0  # pose updates from matches

    # ------------------------------------------------------------------ pose filter
    def predict(self, odom: FloatArray, odom_sigmas: FloatArray) -> None:
        """Propagate the pose and biases with one odometry increment ``[dx, dy, dz, dyaw]``."""
        d = np.asarray(odom[:2], dtype=float)
        dist = float(np.linalg.norm(odom[:3]))
        dth = float(odom[3])
        R, Rp = _rot(self.x[YAW])
        scale = 1.0 - self.x[SCALE]
        gscale = 1.0 - self.x[GYRO]
        raw_turn = dth - self.x[BIAS] * dist
        F = np.eye(N_STATE)
        F[POS, YAW] = scale * (Rp @ d)
        F[POS, SCALE] = -(R @ d)
        F[YAW, BIAS] = -dist * gscale
        F[YAW, GYRO] = -raw_turn
        self.x[POS] += scale * (R @ d)
        self.x[YAW] += raw_turn * gscale
        q = self.params.odom_noise_scale
        Q = np.zeros((N_STATE, N_STATE))
        Q[POS, POS] = R @ np.diag([odom_sigmas[0] ** 2, odom_sigmas[1] ** 2]) @ R.T * q**2
        Q[YAW, YAW] = (odom_sigmas[3] * q) ** 2
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        self.k += 1
        self._cum_pos += 0.5 * (Q[0, 0] + Q[1, 1])
        self._cum_yaw += Q[YAW, YAW]

    def _measurement(
        self, p_body: FloatArray, sigmas: FloatArray
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """World position ``q`` (2,), Jacobian ``H`` (2 x 6) and sensor covariance (2 x 2)."""
        R, Rp = _rot(self.x[YAW])
        pb = np.asarray(p_body[:2], dtype=float)
        q = self.x[POS] + R @ pb
        H = np.zeros((2, N_STATE))
        H[:, POS] = np.eye(2)
        H[:, YAW] = Rp @ pb
        Rs = R @ np.diag([sigmas[0] ** 2, sigmas[1] ** 2]) @ R.T
        return q, H, Rs

    # ------------------------------------------------------------------ gating
    def _score(
        self, q: FloatArray, S_base: FloatArray, medium: Medium, truth: int | None = None
    ) -> tuple[FloatArray, FloatArray]:
        """Squared Mahalanobis distance and log-likelihood to every landmark.

        ``inf`` / ``-inf`` for landmarks of the other medium. ``truth`` (diagnostic,
        ground-truth identity): only landmarks with this label (none if negative) match.
        """
        n = self.n_landmarks
        if n == 0:
            return np.zeros(0), np.zeros(0)
        nu = self._m[:n] - q
        S = self._C[:n] + S_base
        a, b, d = S[:, 0, 0], S[:, 0, 1], S[:, 1, 1]
        det = np.maximum(a * d - b * b, 1e-12)
        d2 = (d * nu[:, 0] ** 2 - 2.0 * b * nu[:, 0] * nu[:, 1] + a * nu[:, 1] ** 2) / det
        ll = -0.5 * (d2 + np.log(det)) - np.log(2.0 * np.pi)
        bad = self._medium[:n] != int(medium)
        if truth is not None:
            bad |= (self._truth[:n] != truth) | (truth < 0)
        d2[bad], ll[bad] = np.inf, -np.inf
        return d2, ll

    def _s_base(self, H: FloatArray, Rs: FloatArray) -> FloatArray:
        floor = self.params.map_sigma_floor_m**2
        return H @ self.P @ H.T + Rs + floor * np.eye(2)

    def _update_pose(
        self, j: int, q: FloatArray, H: FloatArray, Rs: FloatArray, tentative: bool
    ) -> None:
        """Sequential EKF update of the pose from the pair (detection, landmark ``j``).

        A tentative landmark (not yet proved static) gets ``tentative_inflation`` times
        the measurement variance and cannot change the bias states.
        """
        prm = self.params
        Rm = self._C[j] + Rs + prm.map_sigma_floor_m**2 * np.eye(2)
        if tentative:
            Rm = Rm * prm.tentative_inflation
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        if tentative:
            K[BIAS:, :] = 0.0
        dx = K @ (self._m[j] - q)
        self.x = self.x + dx
        # A pose correction would have moved the birth detections of tentative landmarks
        # by as much: keep their birth test relative to the corrected pose.
        n = self.n_landmarks
        self._first_q[:n][self._status[:n] == TENTATIVE] += dx[POS]
        I_KH = np.eye(N_STATE) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.n_updates += 1

    # ------------------------------------------------------------------ landmarks
    def _grow(self) -> None:
        cap = len(self._m) * 2
        for name, shape in (("_m", (cap, 2)), ("_C", (cap, 2, 2))):
            new = np.zeros(shape)
            new[: len(getattr(self, name))] = getattr(self, name)
            setattr(self, name, new)
        for name, shape in (("_first_q", (cap, 2)), ("_first", (cap, 3))):
            new = np.zeros(shape)
            new[: len(getattr(self, name))] = getattr(self, name)
            setattr(self, name, new)
        for name in ("_medium", "_key", "_truth", "_status", "_nobs", "_last_k"):
            new_i = np.full(cap, -2 if name == "_truth" else 0, dtype=np.int64)
            new_i[: len(getattr(self, name))] = getattr(self, name)
            setattr(self, name, new_i)

    def _begin_streak(self, j: int, q: FloatArray, sensor_var: float) -> None:
        """(Re)start the birth test of landmark ``j`` at the current keyframe."""
        self._first_q[j] = q
        self._first[j] = (sensor_var, self._cum_pos, self._cum_yaw)
        self._nobs[j], self._last_k[j] = 1, self.k

    def _new_landmark(
        self,
        medium: Medium,
        q: FloatArray,
        cov: FloatArray,
        sensor_var: float = 0.0,
        truth: int = -2,
    ) -> int:
        if self.n_landmarks == len(self._m):
            self._grow()
        j = self.n_landmarks
        key = j
        other = Medium.BELOW if medium == Medium.ABOVE else Medium.ABOVE
        if j:
            d = np.hypot(*(self._m[:j] - q).T)
            d[self._medium[:j] != int(other)] = np.inf
            near = int(np.argmin(d))
            if d[near] < self.params.coaxial_link_m:  # intra-agent coaxial link
                key = int(self._key[near])
        self._m[j], self._C[j] = q, cov
        self._medium[j], self._key[j], self._truth[j] = int(medium), key, truth
        self._status[j] = TENTATIVE
        self._begin_streak(j, q, sensor_var)
        self.n_landmarks += 1
        return j

    def released(self, j: int) -> bool:
        """Whether landmark ``j``'s detections are released (it did not fail the static test)."""
        return bool(self._status[j] != BAD)

    def _static_d2(self, j: int, q: FloatArray, Rs: FloatArray, range_m: float) -> float | None:
        """``|Δ|²/σ²`` of a detection at ``q`` against landmark ``j``'s first observation.

        σ² adds the sensor variances, the process noise accumulated since then (position,
        and yaw times range) and ``static_floor_m``. ``None`` if the birth test does not
        apply (the landmark is not tentative, or its streak was broken by a long gap).
        """
        prm = self.params
        if self._status[j] != TENTATIVE or self.k - self._last_k[j] > prm.max_gap_frames:
            return None
        var0, cum_pos0, cum_yaw0 = self._first[j]
        sigma2 = (
            0.5 * float(np.trace(Rs))
            + var0
            + (self._cum_pos - cum_pos0)
            + range_m**2 * (self._cum_yaw - cum_yaw0)
            + prm.static_floor_m**2
        )
        return float(np.sum((q - self._first_q[j]) ** 2)) / sigma2

    def _birth_test(self, j: int, q: FloatArray, Rs: FloatArray, range_m: float) -> None:
        """Advance landmark ``j``'s birth test with a detection at world position ``q``."""
        prm = self.params
        if self._status[j] != TENTATIVE:
            self._last_k[j] = self.k
            return
        d2 = self._static_d2(j, q, Rs, range_m)
        if d2 is None:  # streak broken by a long gap: start again
            self._begin_streak(j, q, 0.5 * float(np.trace(Rs)))
            return
        self._nobs[j] += 1
        self._last_k[j] = self.k
        if self._nobs[j] >= 3 and d2 > prm.static_chi2:
            self._status[j] = BAD
        elif self._nobs[j] >= prm.confirm_frames:
            self._status[j] = CONFIRMED

    def _fuse(self, j: int, q: FloatArray, D: FloatArray) -> None:
        """Information-form fusion of landmark ``j`` with a detection ``(q, D)``."""
        Ci, Di = np.linalg.inv(self._C[j]), np.linalg.inv(D)
        C_new = np.linalg.inv(Ci + Di)
        self._m[j] = C_new @ (Ci @ self._m[j] + Di @ q)
        self._C[j] = 0.5 * (C_new + C_new.T)

    # ------------------------------------------------------------------ association
    def associate(
        self,
        media: list[Medium],
        p_body: FloatArray,
        sigmas: FloatArray,
        truth: list[int] | None = None,
    ) -> list[tuple[int, int] | None]:
        """Landmark ids ``(id, object key)`` of one keyframe's detections.

        ``p_body`` is ``(n, 3)`` (only x, y are used), ``sigmas`` ``(n, 3)`` the
        detections' 1-σ. ``None``: dropped (ambiguous between landmarks, or attached
        to a landmark that failed the static test).

        ``truth`` is a **diagnostic** (never used by the front-end): the
        ground-truth part index of every detection (negative: clutter). Detections
        then match only landmarks of the same part, which measures the filter's
        consistency (NEES) under perfect association, and is an upper bound.
        """
        prm = self.params
        n = len(media)
        out: list[tuple[int, int] | None] = [None] * n
        if n == 0:
            return out
        ll_new = float(np.log(prm.landmark_density_per_m2))
        margin_ll = 0.5 * prm.ambiguity_margin

        def t(i: int) -> int | None:
            return None if truth is None else int(truth[i])

        def candidates(i: int) -> tuple[FloatArray, FloatArray]:
            """In-gate landmarks of detection ``i`` (best log-likelihood first)."""
            q, H, Rs = self._measurement(p_body[i], sigmas[i])
            d2, ll = self._score(q, self._s_base(H, Rs), media[i], t(i))
            idx = np.flatnonzero(d2 < prm.gate_chi2)
            return idx[np.argsort(-ll[idx])], ll

        def confident(idx: FloatArray, ll: FloatArray) -> bool:
            """Best landmark clearly beats the runner-up and the new-landmark hypothesis."""
            if len(idx) == 0 or ll[idx[0]] - ll_new < prm.new_landmark_margin:
                return False
            return len(idx) == 1 or ll[idx[0]] - ll[idx[1]] >= margin_ll

        pre = [self._measurement(p_body[i], sigmas[i]) for i in range(n)]
        # Pose covariance before any update: landmarks are fused with these.
        D_pre = [Rs + H @ self.P @ H.T for _, H, Rs in pre]
        matched: dict[int, int] = {}
        undecided = set(range(n))
        dropped: set[int] = set()
        for _ in range(prm.max_rounds):
            proposals: list[tuple[float, int, int]] = []
            for i in sorted(undecided):
                idx, ll = candidates(i)
                if confident(idx, ll):
                    proposals.append((-float(ll[idx[0]]), i, int(idx[0])))
            updated = False
            for _, i, j in sorted(proposals):
                idx, ll = candidates(i)  # with the poses updated so far
                if not confident(idx, ll) or idx[0] != j:
                    continue  # no longer clear: stays undecided
                undecided.discard(i)
                if self._status[j] == BAD:  # a landmark that failed the static test
                    dropped.add(i)
                    continue
                q, H, Rs = self._measurement(p_body[i], sigmas[i])
                if self._status[j] == CONFIRMED:
                    matched[i] = j
                    self._update_pose(j, q, H, Rs, tentative=False)
                    updated = True
                    continue
                # Tentative: run the static test before the update, so that a body-fixed
                # cluster is flagged before it can move the pose.
                d2s = self._static_d2(j, q, Rs, float(np.hypot(*p_body[i, :2])))
                if d2s is not None and self._nobs[j] + 1 >= 3 and d2s > prm.static_chi2:
                    self._status[j] = BAD
                    dropped.add(i)
                    continue
                matched[i] = j
                if d2s is not None and self._nobs[j] >= prm.tentative_min_obs:
                    self._update_pose(j, q, H, Rs, tentative=True)
                    updated = True
            if not updated:
                break
        # Whatever is left: ambiguous between landmarks (drop), or no landmark clearly
        # beats "new" (start one).
        new: list[int] = []
        for i in sorted(undecided):
            idx, ll = candidates(i)
            if len(idx) > 1 and ll[idx[0]] - ll[idx[1]] < margin_ll:
                dropped.add(i)
            elif confident(idx, ll):  # cleared up after the last round: match without update
                j = int(idx[0])
                if self._status[j] == BAD:
                    dropped.add(i)
                else:
                    matched[i] = j
            else:
                new.append(i)
        for i, j in list(matched.items()):
            q_pre, _, _ = pre[i]
            self._fuse(j, q_pre, D_pre[i])
            q_post, _, Rs = self._measurement(p_body[i], sigmas[i])
            self._birth_test(j, q_post, Rs, float(np.hypot(*p_body[i, :2])))
            if self._status[j] == BAD:  # failed just now: not released
                dropped.add(i)
                del matched[i]
        for i in new:
            q, H, Rs = self._measurement(p_body[i], sigmas[i])
            cov = Rs + H @ self.P @ H.T
            matched[i] = self._new_landmark(
                media[i], q, cov, 0.5 * float(np.trace(Rs)), t(i) if truth is not None else -2
            )
        self.n_dropped += len(dropped)
        for i, j in matched.items():
            out[i] = (self.offset + j, self.offset + int(self._key[j]))
        return out


__all__ = ["EkfTracker", "EkfTrackerParams", "PlatformPrior"]
