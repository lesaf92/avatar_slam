"""EKF-SLAM landmark tracker: intra-agent data association with joint covariance (T-F3-02).

The dead-reckoning-frame trackers of :mod:`avatar.tier2.frontend` gate on a
distance that grows with the path travelled, so once an underwater robot's raw
drift approaches the spacing of similar piles they confuse neighbours
(docs/LOG.md L28). This tracker keeps an extended Kalman filter over the robot's
pose, **the odometry biases the platform spec puts priors on** (heading bias
[rad/m], translation scale error, gyro scale error) **and the landmark
positions, with their joint covariance**. The cross-covariances matter: how far
the robot is from a landmark mapped a while ago is the drift accumulated since,
not the (small) absolute pose uncertainty a filter that treats landmarks as
independent reports (that filter turned true revisits into duplicate landmarks,
LOG L29). A detection is compared with a landmark by the Mahalanobis distance
of their horizontal positions under the covariance of ``m_j - q(x)``:
``P_jj + H P_xx Hᵀ - H P_xj - P_jx Hᵀ`` plus the detection's covariance
(range-dependent sensor σ, rotated into the map) and a model-error floor
(extended objects, the labelling radius).

A landmark is **tentative** until it has proved static: over its first
``confirm_frames`` observations the detection's world position must stay within
the process noise accumulated since its birth (a body-fixed cluster, such as a
wall seen along the track, slides with the robot and fails this test). Only
confirmed landmarks can inform the bias states; tentative ones correct the pose
with inflated noise; detections of landmarks that fail the test are not
released (see :meth:`released`).

A detection is matched to a landmark only if the match is clearly likelier
than (a) the runner-up landmark and (b) a new landmark: the log-likelihood
includes the covariance determinant, so a match under large uncertainty loses
to "new". Among landmarks, ties are dropped.

Association is joint per keyframe, in rounds: confident matches update the
state first (sequential EKF updates, each re-gated), which shrinks the pose
covariance and lets detections that were ambiguous or outside the gate be
re-evaluated. A detection that is still ambiguous between two landmarks is
**dropped**: starting a track from it made the first tracker run away (LOG L28).
Detections that no landmark explains clearly better than "new" start landmarks.

States: ``x = [px, py, θ, b, s, g, m_0, m_1, ...]``, with the pose in the agent's
map frame (origin and heading at the first keyframe), ``b`` the heading bias
[rad/m], ``s`` the translation scale error, ``g`` the gyro scale error and
``m_j`` the horizontal landmark positions. The odometry model matches the
simulator and the estimator (``docs/conventions.md``):
``p' = p + (1 - s) R(θ) d``, ``θ' = θ + (Δθ - b |d|)(1 - g)``.

Not modelled: the vertical axis (gating is horizontal; a landmark's height is
the factor graph's business).
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
    landmark_density_per_m2: float = 0.005
    # Log-likelihood a match must exceed the new-landmark hypothesis by.
    new_landmark_margin: float = 1.0
    # A landmark is released (its detections are emitted) when confirmed or observed
    # this many times in all; one-off clutter and sliding clusters never are.
    release_min_obs: int = 3
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
    """Per-agent EKF-SLAM landmark-part tracker.

    ``predict`` once per keyframe (from the second on), then ``associate`` the
    keyframe's detections. Landmark ids returned are ``id_offset + index``. ``x`` and
    ``P`` are views of the pose and bias block of the joint state and covariance.
    """

    def __init__(
        self, id_offset: int, params: EkfTrackerParams, prior: PlatformPrior | None = None
    ) -> None:
        self.params = params
        self.offset = id_offset
        prior = prior or PlatformPrior()
        cap = 64
        self._x = np.zeros(N_STATE + 2 * cap)  # pose, biases, landmark positions
        self._P = np.zeros((N_STATE + 2 * cap, N_STATE + 2 * cap))
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
        self._medium = np.zeros(cap, dtype=np.int64)
        self._key = np.zeros(cap, dtype=np.int64)  # object key (structure) per landmark
        self._truth = np.full(cap, -2, dtype=np.int64)  # diagnostic label (see ``associate``)
        self._status = np.zeros(cap, dtype=np.int64)  # TENTATIVE / CONFIRMED / BAD
        self._nobs = np.zeros(cap, dtype=np.int64)  # observations in the current birth streak
        self._total_obs = np.zeros(cap, dtype=np.int64)  # observations in all
        self._last_k = np.zeros(cap, dtype=np.int64)
        self._first_q = np.zeros((cap, 2))  # world position of the first observation
        self._first = np.zeros((cap, 3))  # its sensor variance, cumulative process σ² (pos, yaw)
        self.k = 0  # keyframe index (advanced by ``predict``)
        self._cum_pos = 0.0  # cumulative per-axis process variance [m²]
        self._cum_yaw = 0.0  # cumulative yaw process variance [rad²]
        self.n_landmarks = 0
        self.n_dropped = 0  # detections dropped as ambiguous
        self.n_updates = 0  # updates of the pose from matches

    @property
    def x(self) -> FloatArray:
        """Pose and bias estimate ``[px, py, θ, b, s, g]`` (a view)."""
        return self._x[:N_STATE]

    @x.setter
    def x(self, value: FloatArray) -> None:
        self._x[:N_STATE] = value

    @property
    def P(self) -> FloatArray:
        """Covariance of :attr:`x` (a view)."""
        return self._P[:N_STATE, :N_STATE]

    @P.setter
    def P(self, value: FloatArray) -> None:
        self._P[:N_STATE, :N_STATE] = value

    def _landmarks(self) -> FloatArray:
        """Landmark positions ``(n, 2)`` (a view of the state)."""
        return self._x[N_STATE : N_STATE + 2 * self.n_landmarks].reshape(-1, 2)

    # ------------------------------------------------------------------ prediction
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
        self._x[POS] += scale * (R @ d)
        self._x[YAW] += raw_turn * gscale
        q = self.params.odom_noise_scale
        Q = np.zeros((N_STATE, N_STATE))
        Q[POS, POS] = R @ np.diag([odom_sigmas[0] ** 2, odom_sigmas[1] ** 2]) @ R.T * q**2
        Q[YAW, YAW] = (odom_sigmas[3] * q) ** 2
        P = self._P
        P[:N_STATE, :N_STATE] = F @ P[:N_STATE, :N_STATE] @ F.T + Q
        P[:N_STATE, :N_STATE] = 0.5 * (P[:N_STATE, :N_STATE] + P[:N_STATE, :N_STATE].T)
        d_all = N_STATE + 2 * self.n_landmarks
        if self.n_landmarks:  # the landmarks do not move: only their cross-covariance does
            P[:N_STATE, N_STATE:d_all] = F @ P[:N_STATE, N_STATE:d_all]
            P[N_STATE:d_all, :N_STATE] = P[:N_STATE, N_STATE:d_all].T
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
        self,
        q: FloatArray,
        H: FloatArray,
        Rs: FloatArray,
        medium: Medium,
        truth: int | None = None,
    ) -> tuple[FloatArray, FloatArray]:
        """Squared Mahalanobis distance and log-likelihood to every landmark.

        The innovation ``m_j - q`` has covariance ``P_jj + H P_xx Hᵀ - H P_xj - P_jx Hᵀ``
        (joint), plus the detection covariance and the model-error floor. ``inf`` /
        ``-inf`` for landmarks of the other medium. ``truth`` (diagnostic, ground-truth
        identity): only landmarks with this label (none if negative) match.
        """
        n = self.n_landmarks
        if n == 0:
            return np.zeros(0), np.zeros(0)
        d_all = N_STATE + 2 * n
        P = self._P
        nu = self._landmarks() - q
        Pxm = P[:N_STATE, N_STATE:d_all].reshape(N_STATE, n, 2)
        HPxj = np.einsum("ab,bjc->jac", H, Pxm)  # (n, 2, 2)
        idx = N_STATE + 2 * np.arange(n)
        Pjj = np.empty((n, 2, 2))
        Pjj[:, 0, 0], Pjj[:, 0, 1] = P[idx, idx], P[idx, idx + 1]
        Pjj[:, 1, 0], Pjj[:, 1, 1] = P[idx + 1, idx], P[idx + 1, idx + 1]
        base = H @ P[:N_STATE, :N_STATE] @ H.T + Rs + self.params.map_sigma_floor_m**2 * np.eye(2)
        S = base + Pjj - HPxj - HPxj.transpose(0, 2, 1)
        a, b, d = S[:, 0, 0], S[:, 0, 1], S[:, 1, 1]
        det = np.maximum(a * d - b * b, 1e-12)
        d2 = (d * nu[:, 0] ** 2 - 2.0 * b * nu[:, 0] * nu[:, 1] + a * nu[:, 1] ** 2) / det
        ll = -0.5 * (d2 + np.log(det)) - np.log(2.0 * np.pi)
        bad = self._medium[:n] != int(medium)
        if truth is not None:
            bad |= (self._truth[:n] != truth) | (truth < 0)
        d2[bad], ll[bad] = np.inf, -np.inf
        return d2, ll

    def _update(self, j: int, q: FloatArray, H: FloatArray, Rs: FloatArray, mode: str) -> None:
        """Joint EKF update of the pose, biases and landmark ``j`` from one match.

        ``mode``: ``"full"`` (a confirmed landmark) or ``"tentative"`` (inflated noise, the
        bias states keep their value). The covariance uses the general (Joseph) form,
        valid for the masked gain.
        """
        prm = self.params
        d_all = N_STATE + 2 * self.n_landmarks
        c = N_STATE + 2 * j
        P = self._P[:d_all, :d_all]
        Rn = Rs + prm.map_sigma_floor_m**2 * np.eye(2)
        if mode == "tentative":
            Rn = Rn * prm.tentative_inflation
        # Measurement e(x) = m_j - q(x) = 0 with Jacobian [-H on the pose, I on m_j].
        PJt = -(P[:, :N_STATE] @ H.T) + P[:, c : c + 2]
        S = -(H @ PJt[:N_STATE]) + PJt[c : c + 2] + Rn
        K = PJt @ np.linalg.inv(S)
        if mode == "tentative":
            K[BIAS:N_STATE] = 0.0
        dx = K @ (q - self._x[c : c + 2])
        self._x[:d_all] += dx
        KJP = K @ PJt.T
        P -= KJP + KJP.T - (K @ S) @ K.T
        P[:] = 0.5 * (P + P.T)
        # A pose correction would have moved the birth detections of tentative
        # landmarks by as much: keep their birth test relative to the corrected pose.
        n = self.n_landmarks
        self._first_q[:n][self._status[:n] == TENTATIVE] += dx[POS]
        self.n_updates += 1

    # ------------------------------------------------------------------ landmarks
    def _grow(self) -> None:
        cap = len(self._medium) * 2
        x = np.zeros(N_STATE + 2 * cap)
        x[: len(self._x)] = self._x
        P = np.zeros((N_STATE + 2 * cap, N_STATE + 2 * cap))
        P[: len(self._x), : len(self._x)] = self._P
        self._x, self._P = x, P
        for name, shape in (("_first_q", (cap, 2)), ("_first", (cap, 3))):
            new = np.zeros(shape)
            new[: len(getattr(self, name))] = getattr(self, name)
            setattr(self, name, new)
        for name in ("_medium", "_key", "_truth", "_status", "_nobs", "_total_obs", "_last_k"):
            new_i = np.full(cap, -2 if name == "_truth" else 0, dtype=np.int64)
            new_i[: len(getattr(self, name))] = getattr(self, name)
            setattr(self, name, new_i)

    def _begin_streak(self, j: int, q: FloatArray, sensor_var: float) -> None:
        """(Re)start the birth test of landmark ``j`` at the current keyframe."""
        self._first_q[j] = q
        self._first[j] = (sensor_var, self._cum_pos, self._cum_yaw)
        self._nobs[j], self._last_k[j] = 1, self.k

    def _register(self, medium: Medium, q: FloatArray, sensor_var: float, truth: int) -> int:
        """Bookkeeping of a new landmark (its state and covariance are set by the caller)."""
        j = self.n_landmarks
        key = j
        other = Medium.BELOW if medium == Medium.ABOVE else Medium.ABOVE
        if j:
            d = np.hypot(*(self._landmarks() - q).T)
            d[self._medium[:j] != int(other)] = np.inf
            near = int(np.argmin(d))
            if d[near] < self.params.coaxial_link_m:  # intra-agent coaxial link
                key = int(self._key[near])
        self._medium[j], self._key[j], self._truth[j] = int(medium), key, truth
        self._status[j], self._total_obs[j] = TENTATIVE, 1
        self._begin_streak(j, q, sensor_var)
        self.n_landmarks += 1
        return j

    def _new_landmark(
        self,
        medium: Medium,
        q: FloatArray,
        cov: FloatArray,
        sensor_var: float = 0.0,
        truth: int = -2,
    ) -> int:
        """A landmark at ``q`` with covariance ``cov``, uncorrelated with the rest of the
        state (tests and priors; :meth:`associate` creates correlated ones)."""
        if self.n_landmarks == len(self._medium):
            self._grow()
        j = self._register(medium, q, sensor_var, truth)
        c = N_STATE + 2 * j
        self._x[c : c + 2] = q
        self._P[c : c + 2, c : c + 2] = cov
        return j

    def _augment(
        self,
        medium: Medium,
        q: FloatArray,
        H: FloatArray,
        Rs: FloatArray,
        sensor_var: float,
        truth: int,
    ) -> int:
        """Add the landmark measured at ``q`` (state augmentation with cross-covariances)."""
        if self.n_landmarks == len(self._medium):
            self._grow()
        d_all = N_STATE + 2 * self.n_landmarks
        P = self._P
        HP = H @ P[:N_STATE, :d_all]  # (2, d_all)
        j = self._register(medium, q, sensor_var, truth)
        c = N_STATE + 2 * j
        self._x[c : c + 2] = q
        P[c : c + 2, :d_all] = HP
        P[:d_all, c : c + 2] = HP.T
        P[c : c + 2, c : c + 2] = H @ P[:N_STATE, :N_STATE] @ H.T + Rs
        return j

    def released(self, j: int) -> bool:
        """Whether landmark ``j``'s detections are released: it did not fail the static test
        and is confirmed or has ``release_min_obs`` observations."""
        if self._status[j] == BAD:
            return False
        return bool(
            self._status[j] == CONFIRMED or self._total_obs[j] >= self.params.release_min_obs
        )

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
            d2, ll = self._score(q, H, Rs, media[i], t(i))
            idx = np.flatnonzero(d2 < prm.gate_chi2)
            return idx[np.argsort(-ll[idx])], ll

        def confident(idx: FloatArray, ll: FloatArray) -> bool:
            """Best landmark clearly beats the runner-up and the new-landmark hypothesis."""
            if len(idx) == 0 or ll[idx[0]] - ll_new < prm.new_landmark_margin:
                return False
            return len(idx) == 1 or ll[idx[0]] - ll[idx[1]] >= margin_ll

        matched: dict[int, int] = {}
        undecided = set(range(n))
        dropped: set[int] = set()

        def apply_match(i: int, j: int) -> bool:
            """Update the state with the match; ``True`` if the pose was corrected."""
            undecided.discard(i)
            if self._status[j] == BAD:  # a landmark that failed the static test
                dropped.add(i)
                return False
            q, H, Rs = self._measurement(p_body[i], sigmas[i])
            if self._status[j] == CONFIRMED:
                matched[i] = j
                self._update(j, q, H, Rs, "full")
                return True
            # Tentative: run the static test before the update, so that a body-fixed
            # cluster is flagged before it can move the pose.
            d2s = self._static_d2(j, q, Rs, float(np.hypot(*p_body[i, :2])))
            if d2s is not None and self._nobs[j] + 1 >= 3 and d2s > prm.static_chi2:
                self._status[j] = BAD
                dropped.add(i)
                return False
            matched[i] = j
            # A young landmark is not updated at all, so that the static test keeps
            # comparing with where it was first seen.
            if d2s is not None and self._nobs[j] >= prm.tentative_min_obs:
                self._update(j, q, H, Rs, "tentative")
                return True
            return False

        for _ in range(prm.max_rounds):
            proposals: list[tuple[float, int, int]] = []
            for i in sorted(undecided):
                idx, ll = candidates(i)
                if confident(idx, ll):
                    proposals.append((-float(ll[idx[0]]), i, int(idx[0])))
            updated = False
            for _, i, j in sorted(proposals):
                idx, ll = candidates(i)  # with the state updated so far
                if not confident(idx, ll) or idx[0] != j:
                    continue  # no longer clear: stays undecided
                updated |= apply_match(i, j)
            if not updated:
                break
        # Whatever is left: ambiguous between landmarks (drop), or no landmark clearly
        # beats "new" (start one).
        new: list[int] = []
        for i in sorted(undecided):
            idx, ll = candidates(i)
            if len(idx) > 1 and ll[idx[0]] - ll[idx[1]] < margin_ll:
                dropped.add(i)
            elif confident(idx, ll):  # cleared up after the last round
                apply_match(i, int(idx[0]))
            else:
                new.append(i)
        for i, j in list(matched.items()):
            self._total_obs[j] += 1
            q_post, _, Rs = self._measurement(p_body[i], sigmas[i])
            self._birth_test(j, q_post, Rs, float(np.hypot(*p_body[i, :2])))
            if self._status[j] == BAD:  # failed just now: not released
                dropped.add(i)
                del matched[i]
        for i in new:
            q, H, Rs = self._measurement(p_body[i], sigmas[i])
            matched[i] = self._augment(
                media[i], q, H, Rs, 0.5 * float(np.trace(Rs)), t(i) if truth is not None else -2
            )
        self.n_dropped += len(dropped)
        for i, j in matched.items():
            out[i] = (self.offset + j, self.offset + int(self._key[j]))
        return out


__all__ = ["EkfTracker", "EkfTrackerParams", "PlatformPrior"]
