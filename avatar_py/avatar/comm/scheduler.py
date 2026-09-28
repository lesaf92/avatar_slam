"""Value-of-information-per-byte selection of landmark records (task T-C2-01).

A receiver uses the landmarks we send to estimate the 3-DoF (x, y, yaw)
transform between our local frames (the vertical offset is pinned by the
waterline datum). A record at horizontal position ``p = (x, y)`` with
horizontal σ adds the Fisher information

    J_i = H_iᵀ H_i / σ_i²,   H_i = [[1, 0, −y], [0, 1, x]]

to that estimate. Records already sent on the link are part of what the
receivers know. We greedily pick the record with the largest
**expected D-optimal gain per byte**:

    gain_i = Σ_d ρ_id · [log det(J_d + J_i) − log det(J_d)]
           = Σ_d ρ_id · log det(I₂ + H_i J_d⁻¹ H_iᵀ / σ_i²)   (determinant lemma)

where d runs over the receiver domains on this link (a gateway extends the
link to the domains on its other links), J_d is what receivers in d hold, and
ρ_id is the probability that a receiver in d can match the record. That
probability depends on the receiver's medium and the record's part: an
underwater receiver cannot match a container on the quay, and a pile top only
through a (weaker) cross-medium coaxial pair. The log det of the information
is origin-invariant (translation is estimated jointly), so no centring is
needed.

The criterion favours well-observed landmarks that are **spread out**. This is
what fixes yaw, which dominates the alignment error at large lever arms. It
also stops spending bytes on landmarks that add little information.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.semantics import CLASS_ID
from avatar.types import Domain, LandmarkFlags

FloatArray = NDArray[np.float64]

CROSSING_CLASSES = frozenset(
    {0, CLASS_ID["pile"], CLASS_ID["hull"], CLASS_ID["buoy"], CLASS_ID["quay_wall"]}
)
CROSS_MEDIUM_RELIABILITY = 0.5  # coaxial (horizontal-only) matches are weaker and alias-prone


def match_probability(flags: int, class_id: int, n_obs: int, receiver: Domain) -> float:
    """Probability that a receiver in ``receiver``'s domain can use this record."""
    above = bool(flags & LandmarkFlags.ABOVE)
    crossing = class_id in CROSSING_CLASSES
    if receiver == Domain.SURFACE:
        medium = 1.0
    elif receiver == Domain.UNDERWATER:
        medium = 1.0 if not above else (CROSS_MEDIUM_RELIABILITY if crossing else 0.0)
    else:  # AERIAL / GROUND
        medium = 1.0 if above else (CROSS_MEDIUM_RELIABILITY if crossing else 0.0)
    reliability = 1.0 - np.exp(-n_obs / 3.0)  # few observations → often not re-detected
    return float(medium * reliability)


def record_information(xy: FloatArray, sigma_xy: float) -> FloatArray:
    """Fisher information (3 × 3) of one record for the (x, y, yaw) alignment."""
    x, y = float(xy[0]), float(xy[1])
    H = np.array([[1.0, 0.0, -y], [0.0, 1.0, x]])
    return H.T @ H / max(sigma_xy, SIGMA_FLOOR_M) ** 2


SIGMA_FLOOR_M = 0.02  # no record is trusted beyond the association gate's resolution


@dataclass
class AlignmentInformation:
    """Information the receivers in one domain already hold about our frame (x, y, yaw)."""

    J: FloatArray

    @classmethod
    def empty(cls) -> AlignmentInformation:
        """Weak prior so that the first record has a finite gain."""
        return cls(np.diag([1e-6, 1e-6, 1e-8]))

    def add(self, xy: FloatArray, sigma_xy: float, weight: float = 1.0) -> None:
        """Account for a delivered record, weighted by its match probability."""
        if weight > 0.0:
            self.J = self.J + weight * record_information(xy, sigma_xy)

    def gain(self, xy: FloatArray, sigma_xy: float) -> float:
        """log det(J + J_i) − log det(J) of one record [nats]."""
        return float(self.gains(np.atleast_2d(xy), np.atleast_1d(sigma_xy))[0])

    def gains(self, xy: FloatArray, sigma_xy: FloatArray) -> FloatArray:
        """Vectorised :meth:`gain` for ``xy (n, 2)`` [m] and ``sigma_xy (n,)`` [m].

        With P = J⁻¹ and H = [[1, 0, −y], [0, 1, x]], the determinant lemma gives
        log det(I₂ + H P Hᵀ / σ²), a closed-form 2 × 2 determinant.
        """
        P = np.linalg.inv(self.J)
        x, y = xy[:, 0], xy[:, 1]
        s2 = np.maximum(sigma_xy, SIGMA_FLOOR_M) ** 2
        a = P[0, 0] - 2 * y * P[0, 2] + y * y * P[2, 2]
        b = P[0, 1] + x * P[0, 2] - y * P[1, 2] - x * y * P[2, 2]
        c = P[1, 1] + 2 * x * P[1, 2] + x * x * P[2, 2]
        det = (1 + a / s2) * (1 + c / s2) - (b / s2) ** 2
        return np.log(np.maximum(det, 1.0))


def select_voi(
    candidates: list[tuple[int, FloatArray, float, tuple[float, ...]]],
    infos: list[AlignmentInformation],
    max_records: int,
    min_gain: float = 1e-3,
) -> list[int]:
    """Greedy expected-D-optimal selection over several receiver domains.

    The value of a record is ``Σ_d ρ_d · gain_d``: its information gain for
    each receiver domain ``d`` on the link, weighted by the probability that
    receivers in ``d`` can match it. All records cost the same number of bytes
    on a given link, so this is also the order of value per byte.

    Parameters
    ----------
    candidates
        ``(id, xy [m], sigma_xy [m], rho)`` per record, with ``rho[d]`` the
        match probability for ``infos[d]``'s receivers.
    infos
        Information already delivered to each receiver domain on this link.
        **Updated** with the selected records (weighted by ``rho``).
    max_records
        How many records fit in the byte budget.
    min_gain
        Stop when the best remaining record adds less than this [nats].

    Returns
    -------
    Selected ids in transmission order (most valuable first).
    """
    keep = [c for c in candidates if max(c[3], default=0.0) > 0.0]
    if not keep or max_records <= 0:
        return []
    ids = [c[0] for c in keep]
    xy = np.array([c[1][:2] for c in keep], dtype=float)
    sig = np.array([c[2] for c in keep], dtype=float)
    rho = np.array([c[3] for c in keep], dtype=float).reshape(len(keep), len(infos))
    alive = np.ones(len(keep), dtype=bool)
    chosen: list[int] = []
    while alive.any() and len(chosen) < max_records:
        value = np.zeros(len(keep))
        for d, info in enumerate(infos):
            value += rho[:, d] * info.gains(xy, sig)
        value[~alive] = -np.inf
        k = int(np.argmax(value))
        if value[k] < min_gain:
            break
        alive[k] = False
        for d, info in enumerate(infos):
            info.add(xy[k], sig[k], rho[k, d])
        chosen.append(ids[k])
    return chosen
