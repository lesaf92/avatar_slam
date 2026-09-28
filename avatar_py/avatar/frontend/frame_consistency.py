"""Team frame-graph cycle consistency (task T-X2-02).

Every agent knows a set of inter-agent frame estimates ``T_a_from_b``: the
alignments it computed itself plus the ``FRAME_ALIGNMENT`` messages it
received. Around any cycle of agents the composed transforms must return to
the identity. A wrong alignment (e.g. two agents with no true overlap that
matched aliased piles) breaks every cycle through it.

We keep a consistent subset with a Kruskal-style pass. Edges are visited from
strongest to weakest (more inliers, then lower σ). An edge that joins two
components of the accepted spanning forest is accepted. An edge that closes a
cycle is accepted only if the cycle error (edge vs. forest path) is within a
σ-scaled gate, and is rejected otherwise. Because stronger edges come first,
a rejected edge is always the weakest edge of the inconsistent cycle it
closes. Edges without cycles cannot be checked and are kept.

All frames are 4-DoF ``[x, y, z, yaw]`` (conventions.md); only the horizontal
translation and yaw are tested because z is pinned by the waterline datum.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import compose, inverse, wrap_angle

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class FrameEdge:
    """One estimate of ``T_a_from_b`` [m, rad] with its 1-σ and support."""

    a: int
    b: int
    T: FloatArray
    sigma_xy: float
    sigma_yaw: float
    n_inliers: int


@dataclass(frozen=True)
class CycleGate:
    """Acceptance gate for the cycle error.

    The horizontal gate is ``max(min_xy_m, n_sigma · σ_cycle)``, with
    ``σ_cycle² = Σ_e σ_xy,e² + (σ_yaw,e · L)²`` over the cycle's edges and ``L``
    the largest translation on the cycle (yaw error acts on that lever arm).
    The yaw gate is ``max(min_yaw_rad, n_sigma · sqrt(Σ σ_yaw,e²))``.
    """

    n_sigma: float = 3.0
    min_xy_m: float = 1.0
    min_yaw_rad: float = 0.1


DEFAULT_GATE = CycleGate()


def _path(forest: dict[int, list[tuple[int, FrameEdge, bool]]], src: int, dst: int):
    """Edges (with direction flag) of the forest path src → dst, or None."""
    prev: dict[int, tuple[int, FrameEdge, bool]] = {}
    seen = {src}
    q = deque([src])
    while q:
        n = q.popleft()
        if n == dst:
            break
        for m, e, fwd in forest.get(n, []):
            if m not in seen:
                seen.add(m)
                prev[m] = (n, e, fwd)
                q.append(m)
    if dst not in seen:
        return None
    out = []
    n = dst
    while n != src:
        p, e, fwd = prev[n]
        out.append((e, fwd))
        n = p
    return out[::-1]


def cycle_error(edge: FrameEdge, path: list[tuple[FrameEdge, bool]]) -> tuple[float, float]:
    """(horizontal [m], |yaw| [rad]) discrepancy between ``edge`` and a path a → b."""
    T = np.zeros(4)
    for e, fwd in path:
        T = compose(T, e.T if fwd else inverse(e.T))
    d = compose(inverse(T), edge.T)
    return float(np.hypot(d[0], d[1])), float(abs(wrap_angle(d[3])))


def is_consistent(
    edge: FrameEdge, path: list[tuple[FrameEdge, bool]], gate: CycleGate = DEFAULT_GATE
) -> bool:
    """Whether ``edge`` agrees with the path between its endpoints (see :class:`CycleGate`)."""
    cycle = [edge] + [e for e, _ in path]
    lever = max(float(np.hypot(e.T[0], e.T[1])) for e in cycle)
    var_xy = sum(e.sigma_xy**2 + (e.sigma_yaw * lever) ** 2 for e in cycle)
    var_yaw = sum(e.sigma_yaw**2 for e in cycle)
    dxy, dyaw = cycle_error(edge, path)
    return dxy <= max(gate.min_xy_m, gate.n_sigma * np.sqrt(var_xy)) and dyaw <= max(
        gate.min_yaw_rad, gate.n_sigma * np.sqrt(var_yaw)
    )


def consistent_subset(
    edges: list[FrameEdge], gate: CycleGate = DEFAULT_GATE
) -> tuple[list[FrameEdge], list[FrameEdge]]:
    """Split ``edges`` into (accepted, rejected) by cycle consistency.

    Parallel edges (``a→b`` and ``b→a`` from both agents, or repeated
    estimates) form 2-cycles and are checked like any other cycle.
    """
    order = sorted(edges, key=lambda e: (-e.n_inliers, e.sigma_xy, e.a, e.b))
    forest: dict[int, list[tuple[int, FrameEdge, bool]]] = {}
    accepted: list[FrameEdge] = []
    rejected: list[FrameEdge] = []
    for e in order:
        path = _path(forest, e.a, e.b)
        if path is None:
            forest.setdefault(e.a, []).append((e.b, e, True))
            forest.setdefault(e.b, []).append((e.a, e, False))
            accepted.append(e)
        elif is_consistent(e, path, gate):
            accepted.append(e)
        else:
            rejected.append(e)
    return accepted, rejected
