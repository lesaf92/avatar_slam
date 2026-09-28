"""Evaluation metrics: trajectory error, team consistency, frame alignment.

All trajectory errors use a 4-DoF (x, y, z, yaw) least-squares alignment,
matching the observability of gravity-aligned SLAM (conventions §4).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import align_4dof, compose, inverse, transform_points, wrap_angle

FloatArray = NDArray[np.float64]


def ate_rmse(est: FloatArray, gt: FloatArray, align: bool = True) -> float:
    """Absolute trajectory error (RMSE of positions) [m], optionally after 4-DoF alignment."""
    est = np.asarray(est, dtype=float)
    gt = np.asarray(gt, dtype=float)
    if est.shape[0] != gt.shape[0]:
        raise ValueError("trajectories must have the same length")
    p = est[:, :3]
    if align:
        T = align_4dof(p, gt[:, :3])
        p = transform_points(T, p)
    return float(np.sqrt(np.mean(np.sum((p - gt[:, :3]) ** 2, axis=1))))


@dataclass(frozen=True)
class TeamError:
    """Team-level error after one common alignment."""

    ate_m: float
    per_agent_m: dict[int, float]
    connected: tuple[int, ...]
    disconnected: tuple[int, ...]


def team_ate(
    trajectories_team: dict[int, FloatArray],
    gt: dict[int, FloatArray],
    all_agents: list[int],
) -> TeamError:
    """ATE over all connected agents expressed in one team frame, single 4-DoF alignment."""
    ids = sorted(trajectories_team)
    if not ids:
        return TeamError(float("nan"), {}, (), tuple(sorted(all_agents)))
    est = np.vstack([trajectories_team[i][:, :3] for i in ids])
    ref = np.vstack([gt[i][:, :3] for i in ids])
    T = align_4dof(est, ref)
    per = {}
    sq = []
    for i in ids:
        e = transform_points(T, trajectories_team[i][:, :3]) - gt[i][:, :3]
        d2 = np.sum(e**2, axis=1)
        per[i] = float(np.sqrt(np.mean(d2)))
        sq.append(d2)
    disc = tuple(sorted(set(all_agents) - set(ids)))
    return TeamError(float(np.sqrt(np.mean(np.concatenate(sq)))), per, tuple(ids), disc)


def chain_frames(
    anchor: int, edges: dict[tuple[int, int], tuple[FloatArray, float]]
) -> dict[int, FloatArray]:
    """Compose frame estimates into ``T_anchor_from_j`` for every reachable agent.

    ``edges[(a, b)] = (T_a_from_b, sigma_xy)``. Reverse edges are used via
    inversion. Among multiple paths, a best-first search prefers low
    accumulated ``sigma_xy``.
    """
    adj: dict[int, list[tuple[int, FloatArray, float]]] = {}
    for (a, b), (T, s) in edges.items():
        adj.setdefault(a, []).append((b, np.asarray(T, dtype=float), s))
        adj.setdefault(b, []).append((a, inverse(T), s))
    out = {anchor: np.zeros(4)}
    cost = {anchor: 0.0}
    frontier = deque([anchor])
    while frontier:
        # small graphs: pick the lowest-cost frontier node each step
        frontier = deque(sorted(frontier, key=lambda n: cost[n]))
        a = frontier.popleft()
        for b, T_ab, s in adj.get(a, []):
            c = cost[a] + s
            if b not in cost or c < cost[b] - 1e-12:
                cost[b] = c
                out[b] = compose(out[a], T_ab)
                frontier.append(b)
    return out


def frame_error(T_est: FloatArray, T_gt: FloatArray) -> tuple[float, float]:
    """(horizontal translation error [m], |yaw error| [rad]) of a frame estimate."""
    d = compose(inverse(T_gt), T_est)
    return float(np.hypot(d[0], d[1])), float(abs(wrap_angle(d[3])))
