"""Inter-agent landmark association and robust 4-DoF map alignment (task X1, v0.1).

Given an agent's own landmark parts and a neighbour's condensed landmarks
(decoded from the wire, in the neighbour's local frame), estimate
``T_mine_from_remote`` and the set of corresponding pairs.

Pipeline
--------
1. **Candidate gating** (vectorised over all pairs): medium compatibility
   (same medium → full 3-D pair; above↔below → *cross-medium* horizontal-only
   pair), class compatibility, footprint similarity (yaw-invariant: sorted
   footprint dimensions), vertical-extent similarity for same-medium pairs,
   and descriptor cosine similarity when both sides carry descriptors.
2. **Pairwise-consistency graph + greedy max-clique** hypotheses (PCM-style,
   Mangelson et al. ICRA 2018). Two candidate pairs are consistent if they
   use distinct landmarks on both sides and preserve horizontal distance
   (which holds for any 4-DoF transform and for the coaxial model), and, when
   both pairs are same-medium, preserve vertical offset. Greedy cliques grown
   from the highest-degree candidates give the hypotheses. Random 2-point
   sampling was abandoned because, with ~10³ candidates, it rarely drew two
   correct pairs (docs/LOG.md, 2026-09-28).
   Transform: yaw/x/y by weighted Kabsch on the clique; ``z`` from same-medium
   pairs, or from the datum prior ``z = 0`` if there are none.
3. **Uncertainty-scaled gates** (``gate_sigmas`` × combined σ, clamped to
   ``[*_min_m, *_max_m]``). The vertical gate applies to same-medium pairs
   only, so the heights of above-water parts (pile tops) help break the
   periodicity of pile rows. Hypotheses are scored with **MSAC** over a
   **one-to-one** greedy inlier selection. Cross-medium inliers weigh 2/3,
   because they constrain 2 of 3 axes.
4. **Ambiguity test** against perceptual aliasing (regular pile grids): if
   the best *distinct* competing hypothesis reaches ``ambiguity_ratio`` of the
   best score, the alignment is rejected rather than risking a wrong one.
   Alignments supported only by cross-medium (horizontal-only) pairs need
   ``min_inliers_cross_only`` pairs. Team-level cycle consistency of the frame
   graph is the planned stronger check (task T-X2-02).
5. Weighted refinement (:func:`avatar.geometry.align_4dof` in x/y, weighted
   mean in z with the datum prior) and inlier re-selection.

Association is **id-agnostic**: landmark ids are only passed through, never
compared across agents (conventions §6, anti-leak rule).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import align_4dof, rot_z, wrap_angle
from avatar.types import LandmarkFlags

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]


@dataclass(frozen=True)
class LandmarkSet:
    """Column-oriented set of landmark parts (one agent, one frame)."""

    ids: IntArray
    positions: FloatArray  # (n, 3)
    sigma_xy: FloatArray  # (n,)
    sigma_z: FloatArray  # (n,)
    extents: FloatArray  # (n, 3)
    class_ids: IntArray
    flags: IntArray
    descriptors: FloatArray  # (n, D) zeros = none

    def __len__(self) -> int:
        return len(self.ids)

    @property
    def above(self) -> NDArray[np.bool_]:
        return (self.flags & int(LandmarkFlags.ABOVE)) != 0


@dataclass(frozen=True)
class AssociationParams:
    """Tuning of gating, RANSAC, ambiguity rejection and acceptance."""

    min_inliers: int = 4
    min_inliers_cross_only: int = 8  # horizontal-only support is weaker and alias-prone
    gate_sigmas: float = 3.0
    inlier_xy_min_m: float = 0.3
    inlier_xy_max_m: float = 1.0
    inlier_z_min_m: float = 0.3
    inlier_z_max_m: float = 2.0
    cross_model_sigma_m: float = 0.15  # coaxial model error (pile rake / sway)
    clique_seeds: int = 40
    max_candidates_per_landmark: int = 12
    footprint_ratio_max: float = 1.5
    height_ratio_max: float = 2.0
    descriptor_min_cos: float = 0.5
    allow_cross_medium: bool = True
    z_prior_sigma_m: float = 0.1
    ambiguity_ratio: float = 0.8
    distinct_xy_m: float = 2.0
    distinct_yaw_rad: float = 0.1


@dataclass(frozen=True)
class Alignment:
    """Result of aligning a remote landmark set into the local frame."""

    T_mine_from_remote: FloatArray  # [x, y, z, yaw]
    pairs: tuple[tuple[int, int, bool], ...]  # (my index, remote index, cross_medium)
    rms_xy_m: float
    sigma_xy_m: float
    sigma_z_m: float
    sigma_yaw_rad: float

    @property
    def n_inliers(self) -> int:
        return len(self.pairs)


def candidate_pairs(
    mine: LandmarkSet, remote: LandmarkSet, params: AssociationParams
) -> tuple[IntArray, IntArray, NDArray[np.bool_]]:
    """Gate all (mine, remote) pairs; returns index arrays and a cross-medium mask."""
    if len(mine) == 0 or len(remote) == 0:
        empty = np.zeros(0, dtype=np.int64)
        return empty, empty, np.zeros(0, dtype=bool)
    same = mine.above[:, None] == remote.above[None, :]
    ok = same | params.allow_cross_medium

    ca = mine.class_ids[:, None]
    cb = remote.class_ids[None, :]
    ok &= (ca == 0) | (cb == 0) | (ca == cb)

    fa = np.sort(np.maximum(mine.extents[:, :2], 0.25), axis=1)
    fb = np.sort(np.maximum(remote.extents[:, :2], 0.25), axis=1)
    ratio = np.maximum(fa[:, None, :] / fb[None, :, :], fb[None, :, :] / fa[:, None, :])
    ok &= ratio.max(axis=2) <= params.footprint_ratio_max

    ha = np.maximum(mine.extents[:, 2], 0.25)[:, None]
    hb = np.maximum(remote.extents[:, 2], 0.25)[None, :]
    ok &= ~same | (np.maximum(ha / hb, hb / ha) <= params.height_ratio_max)

    da, db = mine.descriptors, remote.descriptors
    if da.shape[1] and db.shape[1] == da.shape[1]:
        na = np.linalg.norm(da, axis=1)
        nb = np.linalg.norm(db, axis=1)
        both = (na[:, None] > 1e-6) & (nb[None, :] > 1e-6)
        cos = (da @ db.T) / np.maximum(na[:, None] * nb[None, :], 1e-12)
        ok &= ~both | (cos >= params.descriptor_min_cos)

    # Keep the most similar remote candidates per own landmark (bounded graph size).
    dissim = np.log(ratio.max(axis=2)) + np.where(same, 0.0, 0.05)
    dissim = np.where(ok, dissim, np.inf)
    k = min(params.max_candidates_per_landmark, dissim.shape[1])
    keep = np.argsort(dissim, axis=1)[:, :k]
    mask = np.zeros_like(ok)
    np.put_along_axis(mask, keep, True, axis=1)
    ok &= mask
    ia, ib = np.nonzero(ok)
    return ia.astype(np.int64), ib.astype(np.int64), ~same[ia, ib]


def _gates(mine, remote, ia, ib, cross, params: AssociationParams):
    """Per-candidate horizontal and vertical gate radii [m]."""
    vxy = mine.sigma_xy[ia] ** 2 + remote.sigma_xy[ib] ** 2
    vxy = vxy + np.where(cross, params.cross_model_sigma_m**2, 0.0)
    gxy = np.clip(params.gate_sigmas * np.sqrt(vxy), params.inlier_xy_min_m, params.inlier_xy_max_m)
    vz = mine.sigma_z[ia] ** 2 + remote.sigma_z[ib] ** 2
    gz = np.clip(params.gate_sigmas * np.sqrt(vz), params.inlier_z_min_m, params.inlier_z_max_m)
    return gxy, gz


def _normalized_residuals(T: FloatArray, mine, remote, ia, ib, cross, gxy, gz):
    """Horizontal and vertical residuals divided by their gates (vertical = 0 if cross)."""
    q = remote.positions[ib] @ rot_z(T[3]).T + T[:3]
    d = mine.positions[ia] - q
    nxy = np.hypot(d[:, 0], d[:, 1]) / gxy
    nz = np.where(cross, 0.0, np.abs(d[:, 2]) / gz)
    return nxy, nz


def _select_inliers(nxy, nz, ia, ib, cross) -> tuple[IntArray, float]:
    """Greedy one-to-one inlier selection; returns (candidate indices, MSAC score)."""
    e2 = 0.5 * (nxy**2 + nz**2)
    ok = np.flatnonzero((nxy <= 1.0) & (nz <= 1.0))
    order = ok[np.argsort(e2[ok])]
    used_a: set[int] = set()
    used_b: set[int] = set()
    keep = []
    for c in order:
        a, b = int(ia[c]), int(ib[c])
        if a in used_a or b in used_b:
            continue
        used_a.add(a)
        used_b.add(b)
        keep.append(c)
    sel = np.asarray(keep, dtype=np.int64)
    w = np.where(cross[sel], 2.0 / 3.0, 1.0)
    return sel, float(np.sum(w * (1.0 - e2[sel])))


def _consistency(mine, remote, ia, ib, cross, gxy, gz) -> NDArray[np.bool_]:
    """Pairwise consistency matrix between candidate pairs (distinct + rigid)."""
    pa = mine.positions[ia]
    pb = remote.positions[ib]
    dxa = np.hypot(pa[:, None, 0] - pa[None, :, 0], pa[:, None, 1] - pa[None, :, 1])
    dxb = np.hypot(pb[:, None, 0] - pb[None, :, 0], pb[:, None, 1] - pb[None, :, 1])
    C = np.abs(dxa - dxb) <= gxy[:, None] + gxy[None, :]
    same = ~cross
    both_same = same[:, None] & same[None, :]
    dza = pa[:, None, 2] - pa[None, :, 2]
    dzb = pb[:, None, 2] - pb[None, :, 2]
    C &= ~both_same | (np.abs(dza - dzb) <= gz[:, None] + gz[None, :])
    C &= ia[:, None] != ia[None, :]
    C &= ib[:, None] != ib[None, :]
    return C


def _greedy_cliques(C: NDArray[np.bool_], n_seeds: int) -> list[IntArray]:
    """Greedy maximal cliques grown from the highest-degree nodes."""
    deg = C.sum(axis=1)
    seeds = np.argsort(-deg)[:n_seeds]
    cliques = []
    for s0 in seeds:
        clique = [int(s0)]
        cand = np.flatnonzero(C[s0])
        while len(cand):
            sub = C[np.ix_(cand, cand)].sum(axis=1)
            nxt = int(cand[np.argmax(sub)])
            clique.append(nxt)
            cand = cand[C[nxt, cand]]
        cliques.append(np.asarray(clique, dtype=np.int64))
    return cliques


def _refine(mine, remote, ia, ib, cross, sel, params: AssociationParams) -> FloatArray:
    wa = 1.0 / (mine.sigma_xy[ia[sel]] ** 2 + remote.sigma_xy[ib[sel]] ** 2 + 1e-4)
    T = align_4dof(remote.positions[ib[sel]], mine.positions[ia[sel]], wa, estimate_z=False)
    same = sel[~cross[sel]]
    if len(same):
        dz = mine.positions[ia[same], 2] - remote.positions[ib[same], 2]
        wz = 1.0 / (mine.sigma_z[ia[same]] ** 2 + remote.sigma_z[ib[same]] ** 2 + 1e-4)
        prior_w = 1.0 / params.z_prior_sigma_m**2
        T[2] = float(np.sum(wz * dz) / (np.sum(wz) + prior_w))
    return T


def align(
    mine: LandmarkSet,
    remote: LandmarkSet,
    params: AssociationParams,
) -> Alignment | None:
    """Estimate ``T_mine_from_remote``; ``None`` if unsupported or ambiguous."""
    ia, ib, cross = candidate_pairs(mine, remote, params)
    n_cand = len(ia)
    if n_cand < params.min_inliers:
        return None
    gxy, gz = _gates(mine, remote, ia, ib, cross, params)
    C = _consistency(mine, remote, ia, ib, cross, gxy, gz)
    hyps: list[tuple[float, int, FloatArray, IntArray]] = []
    for clique in _greedy_cliques(C, params.clique_seeds):
        if len(clique) < 2:
            continue
        T = _refine(mine, remote, ia, ib, cross, clique, params)
        nxy, nz = _normalized_residuals(T, mine, remote, ia, ib, cross, gxy, gz)
        sel, score = _select_inliers(nxy, nz, ia, ib, cross)
        if len(sel) >= 2:
            hyps.append((score, len(sel), T, sel))
    if not hyps:
        return None
    hyps.sort(key=lambda h: h[0], reverse=True)
    best_score, best_n, best_T, sel = hyps[0]
    if best_n < params.min_inliers:
        return None
    for score, _, T, _ in hyps[1:]:
        dxy = np.hypot(T[0] - best_T[0], T[1] - best_T[1])
        dyaw = abs(wrap_angle(T[3] - best_T[3]))
        if dxy > params.distinct_xy_m or dyaw > params.distinct_yaw_rad:
            if score >= params.ambiguity_ratio * best_score:
                return None  # perceptual aliasing: two distinct hypotheses explain the data
            break

    T = _refine(mine, remote, ia, ib, cross, sel, params)
    for _ in range(3):
        nxy, nz = _normalized_residuals(T, mine, remote, ia, ib, cross, gxy, gz)
        new_sel, _ = _select_inliers(nxy, nz, ia, ib, cross)
        if len(new_sel) < params.min_inliers:
            break
        sel = new_sel
        T = _refine(mine, remote, ia, ib, cross, sel, params)
    if len(sel) < params.min_inliers:
        return None
    if np.all(cross[sel]) and len(sel) < params.min_inliers_cross_only:
        return None  # e.g. shifted matches along a periodic pile row (docs/LOG.md)

    q = remote.positions[ib[sel]] @ rot_z(T[3]).T + T[:3]
    d = mine.positions[ia[sel]] - q
    exy = np.hypot(d[:, 0], d[:, 1])
    n = len(sel)
    rms_xy = float(np.sqrt(np.mean(exy**2)))
    pts = mine.positions[ia[sel], :2]
    spread = float(np.sqrt(np.mean(np.sum((pts - pts.mean(axis=0)) ** 2, axis=1))))
    same = ~cross[sel]
    rms_z = float(np.sqrt(np.mean(d[same, 2] ** 2))) if np.any(same) else params.z_prior_sigma_m
    sig_xy = max(rms_xy, 0.05) / np.sqrt(n)
    sig_yaw = max(rms_xy, 0.05) / (np.sqrt(n) * max(spread, 1.0))
    sig_z = max(rms_z, 0.05) / np.sqrt(max(int(np.sum(same)), 1))
    T[3] = wrap_angle(T[3])
    pairs = tuple((int(ia[c]), int(ib[c]), bool(cross[c])) for c in sel)
    return Alignment(T, pairs, rms_xy, float(sig_xy), float(sig_z), float(sig_yaw))
