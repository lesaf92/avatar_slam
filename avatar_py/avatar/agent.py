"""Avatar SLAM agent runtime (decentralized backbone, v0).

Each :class:`AvatarAgent` owns two factor graphs (ADR-0004):

* ``local``: built only from the agent's own measurements. Condensed
  landmarks shared with other agents are computed from this graph, so
  received information is never re-broadcast and cannot be double counted.
* ``fused``: ``local`` plus a frame variable ``("T", j)`` per aligned neighbour
  ``j`` and robust *linked-point* factors to the neighbour's landmarks
  (horizontal-only for cross-medium pairs). This is the agent's best estimate.

The runtime is transport-agnostic: it produces and consumes wire-format bytes
(`avatar.comm.codec`). The same class can therefore sit behind the
simulated :class:`avatar.comm.network.Network` or a ROS 2 node.
"""

from __future__ import annotations

import dataclasses
from collections import Counter
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from avatar.backend.graph import FactorGraph, VarType
from avatar.comm import codec
from avatar.comm.scheduler import AlignmentInformation, match_probability, select_voi
from avatar.eval.metrics import chain_frames
from avatar.frontend.association import Alignment, AssociationParams, LandmarkSet, align
from avatar.frontend.frame_consistency import (
    CycleGate,
    FrameEdge,
    consistent_subset,
    optimize_frame_graph,
)
from avatar.geometry import compose, transform_points
from avatar.semantics import normalize
from avatar.sim.agents import AgentConfig
from avatar.sim.measurements import KeyframeData
from avatar.types import Domain, LandmarkFlags, LinkType, Medium, medium_flag

DEFAULT_LINK_RECEIVERS: dict[LinkType, tuple[Domain, ...]] = {
    LinkType.RF: (Domain.AERIAL, Domain.GROUND, Domain.SURFACE),
    LinkType.ACOUSTIC: (Domain.UNDERWATER, Domain.SURFACE),
}

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class AvatarParams:
    """Tuning of the decentralized backbone."""

    exchange_period_s: float = 20.0
    descriptor_dim: int = 8
    acoustic_descriptor_dim: int = 0  # descriptors are not worth their bytes at 10²-10³ bps
    # Digest ordering: "quality" (v0, best in the bandwidth sweep, LOG L17), "voi"
    # (expected D-optimal alignment gain per byte, T-C2-01) or "fifo" (baseline).
    scheduler: str = "quality"
    min_obs_to_share: int = 2
    max_share_sigma_m: float = 3.0
    coaxial_sigma_m: float = 0.1
    cross_medium_model_sigma_m: float = 0.15  # coaxial model error (pile rake, sway)
    frame_z_prior_sigma_m: float = 0.1
    initial_z_sigma_m: float = 0.1
    local_iters: int = 10
    fused_iters: int = 15
    association: AssociationParams = field(default_factory=AssociationParams)
    cycle_check: bool = True  # team frame-graph cycle consistency (T-X2-02)
    point_obs_robust_k: float | None = None  # Huber on landmark observations (front-end errors)
    fuse_team_frames: bool = True  # pose graph over agent frames (else: least-σ chain only)
    # Drift-tolerant association: also align sliding windows of this many own
    # keyframes (0 = whole map only). Recent sub-maps stay nearly rigid when the
    # whole map is bent by drift (LOG L19).
    align_window_kf: int = 0
    # Couple neighbour frame variables in the fused graph with received
    # estimates between neighbours (σ inflated against double counting; L23).
    frame_links_in_fused: bool = False
    frame_link_inflation: float = 2.0
    align_window_min_landmarks: int = 6
    cycle_gate: CycleGate = field(default_factory=CycleGate)


@dataclass
class LandmarkMeta:
    """Front-end bookkeeping for one private landmark part."""

    lid: int
    medium: Medium
    flags: int
    n_obs: int = 0
    class_votes: Counter = field(default_factory=Counter)
    extent_sum: FloatArray = field(default_factory=lambda: np.zeros(3))
    desc_sum: FloatArray | None = None
    n_desc: int = 0
    first_k: int = -1  # first and last own keyframe that observed this part
    last_k: int = -1

    def update(self, modality: LandmarkFlags, cls: int, extent: FloatArray, desc: FloatArray):
        self.n_obs += 1
        self.flags |= int(modality)
        if cls:
            self.class_votes[cls] += 1
        self.extent_sum += extent
        if desc.size and np.linalg.norm(desc) > 1e-9:
            self.desc_sum = desc.copy() if self.desc_sum is None else self.desc_sum + desc
            self.n_desc += 1

    @property
    def class_id(self) -> int:
        return self.class_votes.most_common(1)[0][0] if self.class_votes else 0

    @property
    def extent(self) -> FloatArray:
        return self.extent_sum / max(self.n_obs, 1)

    def descriptor(self, dim: int) -> FloatArray:
        if self.desc_sum is None or dim == 0:
            return np.zeros(dim)
        return normalize(self.desc_sum)


@dataclass
class FrameEstimate:
    """A known estimate of ``T_a_from_b`` (own or received)."""

    T: FloatArray
    sigma_xy: float
    sigma_z: float
    sigma_yaw: float
    n_inliers: int
    source: int  # agent that produced it


class AvatarAgent:
    """One decentralized Avatar SLAM agent."""

    def __init__(
        self,
        cfg: AgentConfig,
        params: AvatarParams,
        part_object_ids: NDArray[np.int64],
        initial_z_m: float,
        rng: np.random.Generator,
    ) -> None:
        self.cfg = cfg
        self.id = cfg.agent_id
        self.params = params
        self.rng = rng
        self.local = FactorGraph(point_obs_robust_k=params.point_obs_robust_k)
        self.fused: FactorGraph | None = None
        self.k = -1
        self.meta: dict[int, LandmarkMeta] = {}
        self._part_object_ids = part_object_ids
        self._initial_z = initial_z_m
        self._lid_of_part: dict[int, int] = {}
        self._used_lids: set[int] = set()
        self._object_parts: dict[int, dict[Medium, int]] = {}
        self._lm_cache: dict[int, tuple[FloatArray, float, float]] = {}
        self._sent: dict[LinkType, dict[int, tuple[FloatArray, int]]] = {}
        self.inbox: dict[int, dict[int, codec.LandmarkRecord]] = {}
        self.inbox_domain: dict[int, int] = {}
        self._dirty: set[int] = set()
        self.alignments: dict[int, Alignment] = {}
        self.alignment_ids: dict[int, list[tuple[int, int, bool]]] = {}
        self.frames: dict[tuple[int, int], FrameEstimate] = {}
        # Neighbours' estimates of T_j_from_self: evidence for the cycle check only.
        self.frames_about_me: dict[int, FrameEstimate] = {}
        self.vetoed: set[int] = set()  # own alignments rejected by the cycle check
        self.rejected_frames: set[tuple[int, int]] = set()  # received ones rejected
        self.rejected_about_me: set[int] = set()
        self.seq = 0
        self.decode_errors = 0
        # Domains whose agents (directly or through a gateway) receive each link.
        self.link_receivers: dict[LinkType, tuple[Domain, ...]] = dict(DEFAULT_LINK_RECEIVERS)
        self._info: dict[tuple[LinkType, Domain], AlignmentInformation] = {}

    # ------------------------------------------------------------------ front-end
    def _new_lid(self) -> int:
        while True:
            lid = int(self.rng.integers(1, 0xFFFF))
            if lid not in self._used_lids:
                self._used_lids.add(lid)
                return lid

    def private_landmark_id(self, part_index: int) -> int:
        """Private id of a ground-truth part (anti-leak remapping, conventions §6)."""
        if part_index not in self._lid_of_part:
            self._lid_of_part[part_index] = self._new_lid()
        return self._lid_of_part[part_index]

    def on_keyframe(self, kf: KeyframeData) -> None:
        """Insert one keyframe of own measurements into the local graph."""
        self.k += 1
        key = ("x", self.k)
        if self.k == 0:
            z0 = kf.abs_z if kf.abs_z is not None else self._initial_z
            init = np.array([0.0, 0.0, z0, 0.0])
            self.local.add_variable(key, VarType.POSE4, init)
            self.local.add_pose_prior(key, init, [1e-3, 1e-3, self.params.initial_z_sigma_m, 1e-3])
        else:
            if kf.odom is None or kf.odom_sigmas is None:
                raise ValueError("keyframes after the first need odometry")
            prev = ("x", self.k - 1)
            init = compose(self.local.value(prev), kf.odom)
            if kf.abs_z is not None:
                init[2] = kf.abs_z
            self.local.add_variable(key, VarType.POSE4, init)
            self.local.add_between(prev, key, kf.odom, kf.odom_sigmas)
        if kf.abs_z is not None and kf.abs_z_sigma is not None:
            self.local.add_z_prior(key, kf.abs_z, kf.abs_z_sigma)
        pose = self.local.value(key)
        for det in kf.detections:
            lid = self.private_landmark_id(det.part_index)
            lkey = ("l", lid)
            if not self.local.has(lkey):
                self.local.add_variable(lkey, VarType.POINT3, transform_points(pose, det.p_body))
                self.meta[lid] = LandmarkMeta(lid, det.medium, int(medium_flag(det.medium)))
                n_parts = len(self._part_object_ids)
                obj = (  # clutter (index beyond the world's parts) belongs to no structure
                    int(self._part_object_ids[det.part_index])
                    if 0 <= det.part_index < n_parts
                    else -1 - det.part_index
                )
                parts = self._object_parts.setdefault(obj, {})
                parts[det.medium] = lid
                if len(parts) == 2:  # both parts of one structure seen by this agent
                    self.local.add_coaxial(
                        ("l", parts[Medium.ABOVE]),
                        ("l", parts[Medium.BELOW]),
                        self.params.coaxial_sigma_m,
                    )
            self.local.add_point_obs(key, lkey, det.p_body, det.sigmas)
            meta = self.meta[lid]
            meta.update(det.modality, det.class_id, det.extent, det.descriptor)
            meta.last_k = self.k
            if meta.first_k < 0:
                meta.first_k = self.k

    # ------------------------------------------------------------------ local solve
    def solve_local(self, with_marginals: bool = True) -> None:
        """Optimize the local graph and refresh the condensed-landmark cache."""
        self.local.optimize(max_iters=self.params.local_iters)
        if not with_marginals:
            return
        lids = [lid for lid, m in self.meta.items() if m.n_obs >= self.params.min_obs_to_share]
        covs = self.local.marginal_covariances([("l", lid) for lid in lids])
        self._lm_cache = {}
        for lid in lids:
            C = covs[("l", lid)]
            sxy = float(np.sqrt(max(C[0, 0], C[1, 1], 0.0)))
            sz = float(np.sqrt(max(C[2, 2], 0.0)))
            self._lm_cache[lid] = (self.local.value(("l", lid)), sxy, sz)

    def _record(self, lid: int, dim: int) -> codec.LandmarkRecord:
        pos, sxy, sz = self._lm_cache[lid]
        m = self.meta[lid]
        return codec.LandmarkRecord(
            landmark_id=lid,
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            sigma_xy=sxy,
            sigma_z=sz,
            extent=tuple(float(v) for v in m.extent),
            class_id=m.class_id,
            flags=m.flags,
            n_obs=m.n_obs,
            descriptor=tuple(float(v) for v in m.descriptor(dim)),
        )

    def _shareable(self) -> list[int]:
        lim = codec.POS_LSB_M * 32767
        return [
            lid
            for lid, (pos, sxy, _) in self._lm_cache.items()
            if sxy <= self.params.max_share_sigma_m and np.all(np.abs(pos) < lim)
        ]

    # ------------------------------------------------------------------ outgoing
    def build_digests(self, t: float, link: LinkType, budget_B: int, mtu_B: int) -> list[bytes]:
        """Condensed-landmark packets for one link, prioritised and within ``budget_B``.

        ``scheduler="voi"`` (T-C2-01): never-sent records are ordered by
        expected D-optimal information gain for the receivers' frame alignment
        (``avatar.comm.scheduler``), accounting for what was already sent on this
        link and for whether the receivers' media can match each record.
        ``scheduler="quality"`` (v0, default): never-sent first by
        ``n_obs / (σ_xy + 0.05)``.
        ``scheduler="fifo"``: never-sent first in the order they were first
        observed (the naive baseline for H2). In all, landmarks that moved > 0.25 m
        or doubled their observation count since last sent follow, then the rest.
        Acoustic packets use ``acoustic_descriptor_dim`` (default 0: no descriptors).
        """
        p = self.params
        dim = p.acoustic_descriptor_dim if link == LinkType.ACOUSTIC else p.descriptor_dim
        per_pkt = codec.max_records_per_packet(mtu_B, dim)
        if per_pkt == 0:
            return []
        sent = self._sent.setdefault(link, {})
        fresh, changed = [], []
        for lid in self._shareable():
            pos, sxy, _ = self._lm_cache[lid]
            quality = self.meta[lid].n_obs / (sxy + 0.05)
            if lid not in sent:
                fresh.append((quality, lid))
            else:
                old_pos, old_n = sent[lid]
                if np.linalg.norm(pos - old_pos) > 0.25 or self.meta[lid].n_obs >= 2 * old_n:
                    changed.append((quality, lid))
        fresh_order = [lid for _, lid in sorted(fresh, reverse=True)]
        receivers = self.link_receivers.get(link, DEFAULT_LINK_RECEIVERS[link])
        infos = [self._info.setdefault((link, d), AlignmentInformation.empty()) for d in receivers]

        def rho(lid: int) -> tuple[float, ...]:
            m = self.meta[lid]
            return tuple(match_probability(m.flags, m.class_id, m.n_obs, d) for d in receivers)

        if p.scheduler == "voi":
            cap = self._capacity(budget_B, per_pkt, dim)
            cands = [(lid, self._lm_cache[lid][0][:2], self._lm_cache[lid][1], rho(lid))
                     for lid in fresh_order]  # fmt: skip
            picked = select_voi(cands, infos, cap)
            rest = [lid for lid in fresh_order if lid not in set(picked)]
            order = picked + [lid for _, lid in sorted(changed, reverse=True)] + rest
            credited = set(picked)
        elif p.scheduler == "quality":
            order = fresh_order + [lid for _, lid in sorted(changed, reverse=True)]
            credited = set()
        elif p.scheduler == "fifo":  # naive baseline: first observed, first sent
            order = [lid for _, lid in fresh] + [lid for _, lid in changed]
            credited = set()
        else:
            raise ValueError(f"unknown scheduler {p.scheduler!r}")
        packets: list[bytes] = []
        used = 0
        i = 0
        while i < len(order):
            room = budget_B - used - codec.digest_size(0, dim)
            n = min(per_pkt, room // codec.record_size(dim), len(order) - i)
            if n <= 0:
                break
            recs = tuple(self._record(lid, dim) for lid in order[i : i + n])
            msg = codec.LandmarkDigest(
                sender_id=self.id,
                seq=self._next_seq(),
                stamp_ms=round(t * 1000),
                domain=int(self.cfg.domain),
                descriptor_dim=dim,
                records=recs,
            )
            pkt = codec.encode(msg)
            packets.append(pkt)
            used += len(pkt)
            for lid in order[i : i + n]:
                if lid not in sent and lid not in credited:
                    for info, r in zip(infos, rho(lid), strict=True):
                        info.add(self._lm_cache[lid][0][:2], self._lm_cache[lid][1], r)
                sent[lid] = (self._lm_cache[lid][0].copy(), self.meta[lid].n_obs)
            i += n
        return packets

    @staticmethod
    def _capacity(budget_B: int, per_pkt: int, dim: int) -> int:
        """Number of records that fit in ``budget_B`` bytes with per-packet overhead."""
        n = used = 0
        while True:
            room = budget_B - used - codec.digest_size(0, dim)
            k = min(per_pkt, room // codec.record_size(dim))
            if k <= 0:
                return n
            n += k
            used += codec.digest_size(k, dim)

    def build_alignment_messages(self, t: float) -> list[bytes]:
        """``FRAME_ALIGNMENT`` packets for every neighbour this agent has aligned."""
        out = []
        for (a, b), fe in self.frames.items():
            if a != self.id or fe.source != self.id:
                continue
            msg = codec.FrameAlignment(
                sender_id=self.id,
                seq=self._next_seq(),
                stamp_ms=round(t * 1000),
                other_id=b,
                n_inliers=fe.n_inliers,
                x=float(fe.T[0]),
                y=float(fe.T[1]),
                z=float(fe.T[2]),
                yaw=float(fe.T[3]),
                sigma_xy=fe.sigma_xy,
                sigma_z=fe.sigma_z,
                sigma_yaw=fe.sigma_yaw,
            )
            out.append(codec.encode(msg))
        return out

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    # ------------------------------------------------------------------ incoming
    def on_packet(self, payload: bytes) -> None:
        """Consume one received packet (wire bytes)."""
        try:
            msg = codec.decode(payload)
        except codec.CodecError:
            self.decode_errors += 1
            return
        if msg.sender_id == self.id:
            return
        if isinstance(msg, codec.LandmarkDigest):
            box = self.inbox.setdefault(msg.sender_id, {})
            for rec in msg.records:
                old = box.get(rec.landmark_id)
                if not rec.descriptor and old is not None and old.descriptor:
                    # e.g. a relayed/acoustic copy without descriptor: keep the RF one
                    rec = dataclasses.replace(rec, descriptor=old.descriptor)
                box[rec.landmark_id] = rec
            self.inbox_domain[msg.sender_id] = msg.domain
            self._dirty.add(msg.sender_id)
        elif isinstance(msg, codec.FrameAlignment):
            fe = FrameEstimate(
                np.array([msg.x, msg.y, msg.z, msg.yaw]),
                msg.sigma_xy,
                msg.sigma_z,
                msg.sigma_yaw,
                msg.n_inliers,
                source=msg.sender_id,
            )
            if msg.other_id == self.id:
                # our frame relative to the sender: we estimate it ourselves, but it
                # closes a 2-cycle with our own alignment of the sender
                self.frames_about_me[msg.sender_id] = fe
                return
            self.frames[(msg.sender_id, msg.other_id)] = fe

    def _my_landmarks(self, lids: list[int] | None = None) -> tuple[LandmarkSet, list[int]]:
        lids = list(self._lm_cache) if lids is None else lids
        return self._landmark_set(
            lids,
            [self._lm_cache[i][0] for i in lids],
            [self._lm_cache[i][1] for i in lids],
            [self._lm_cache[i][2] for i in lids],
            [self.meta[i].extent for i in lids],
            [self.meta[i].class_id for i in lids],
            [self.meta[i].flags for i in lids],
            [self.meta[i].descriptor(self.params.descriptor_dim) for i in lids],
        ), lids

    def _remote_landmarks(self, sender: int) -> tuple[LandmarkSet, list[int]]:
        recs = list(self.inbox.get(sender, {}).values())
        q = np.sqrt(codec.POSITION_QUANT_VAR_M2)
        return self._landmark_set(
            [r.landmark_id for r in recs],
            [r.position for r in recs],
            [np.hypot(r.sigma_xy, q) for r in recs],
            [np.hypot(r.sigma_z, q) for r in recs],
            [r.extent for r in recs],
            [r.class_id for r in recs],
            [r.flags for r in recs],
            [r.descriptor for r in recs],
        ), [r.landmark_id for r in recs]

    def _landmark_set(self, ids, pos, sxy, sz, ext, cls, flags, desc) -> LandmarkSet:
        dim = self.params.descriptor_dim
        n = len(ids)
        # Records may carry D = 0 (acoustic / relayed) or another D: zero-pad or truncate.
        padded = np.zeros((n, dim))
        for i, d in enumerate(desc):
            d = np.asarray(d, dtype=float)[:dim]
            padded[i, : len(d)] = d
        return LandmarkSet(
            ids=np.asarray(ids, dtype=np.int64),
            positions=np.asarray(pos, dtype=float).reshape(n, 3),
            sigma_xy=np.asarray(sxy, dtype=float).reshape(n),
            sigma_z=np.asarray(sz, dtype=float).reshape(n),
            extents=np.asarray(ext, dtype=float).reshape(n, 3),
            class_ids=np.asarray(cls, dtype=np.int64).reshape(n),
            flags=np.asarray(flags, dtype=np.int64).reshape(n),
            descriptors=padded,
        )

    def update_alignments(self) -> None:
        """Re-run association for neighbours whose digests changed."""
        mine, my_ids = self._my_landmarks()
        for sender in sorted(self._dirty):
            remote, remote_ids = self._remote_landmarks(sender)
            res = align(mine, remote, self.params.association)
            ids = None
            if res is not None:
                ids = [(my_ids[i], remote_ids[j], c) for i, j, c in res.pairs]
            if self.params.align_window_kf > 0:
                res, ids = self._windowed_alignment(remote, remote_ids, res, ids)
            if res is None:
                continue
            old = self.alignments.get(sender)
            if self.params.align_window_kf > 0:
                # windows of a drifting map legitimately disagree on the frame by
                # metres; keep whichever pairing links more parts
                if len(ids) < len(self.alignment_ids.get(sender, [])):
                    continue
            elif old is not None and res.n_inliers < old.n_inliers:
                d = res.T_mine_from_remote - old.T_mine_from_remote
                if np.hypot(d[0], d[1]) > 1.0:
                    continue  # weaker, inconsistent hypothesis: keep the old one
            self.alignments[sender] = res
            self.alignment_ids[sender] = ids
        self._dirty.clear()
        self.check_cycles()

    def _windowed_alignment(self, remote, remote_ids, full, full_ids):
        """Align sliding windows of own keyframes and merge their landmark pairs.

        Windows of ``align_window_kf`` keyframes (stride half a window) select
        the own parts observed inside them. Each window with enough parts is
        aligned on its own. Pairs from all accepted windows (and the whole-map
        alignment) are merged; a part paired differently by two windows keeps
        the pair of the window with more inliers. Returns the alignment with
        the most inliers (its frame initialises the fused graph) and the merged
        pairs.
        """
        w = self.params.align_window_kf
        results = [] if full is None else [(full, full_ids)]
        for t0 in range(0, max(self.k - w // 2, 0) + 1, max(w // 2, 1)):
            lids = [
                lid for lid in self._lm_cache
                if self.meta[lid].first_k < t0 + w and self.meta[lid].last_k >= t0
            ]  # fmt: skip
            if len(lids) < self.params.align_window_min_landmarks:
                continue
            sub, sub_ids = self._my_landmarks(lids)
            res = align(sub, remote, self.params.association)
            if res is not None:
                results.append((res, [(sub_ids[i], remote_ids[j], c) for i, j, c in res.pairs]))
        if not results:
            return None, None
        best: dict[int, tuple[int, tuple[int, int, bool]]] = {}
        taken: dict[int, int] = {}
        for res, ids in sorted(results, key=lambda r: -r[0].n_inliers):
            for pair in ids:
                mine_id, rem_id, _ = pair
                if mine_id in best or rem_id in taken:
                    continue  # one-to-one; stronger windows win
                best[mine_id] = (res.n_inliers, pair)
                taken[rem_id] = mine_id
        top = max(results, key=lambda r: r[0].n_inliers)[0]
        return top, [pair for _, pair in best.values()]

    def check_cycles(self) -> None:
        """Veto frame estimates that break a cycle of the team frame graph (T-X2-02).

        Uses this agent's own alignments, the ``FRAME_ALIGNMENT`` estimates it
        received, and the neighbours' estimates of its own frame. Own vetoed
        alignments are left out of the fused graph and are not advertised;
        vetoed received estimates are ignored by :meth:`consistent_frames`.
        The check is re-run from scratch every time, so a veto is lifted when
        new evidence makes the estimate consistent again.
        """
        self.vetoed, self.rejected_frames, self.rejected_about_me = set(), set(), set()
        if not self.params.cycle_check:
            return
        own = {
            s: FrameEdge(self.id, s, a.T_mine_from_remote, a.sigma_xy_m, a.sigma_yaw_rad,
                         a.n_inliers)
            for s, a in self.alignments.items()
        }  # fmt: skip
        received = {
            k: FrameEdge(k[0], k[1], fe.T, fe.sigma_xy, fe.sigma_yaw, fe.n_inliers)
            for k, fe in self.frames.items()
            if fe.source != self.id
        }
        about_me = {
            j: FrameEdge(j, self.id, fe.T, fe.sigma_xy, fe.sigma_yaw, fe.n_inliers)
            for j, fe in self.frames_about_me.items()
        }
        _, rejected = consistent_subset(
            [*own.values(), *received.values(), *about_me.values()], self.params.cycle_gate
        )
        bad = {id(e) for e in rejected}
        self.vetoed = {s for s, e in own.items() if id(e) in bad}
        self.rejected_frames = {k for k, e in received.items() if id(e) in bad}
        self.rejected_about_me = {j for j, e in about_me.items() if id(e) in bad}
        for s in self.vetoed:
            self.frames.pop((self.id, s), None)

    def consistent_frames(self) -> dict[tuple[int, int], FrameEstimate]:
        """Known ``T_a_from_b`` estimates that passed the cycle check."""
        return {k: fe for k, fe in self.frames.items() if k not in self.rejected_frames}

    def team_frames(self, fuse: bool | None = None) -> dict[int, FloatArray]:
        """``T_self_from_j`` [x, y, z, yaw] for every agent reachable in the frame graph.

        The chain of least accumulated σ initialises a small 4-DoF pose graph
        over the agents' frames that fuses every cycle-consistent estimate this
        agent knows: its own, the received ones, and its neighbours' estimates
        of its own frame (``fuse``, default ``params.fuse_team_frames``).
        """
        known = self.consistent_frames()
        edges = [
            FrameEdge(a, b, fe.T, fe.sigma_xy, fe.sigma_yaw, fe.n_inliers, fe.sigma_z)
            for (a, b), fe in known.items()
        ]
        edges += [
            FrameEdge(j, self.id, fe.T, fe.sigma_xy, fe.sigma_yaw, fe.n_inliers, fe.sigma_z)
            for j, fe in self.frames_about_me.items()
            if j not in self.rejected_about_me
        ]
        init = chain_frames(self.id, {(e.a, e.b): (e.T, e.sigma_xy) for e in edges})
        if not (self.params.fuse_team_frames if fuse is None else fuse):
            return init
        return optimize_frame_graph(self.id, edges, init)

    # ------------------------------------------------------------------ fused solve
    def solve_fused(self) -> None:
        """Build and optimize the fused graph (local + inter-agent factors)."""
        fused = self.local.copy()
        p = self.params
        frame_keys = []
        for sender, pairs in self.alignment_ids.items():
            if sender in self.vetoed:
                continue
            fkey = ("T", sender)
            init = self.alignments[sender].T_mine_from_remote
            if self.fused is not None and self.fused.has(fkey):
                init = self.fused.value(fkey)
            fused.add_variable(fkey, VarType.POSE4, init)
            fused.add_pose_prior(fkey, np.zeros(4), [1e4, 1e4, p.frame_z_prior_sigma_m, 1e4])
            box = self.inbox[sender]
            q2 = codec.POSITION_QUANT_VAR_M2
            for my_lid, rlid, cross in pairs:
                rec = box.get(rlid)
                if rec is None or not fused.has(("l", my_lid)):
                    continue
                extra = p.cross_medium_model_sigma_m**2 if cross else 0.0
                sxy = max(np.sqrt(rec.sigma_xy**2 + q2 + extra), 0.05)
                sz = max(np.sqrt(rec.sigma_z**2 + q2), 0.05)
                fused.add_linked_point(
                    ("l", my_lid), fkey, rec.position, [sxy, sxy, sz], horizontal_only=cross
                )
            frame_keys.append(fkey)
        if p.frame_links_in_fused:
            self._add_frame_links(fused, {k[1] for k in frame_keys})
        if self.fused is not None:  # warm start own variables
            for key in fused.keys():
                if key[0] in ("x", "l") and self.fused.has(key):
                    fused.set_value(key, self.fused.value(key))
        fused.optimize(max_iters=p.fused_iters)
        self.fused = fused
        if frame_keys:
            covs = fused.marginal_covariances(frame_keys)
            for fkey in frame_keys:
                C = covs[fkey]
                s = fkey[1]
                self.frames[(self.id, s)] = FrameEstimate(
                    fused.value(fkey),
                    float(np.sqrt(max(C[0, 0], C[1, 1], 0.0))),
                    float(np.sqrt(max(C[2, 2], 0.0))),
                    float(np.sqrt(max(C[3, 3], 0.0))),
                    self.alignments[s].n_inliers,
                    source=self.id,
                )

    def _add_frame_links(self, fused: FactorGraph, neighbours: set[int]) -> None:
        """Between factors among neighbour frame variables from received estimates.

        Without them, each neighbour's frame ``T_self_from_j`` is free, so a
        drifting agent whose map start matches neighbour ``j`` and whose map end
        matches neighbour ``k`` fits each cluster rigidly and never bends its
        trajectory (docs/LOG.md L22). A received, cycle-consistent estimate of
        ``T_j_from_k`` couples the two frames. It reuses information from
        landmarks this graph may also hold, so its σ is inflated by
        ``frame_link_inflation`` (conservative, not exact).
        """
        infl = self.params.frame_link_inflation
        done: set[frozenset[int]] = set()
        for (a, b), fe in self.consistent_frames().items():
            if fe.source == self.id or a not in neighbours or b not in neighbours:
                continue
            if frozenset((a, b)) in done:
                continue
            done.add(frozenset((a, b)))
            sxy = infl * max(fe.sigma_xy, 0.05)
            fused.add_between(
                ("T", a),
                ("T", b),
                fe.T,
                [sxy, sxy, infl * max(fe.sigma_z, 0.05), infl * max(fe.sigma_yaw, 0.005)],
            )

    # ------------------------------------------------------------------ outputs
    def trajectory(self, which: str = "fused") -> FloatArray:
        """Estimated poses ``(k+1, 4)`` in this agent's local frame."""
        g = self.fused if (which == "fused" and self.fused is not None) else self.local
        return np.array([g.value(("x", k)) for k in range(self.k + 1)])

    def landmark_estimates(self, which: str = "fused") -> dict[int, FloatArray]:
        """Landmark positions in this agent's local frame."""
        g = self.fused if (which == "fused" and self.fused is not None) else self.local
        return {lid: g.value(("l", lid)) for lid in self.meta}
