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
from avatar.frontend.association import Alignment, AssociationParams, LandmarkSet, align
from avatar.geometry import compose, transform_points
from avatar.semantics import normalize
from avatar.sim.agents import AgentConfig
from avatar.sim.measurements import KeyframeData
from avatar.types import LandmarkFlags, LinkType, Medium, medium_flag

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class AvatarParams:
    """Tuning of the decentralized backbone."""

    exchange_period_s: float = 20.0
    descriptor_dim: int = 8
    acoustic_descriptor_dim: int = 0  # descriptors are not worth their bytes at 10²-10³ bps
    min_obs_to_share: int = 2
    max_share_sigma_m: float = 3.0
    coaxial_sigma_m: float = 0.1
    cross_medium_model_sigma_m: float = 0.15  # coaxial model error (pile rake, sway)
    frame_z_prior_sigma_m: float = 0.1
    initial_z_sigma_m: float = 0.1
    local_iters: int = 10
    fused_iters: int = 15
    association: AssociationParams = field(default_factory=AssociationParams)


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
        self.local = FactorGraph()
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
        self.seq = 0
        self.decode_errors = 0

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
                obj = int(self._part_object_ids[det.part_index])
                parts = self._object_parts.setdefault(obj, {})
                parts[det.medium] = lid
                if len(parts) == 2:  # both parts of one structure seen by this agent
                    self.local.add_coaxial(
                        ("l", parts[Medium.ABOVE]),
                        ("l", parts[Medium.BELOW]),
                        self.params.coaxial_sigma_m,
                    )
            self.local.add_point_obs(key, lkey, det.p_body, det.sigmas)
            self.meta[lid].update(det.modality, det.class_id, det.extent, det.descriptor)

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

        v0 priority: never-sent first, then landmarks that moved > 0.25 m or doubled
        their observation count since last sent on this link; within a group, by
        ``n_obs / (σ_xy + 0.05)``. (The VoI-per-byte scheduler is task T-C2-01.)
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
        order = [lid for _, lid in sorted(fresh, reverse=True)]
        order += [lid for _, lid in sorted(changed, reverse=True)]
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
                sent[lid] = (self._lm_cache[lid][0].copy(), self.meta[lid].n_obs)
            i += n
        return packets

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
            if msg.other_id == self.id:
                return  # our own frame relative to the sender: we estimate it ourselves
            self.frames[(msg.sender_id, msg.other_id)] = FrameEstimate(
                np.array([msg.x, msg.y, msg.z, msg.yaw]),
                msg.sigma_xy,
                msg.sigma_z,
                msg.sigma_yaw,
                msg.n_inliers,
                source=msg.sender_id,
            )

    def _my_landmarks(self) -> tuple[LandmarkSet, list[int]]:
        lids = list(self._lm_cache)
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
            if res is None:
                continue
            old = self.alignments.get(sender)
            if old is not None and res.n_inliers < old.n_inliers:
                d = res.T_mine_from_remote - old.T_mine_from_remote
                if np.hypot(d[0], d[1]) > 1.0:
                    continue  # weaker, inconsistent hypothesis: keep the old one
            self.alignments[sender] = res
            self.alignment_ids[sender] = [
                (my_ids[i], remote_ids[j], cross) for i, j, cross in res.pairs
            ]
        self._dirty.clear()

    # ------------------------------------------------------------------ fused solve
    def solve_fused(self) -> None:
        """Build and optimize the fused graph (local + inter-agent factors)."""
        fused = self.local.copy()
        p = self.params
        frame_keys = []
        for sender, pairs in self.alignment_ids.items():
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

    # ------------------------------------------------------------------ outputs
    def trajectory(self, which: str = "fused") -> FloatArray:
        """Estimated poses ``(k+1, 4)`` in this agent's local frame."""
        g = self.fused if (which == "fused" and self.fused is not None) else self.local
        return np.array([g.value(("x", k)) for k in range(self.k + 1)])

    def landmark_estimates(self, which: str = "fused") -> dict[int, FloatArray]:
        """Landmark positions in this agent's local frame."""
        g = self.fused if (which == "fused" and self.fused is not None) else self.local
        return {lid: g.value(("l", lid)) for lid in self.meta}
