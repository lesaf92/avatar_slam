"""Sensorless RF ↔ acoustic relay (surface gateway, ADR-0006, task T-C3-01 v0).

The reference fleet has no surface vessel by default. A quay-side (or buoy)
**gateway** with a topside acoustic modem and a Wi-Fi mesh radio bridges the
two media. It maps nothing. It only relays wire packets, following a policy
designed for links that differ by ~10⁵× in bit rate:

* **Acoustic → RF.** Every received packet is forwarded unchanged. RF capacity
  is effectively unlimited for these packets.
* **RF → acoustic.** Landmark records are stored per originator and re-encoded
  into small ``LANDMARK_DIGEST`` packets that respect the acoustic MTU and a
  per-tick byte budget:

  - ``sender_id`` stays the **originator** (the relay is transparent, so
    receivers attribute landmarks to the frame they are expressed in);
  - descriptors are **stripped** (``descriptor_dim = 0``). Underwater receivers
    mostly hold sonar landmarks without descriptors, so 8 B per record would be
    wasted;
  - priority: records that can take part in a cross-medium match first (an
    ``ABOVE`` part whose class can cross the waterline: unknown, pile, hull,
    buoy, quay wall). Then never-forwarded before changed (> 0.25 m or doubled
    observation count), then more observations. Originators are served
    round-robin.
  - ``FRAME_ALIGNMENT`` packets are **not** forwarded to acoustic. RF agents
    align underwater agents directly from their relayed digests.

The gateway never re-broadcasts a packet on the link it came from, and it
drops duplicate packets (same originator, type and sequence number).
Relaying an originator's own-measurement digests keeps the no-double-counting
invariant of ADR-0004.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field

import numpy as np

from avatar.comm import codec
from avatar.semantics import CLASS_ID
from avatar.types import LandmarkFlags, LinkType

CROSSING_CLASSES = frozenset(
    {0, CLASS_ID["pile"], CLASS_ID["hull"], CLASS_ID["buoy"], CLASS_ID["quay_wall"]}
)


@dataclass
class _Origin:
    domain: int = 0
    records: dict[int, codec.LandmarkRecord] = field(default_factory=dict)
    forwarded: dict[int, tuple[tuple[float, float, float], int]] = field(default_factory=dict)
    seq: int = 0


class Gateway:
    """Store-and-forward relay between RF and acoustic links."""

    def __init__(self, agent_id: int) -> None:
        self.id = agent_id
        self._origins: dict[int, _Origin] = {}
        self._to_rf: list[bytes] = []
        self._seen: set[tuple[int, int, int]] = set()
        self._rr = 0  # round-robin cursor over originators
        self.decode_errors = 0
        self.stats = {"forwarded_to_rf": 0, "records_to_acoustic": 0, "duplicates": 0}

    def on_packet(self, link_in: LinkType, payload: bytes) -> None:
        """Consume one packet received on ``link_in``."""
        try:
            msg = codec.decode(payload)
        except codec.CodecError:
            self.decode_errors += 1
            return
        kind = 1 if isinstance(msg, codec.LandmarkDigest) else 2
        key = (msg.sender_id, kind, msg.seq)
        if key in self._seen:
            self.stats["duplicates"] += 1
            return
        self._seen.add(key)
        if link_in == LinkType.ACOUSTIC:
            self._to_rf.append(payload)
            return
        if isinstance(msg, codec.LandmarkDigest):
            org = self._origins.setdefault(msg.sender_id, _Origin())
            org.domain = msg.domain
            for rec in msg.records:
                org.records[rec.landmark_id] = rec

    def _candidates(self, org: _Origin) -> list[int]:
        fresh, changed = [], []
        for lid, rec in org.records.items():
            crossing = bool(rec.flags & LandmarkFlags.ABOVE) and rec.class_id in CROSSING_CLASSES
            key = (0 if crossing else 1, -rec.n_obs, lid)
            prev = org.forwarded.get(lid)
            if prev is None:
                fresh.append(key)
            else:
                moved = np.linalg.norm(np.subtract(rec.position, prev[0])) > 0.25
                if moved or rec.n_obs >= 2 * max(prev[1], 1):
                    changed.append(key)
        return [k[2] for k in sorted(fresh)] + [k[2] for k in sorted(changed)]

    def build_packets(self, t: float, link_out: LinkType, budget_B: int, mtu_B: int) -> list[bytes]:
        """Packets to transmit on ``link_out`` now, within ``budget_B`` bytes."""
        if link_out == LinkType.RF:
            out, used, keep = [], 0, []
            for pkt in self._to_rf:
                if used + len(pkt) <= budget_B:
                    out.append(pkt)
                    used += len(pkt)
                else:
                    keep.append(pkt)
            self._to_rf = keep
            self.stats["forwarded_to_rf"] += len(out)
            return out
        per_pkt = codec.max_records_per_packet(mtu_B, 0)
        origins = sorted(self._origins)
        if not origins or per_pkt == 0:
            return []
        queues = {o: self._candidates(self._origins[o]) for o in origins}
        packets: list[bytes] = []
        used = 0
        idle_rounds = 0
        while idle_rounds < len(origins):
            o = origins[self._rr % len(origins)]
            self._rr += 1
            q = queues[o]
            room = (budget_B - used - codec.digest_size(0, 0)) // codec.record_size(0)
            n = min(per_pkt, room, len(q))
            if n <= 0:
                idle_rounds += 1
                if room <= 0:
                    break
                continue
            idle_rounds = 0
            org = self._origins[o]
            lids, queues[o] = q[:n], q[n:]
            recs = tuple(dataclasses.replace(org.records[lid], descriptor=()) for lid in lids)
            org.seq = (org.seq + 1) & 0xFFFF
            pkt = codec.encode(
                codec.LandmarkDigest(o, org.seq, round(t * 1000), org.domain, 0, recs)
            )
            packets.append(pkt)
            used += len(pkt)
            for r in recs:
                org.forwarded[r.landmark_id] = (r.position, r.n_obs)
            self.stats["records_to_acoustic"] += len(recs)
        return packets
