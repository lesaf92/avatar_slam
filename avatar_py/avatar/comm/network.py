"""Discrete-event broadcast network over heterogeneous links.

Every transmission is a broadcast on one link type. Each potential receiver
gets it independently, subject to medium gating, range, and a Bernoulli loss.
Transmissions from one sender on one link are serialised (half duplex): a
packet starts when the previous one has finished. On acoustic links each node
gets a TDMA share ``1 / n_nodes`` of the raw bit rate.
"""

from __future__ import annotations

import heapq
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from avatar.comm.channel import ChannelModel
from avatar.types import LinkType

PositionFn = Callable[[int, float], NDArray[np.float64]]


@dataclass(frozen=True)
class Delivery:
    """A packet arriving at a receiver."""

    t_arrival: float
    t_sent: float
    sender: int
    receiver: int
    link_type: LinkType
    payload: bytes


@dataclass
class LinkStats:
    """Traffic accounting for one link type."""

    packets_sent: int = 0
    bytes_sent: int = 0
    deliveries: int = 0
    bytes_delivered: int = 0
    losses: int = 0


@dataclass
class Network:
    """Broadcast network simulator.

    Parameters
    ----------
    channels
        Channel model per link type.
    memberships
        ``agent_id -> link types the agent can use``.
    position_fn
        ``(agent_id, t) -> ground-truth position [m]`` used for gating/range.
    rng
        Random generator for packet loss.
    """

    channels: dict[LinkType, ChannelModel]
    memberships: dict[int, tuple[LinkType, ...]]
    position_fn: PositionFn
    rng: np.random.Generator
    stats: dict[LinkType, LinkStats] = field(default_factory=lambda: defaultdict(LinkStats))
    events: list[dict] = field(default_factory=list)
    _busy_until: dict[tuple[int, LinkType], float] = field(default_factory=dict)
    _queue: list[tuple[float, int, Delivery]] = field(default_factory=list)
    _counter: int = 0

    def members(self, link: LinkType) -> list[int]:
        """Agents equipped for ``link``."""
        return [a for a, links in self.memberships.items() if link in links]

    def share(self, link: LinkType) -> float:
        """Fraction of the raw bit rate available to one node on ``link``."""
        if link == LinkType.ACOUSTIC:
            return 1.0 / max(1, len(self.members(link)))
        return 1.0

    def send(self, t: float, sender: int, link: LinkType, payload: bytes) -> float:
        """Broadcast ``payload`` from ``sender`` at time ``t``; return the end of airtime."""
        if link not in self.memberships.get(sender, ()):
            raise ValueError(f"agent {sender} has no {link.name} interface")
        ch = self.channels[link]
        if len(payload) > ch.mtu_B:
            raise ValueError(f"payload of {len(payload)} B exceeds {link.name} MTU {ch.mtu_B} B")
        start = max(t, self._busy_until.get((sender, link), -np.inf))
        end = start + ch.airtime_s(len(payload), self.share(link))
        self._busy_until[(sender, link)] = end
        st = self.stats[link]
        st.packets_sent += 1
        st.bytes_sent += len(payload)
        p_s = self.position_fn(sender, start)
        for rx in self.members(link):
            if rx == sender:
                continue
            p_r = self.position_fn(rx, start)
            if not ch.medium_ok(float(p_s[2]), float(p_r[2])):
                continue
            d = float(np.linalg.norm(p_s - p_r))
            if self.rng.random() < ch.loss_prob(d):
                st.losses += 1
                continue
            arrival = end + ch.latency_s(d)
            self._counter += 1
            heapq.heappush(
                self._queue,
                (arrival, self._counter, Delivery(arrival, start, sender, rx, link, payload)),
            )
            self.events.append(
                {
                    "t": start,
                    "t_arrival": arrival,
                    "from": sender,
                    "to": rx,
                    "link": link.name,
                    "bytes": len(payload),
                }
            )
        return end

    def pop_until(self, t: float) -> list[Delivery]:
        """Return all deliveries with arrival time ``<= t`` in arrival order."""
        out: list[Delivery] = []
        while self._queue and self._queue[0][0] <= t:
            _, _, dlv = heapq.heappop(self._queue)
            st = self.stats[dlv.link_type]
            st.deliveries += 1
            st.bytes_delivered += len(dlv.payload)
            out.append(dlv)
        return out

    def pending(self) -> int:
        """Number of packets still in flight."""
        return len(self._queue)
