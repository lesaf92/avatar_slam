"""*Above and Below*-style centralized server baseline (task T-E2-05).

The closest prior work (McConnell et al., RA-L 2026) builds the team map on a
central server from keyframes streamed by the robots. It did not use real
acoustic communication. This baseline gives that design the **same links,
budgets and front-end** as Avatar SLAM, so the comparison isolates the
architecture:

* **Uplink.** Every robot streams its keyframes (odometry increment, absolute
  z, landmark detections) to one server node over its own links, with the
  token-bucket budgets of :func:`avatar.runner.run_decentralized`. RF is used
  when the robot is at the surface or in the air; otherwise the acoustic
  modem is used. Keyframes are fragmented into MTU-sized packets. On acoustic,
  only every ``acoustic_stride``-th keyframe is sent, with the odometry
  composed over the skipped ones and their detections dropped. A keyframe
  whose fragments did not all arrive is retransmitted (NACK). The downlink
  (NACKs, returned frames) is **not** charged, which favours the baseline.
* **Server.** It rebuilds each robot's local graph from the contiguous prefix
  of keyframes it received and aligns every pair of robots with the same
  association front-end as Avatar (:func:`avatar.frontend.association.align`).
  It then applies the same frame-graph cycle check. At the end it solves one
  joint graph: matched landmark parts are merged, and cross-medium pairs get
  coaxial factors.
* **Team estimate.** A robot's pose at keyframe ``k`` is the server's pose at
  the last received keyframe ``s ≤ k`` composed with the robot's own local
  odometry-SLAM increment from ``s`` to ``k``, as if the server had sent the
  correction back.

Uplink message sizes (this baseline's own format, not wire format v0):
keyframe 27 B (index 2, odometry 4 × f32, σ 4 × f16, flags 1) + 4 B absolute
z (f16 value + σ); detection 13 B (id 2, position 3 × i16 at 2 cm, σ_xy and
σ_z as f16, class 1) + the descriptor bytes of the link (as in Avatar);
fragment header 8 B; plus the codec's per-packet overhead.
"""

from __future__ import annotations

import dataclasses
import struct
from collections import deque

import numpy as np
from numpy.typing import NDArray

from avatar.agent import AvatarAgent, AvatarParams
from avatar.backend.graph import FactorGraph, VarType
from avatar.comm import codec
from avatar.comm.network import Network
from avatar.eval.metrics import chain_frames, team_ate
from avatar.frontend.association import align
from avatar.frontend.frame_consistency import FrameEdge, consistent_subset
from avatar.geometry import compose, inverse, transform_points
from avatar.sim.agents import heading_bias_modelled
from avatar.sim.measurements import KeyframeData, SimData
from avatar.sim.scenarios import Scenario
from avatar.types import LinkType, Medium

FloatArray = NDArray[np.float64]

KF_BASE_B = 27
ABS_Z_B = 4
DET_B = 13
FRAG_HEADER = struct.Struct("<BBIBB")  # magic, agent, keyframe, fragment, n_fragments
FRAG_MAGIC = 0xAB


@dataclasses.dataclass(frozen=True)
class ServerParams:
    """Tuning of the centralized-server baseline."""

    acoustic_stride: int = 10  # send every n-th keyframe over acoustic (odometry composed)
    rf_stride: int = 1
    joint_iters: int = 30


def keyframe_size_B(kf: KeyframeData, descriptor_dim: int) -> int:
    """Uplink size [B] of one keyframe with ``descriptor_dim``-byte descriptors."""
    z = ABS_Z_B if kf.abs_z is not None else 0
    return KF_BASE_B + z + len(kf.detections) * (DET_B + descriptor_dim)


def compose_keyframes(kfs: list[KeyframeData], descriptor_dim: int) -> KeyframeData:
    """One keyframe standing for ``kfs``: composed odometry, the last one's detections.

    Odometry σ are summed in quadrature (first-order, ignores the rotation of
    earlier increments). Descriptors beyond ``descriptor_dim`` are dropped.
    """
    last = kfs[-1]
    odom, sig2 = None, None
    for kf in kfs:
        if kf.odom is None:
            continue
        odom = kf.odom.copy() if odom is None else compose(odom, kf.odom)
        s2 = np.asarray(kf.odom_sigmas, dtype=float) ** 2
        sig2 = s2 if sig2 is None else sig2 + s2
    dets = [
        dataclasses.replace(d, descriptor=_truncate(d.descriptor, descriptor_dim))
        for d in last.detections
    ]
    return KeyframeData(
        last.t,
        odom,
        None if sig2 is None else np.sqrt(sig2),
        dets,
        last.abs_z,
        last.abs_z_sigma,
    )


def _truncate(desc: FloatArray, dim: int) -> FloatArray:
    out = np.zeros_like(np.asarray(desc, dtype=float))
    out[:dim] = np.asarray(desc, dtype=float)[:dim]
    return out


@dataclasses.dataclass
class _Uplink:
    """Sender-side state of one robot."""

    next_kf: int = 0  # first keyframe not yet queued
    queue: deque = dataclasses.field(default_factory=deque)  # (first, last) kf ranges
    in_flight: dict = dataclasses.field(default_factory=dict)  # last -> (first, deadline_s)
    current: list | None = None  # [first, last, fragment sizes, next fragment, link]


class Server:
    """Server-side reconstruction from received keyframes."""

    def __init__(self, scenario: Scenario, sim: SimData, params: AvatarParams, seed: int) -> None:
        self.params = params
        obj_ids = np.array([p.object_id for p in sim.world.parts], dtype=np.int64)
        self.shadows: dict[int, AvatarAgent] = {}
        self.fragments: dict[tuple[int, int], set[int]] = {}
        self.complete: dict[int, dict[int, KeyframeData]] = {}  # agent -> last kf -> data
        self.first_of: dict[int, dict[int, int]] = {}  # agent -> last kf -> first kf
        self.fed: dict[int, int] = {}  # agent -> last kf fed to the shadow
        self.kf_index: dict[int, list[int]] = {}  # agent -> original index of each shadow pose
        self.pending: dict[int, dict[int, KeyframeData]] = {}
        self.fed_keyframes: dict[int, list[KeyframeData]] = {}
        for cfg in scenario.agents:
            if cfg.role != "slam":
                continue
            i = cfg.agent_id
            z0 = float(sim.agents[i].gt[0, 2])
            self.shadows[i] = AvatarAgent(
                cfg, params, obj_ids, z0, np.random.default_rng(seed * 1000 + i)
            )
            self.fed[i] = -1
            self.kf_index[i] = []
            self.pending[i] = {}
            self.fed_keyframes[i] = []
        self.alignments: dict[tuple[int, int], object] = {}

    def receive(self, agent: int, first: int, last: int, kf: KeyframeData) -> None:
        """A complete (possibly composed) keyframe covering ``first..last`` arrived."""
        if last <= self.fed[agent] or last in self.pending[agent]:
            return  # duplicate (retransmission raced the original)
        self.pending[agent][last] = kf
        self.first_of.setdefault(agent, {})[last] = first
        # feed the contiguous prefix to the shadow
        while True:
            nxt = [k for k in self.pending[agent] if self.first_of[agent][k] == self.fed[agent] + 1]
            if not nxt:
                break
            k = nxt[0]
            kf = self.pending[agent].pop(k)
            self.shadows[agent].on_keyframe(kf)
            self.fed_keyframes[agent].append(kf)
            self.fed[agent] = k
            self.kf_index[agent].append(k)

    def update(self) -> list[FrameEdge]:
        """Local solves and all-pairs association; returns cycle-consistent frame edges."""
        live = [i for i, s in self.shadows.items() if s.k >= 0]
        for i in live:
            self.shadows[i].solve_local()
        sets = {i: self.shadows[i]._my_landmarks() for i in live}
        edges = []
        for a in live:
            for b in live:
                if b <= a:
                    continue
                res = align(sets[a][0], sets[b][0], self.params.association)
                if res is None:
                    continue
                self.alignments[(a, b)] = (res, sets[a][1], sets[b][1])
                edges.append(
                    FrameEdge(a, b, res.T_mine_from_remote, res.sigma_xy_m, res.sigma_yaw_rad,
                              res.n_inliers)
                )  # fmt: skip
        accepted, _ = consistent_subset(edges, self.params.cycle_gate)
        return accepted


def run_server(
    scenario: Scenario,
    sim: SimData,
    params: AvatarParams,
    seed: int,
    server_params: ServerParams | None = None,
):
    """Run the *A&B*-style centralized server over the simulated network."""
    from avatar.runner import (  # local import: runner registers this mode
        RunResult,
        _gt_trajs,
        _local_ates,
        _make_agents,
        _names,
    )

    sp_ = server_params or ServerParams()
    gws = [a.agent_id for a in scenario.agents if a.role == "gateway"]
    server_id = gws[0] if gws else scenario.anchor_id
    anchor = scenario.anchor_id
    times = sim.agents[scenario.agents[0].agent_id].times

    def position(agent_id: int, t: float) -> FloatArray:
        k = int(np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 1))
        return sim.agents[agent_id].gt[k, :3]

    net = Network(
        channels=scenario.channels,
        memberships={a.agent_id: a.comm for a in scenario.agents},
        position_fn=position,
        rng=np.random.default_rng(seed + 20_000),
    )
    server = Server(scenario, sim, params, seed)
    robots = list(server.shadows)
    up = {i: _Uplink() for i in robots}
    carry: dict[tuple[int, LinkType], float] = {}
    period = params.exchange_period_s
    next_exchange = period
    connected_s: float | None = None
    first_link: dict[str, float] = {}
    payload_room = {lt: ch.mtu_B - codec.PACKET_OVERHEAD - FRAG_HEADER.size
                    for lt, ch in scenario.channels.items()}  # fmt: skip
    composed: dict[tuple[int, int], tuple[int, KeyframeData]] = {}  # (agent, last) -> first, kf

    def deliver(t_now: float) -> None:
        for dlv in net.pop_until(t_now):
            if dlv.receiver != server_id:
                continue
            magic, agent, last, frag, nfrag = FRAG_HEADER.unpack_from(
                dlv.payload, codec.HEADER_SIZE
            )
            if magic != FRAG_MAGIC:
                continue
            got = server.fragments.setdefault((agent, last), set())
            got.add(frag)
            if len(got) == nfrag:
                first, kf = composed[(agent, last)]
                server.receive(agent, first, last, kf)
                up[agent].in_flight.pop(last, None)

    def uplink_link(i: int, k: int) -> LinkType | None:
        z = float(sim.agents[i].gt[k, 2])
        cfg = scenario.agent(i)
        for lt in (LinkType.RF, LinkType.ACOUSTIC):
            if lt in cfg.comm and lt in scenario.agent(server_id).comm:
                if scenario.channels[lt].medium_ok(z, float(sim.agents[server_id].gt[k, 2])):
                    return lt
        return None

    for k, t in enumerate(times):
        deliver(float(t))
        if t + 1e-9 < next_exchange:
            continue
        next_exchange += period
        for i in robots:
            u = up[i]
            if i == server_id:
                continue
            # NACK: fragments that should have arrived by now are resent
            for last, (first, deadline) in list(u.in_flight.items()):
                if t > deadline and last not in server.pending[i] and last > server.fed[i]:
                    u.queue.appendleft((first, last))
                    del u.in_flight[last]
                    server.fragments.pop((i, last), None)
            lt = uplink_link(i, k)
            if lt is None:
                continue
            ch = scenario.channels[lt]
            acoustic = lt == LinkType.ACOUSTIC
            stride = sp_.acoustic_stride if acoustic else sp_.rf_stride
            dim = params.acoustic_descriptor_dim if acoustic else params.descriptor_dim
            while u.next_kf <= k:  # queue new keyframes in stride-sized groups
                if u.next_kf == 0:
                    last = 0  # the first keyframe defines the robot's local frame
                else:
                    last = u.next_kf + stride - 1
                    if last > k:
                        break  # wait for a full group
                u.queue.append((u.next_kf, last))
                u.next_kf = last + 1
            tick = ch.budget_B(period, net.share(lt))
            avail = tick + carry.get((i, lt), 0.0)
            cap = max(4.0 * tick, float(ch.mtu_B))
            used = 0.0
            while u.queue or u.current is not None:
                if u.current is None or u.current[4] != lt:
                    if u.current is not None:  # link changed mid-keyframe: start over on lt
                        u.queue.appendleft((u.current[0], u.current[1]))
                    first, last = u.queue.popleft()
                    kfs = sim.agents[i].keyframes[first : last + 1]
                    ckf = compose_keyframes(kfs, dim)
                    size = keyframe_size_B(ckf, dim)
                    room = payload_room[lt]
                    nfrag = max(1, int(np.ceil(size / room)))
                    sizes = [
                        codec.PACKET_OVERHEAD + FRAG_HEADER.size + min(room, size - f * room)
                        for f in range(nfrag)
                    ]
                    composed[(i, last)] = (first, ckf)
                    u.current = [first, last, sizes, 0, lt]
                first, last, sizes, f, _ = u.current
                if used + sizes[f] > avail:
                    break  # the rest of this keyframe goes out on later ticks
                body = bytearray(sizes[f])
                FRAG_HEADER.pack_into(body, codec.HEADER_SIZE, FRAG_MAGIC, i, last, f, len(sizes))
                end = net.send(float(t), i, lt, bytes(body))
                used += sizes[f]
                u.current[3] += 1
                if u.current[3] == len(sizes):
                    u.in_flight[last] = (first, end + ch.latency_s(ch.max_range_m) + 1.0)
                    u.current = None
            carry[(i, lt)] = min(avail - used, cap)
        if server_id in server.shadows:  # the server's own robot: no uplink
            while server.fed[server_id] < k:
                j = server.fed[server_id] + 1
                server.receive(server_id, j, j, sim.agents[server_id].keyframes[j])
        accepted = server.update()
        for e in accepted:
            first_link.setdefault(f"{e.a}-{e.b}", float(t))
        if connected_s is None:
            edges = {(e.a, e.b): (e.T, e.sigma_xy) for e in accepted}
            if set(robots) <= set(chain_frames(anchor, edges)):
                connected_s = float(t)
    deliver(float(times[-1]) + 3600.0)  # let in-flight packets land (no new sends)
    accepted = server.update()
    edges = {(e.a, e.b): (e.T, e.sigma_xy) for e in accepted}
    T_anchor_from = chain_frames(anchor, edges)

    joint, keys = _joint_graph(server, accepted, T_anchor_from, anchor, params)
    if joint is not None:
        joint.optimize(max_iters=sp_.joint_iters)

    # Robots' own local SLAM (full rate) to fill in between received keyframes.
    own = _make_agents(scenario, sim, params, seed)
    for i, ag in own.items():
        for kf in sim.agents[i].keyframes:
            ag.on_keyframe(kf)
        ag.solve_local(with_marginals=False)
    local = {i: ag.trajectory("local") for i, ag in own.items()}
    team = {}
    for i in T_anchor_from:
        if joint is None or not keys.get(i):
            continue
        idx = server.kf_index[i]
        srv = {kk: joint.value(("x", i, n)) for n, kk in enumerate(idx)}
        tr = np.empty_like(local[i])
        for kk in range(len(local[i])):
            pos = int(np.searchsorted(idx, kk, side="right") - 1)
            s_k = idx[max(pos, 0)]
            rel = compose(inverse(local[i][s_k]), local[i][kk])
            tr[kk] = compose(srv[s_k], rel)
        team[i] = tr
    team_err = team_ate(team, _gt_trajs(sim), robots)
    delivered = {i: (server.fed[i] + 1) / len(sim.agents[i].keyframes) for i in robots}
    stats = {
        lt.name: {
            "packets_sent": st.packets_sent,
            "bytes_sent": st.bytes_sent,
            "deliveries": st.deliveries,
            "bytes_delivered": st.bytes_delivered,
            "losses": st.losses,
        }
        for lt, st in net.stats.items()
    }
    metrics = {
        "agents": _names(sim),
        "server_id": server_id,
        "ate_team_m": team_err.ate_m,
        "ate_team_per_agent_m": team_err.per_agent_m,
        "ate_local_m": _local_ates(local, sim),
        "connected": list(team_err.connected),
        "disconnected": list(team_err.disconnected),
        "team_connected_s": connected_s,
        "first_server_link_s": first_link,
        "delivered_fraction": delivered,
        "comm": stats,
    }
    return RunResult(scenario.name, "server", seed, sim.duration_s, metrics, team, local,
                     net.events)  # fmt: skip


def _joint_graph(server: Server, accepted, T_anchor_from, anchor: int, params: AvatarParams):
    """One graph over all robots connected to the anchor, with estimated association."""
    members = [i for i in T_anchor_from if server.shadows[i].k >= 0]
    if not members:
        return None, {}
    parent: dict[tuple, tuple] = {}

    def find(x):
        while parent.get(x, x) != x:
            x = parent[x]
        return x

    coax: list[tuple[tuple, tuple]] = []
    ok = {(e.a, e.b) for e in accepted}
    for (a, b), (res, ids_a, ids_b) in server.alignments.items():
        if (a, b) not in ok or a not in T_anchor_from or b not in T_anchor_from:
            continue
        for ia, ib, cross in res.pairs:
            ka, kb = ("l", a, ids_a[ia]), ("l", b, ids_b[ib])
            if cross:
                coax.append((ka, kb))
            else:
                ra, rb = find(ka), find(kb)
                if ra != rb:
                    parent[rb] = ra
    g = FactorGraph(point_obs_robust_k=params.point_obs_robust_k)
    if params.point_obs_gnc is not None:
        g.set_kernel("point_obs", "gnc", params.point_obs_gnc)
    keys: dict[int, list] = {}
    for i in members:
        sh = server.shadows[i]
        T = T_anchor_from[i]
        keys[i] = []
        for n in range(sh.k + 1):
            key = ("x", i, n)
            g.add_variable(key, VarType.POSE4, compose(T, sh.local.value(("x", n))))
            keys[i].append(key)
        for lid in sh.meta:
            lk = find(("l", i, lid))
            if not g.has(lk):
                p = transform_points(T, sh.local.value(("l", lid))[None, :])[0]
                g.add_variable(lk, VarType.POINT3, p)
    for i in members:
        sh = server.shadows[i]
        first = g.value(("x", i, 0))
        if i == anchor:
            g.add_pose_prior(("x", i, 0), first, [1e-3, 1e-3, params.initial_z_sigma_m, 1e-3])
        else:  # frame known only through association (weak prior keeps the gauge sane)
            g.add_pose_prior(("x", i, 0), first, [1e3, 1e3, params.initial_z_sigma_m, 1e3])
        bkey = ("b", i)
        if params.model_heading_bias and heading_bias_modelled(sh.cfg):
            g.add_variable(bkey, VarType.SCALAR, [0.0])
            g.add_scalar_prior(bkey, 0.0, sh.cfg.odometry_noise.yaw_bias_std_rad_per_m)
        for n, kf in enumerate(server.fed_keyframes[i]):
            key = ("x", i, n)
            if n > 0 and g.has(bkey):
                dist = float(np.linalg.norm(kf.odom[:3]))
                g.add_between_bias(("x", i, n - 1), key, bkey, kf.odom, dist, kf.odom_sigmas)
            elif n > 0:
                g.add_between(("x", i, n - 1), key, kf.odom, kf.odom_sigmas)
            if kf.abs_z is not None and kf.abs_z_sigma is not None:
                g.add_z_prior(key, kf.abs_z, kf.abs_z_sigma)
            for det in kf.detections:
                lid = sh._lid_of_part[det.part_index]
                g.add_point_obs(key, find(("l", i, lid)), det.p_body, det.sigmas)
        for parts in sh._object_parts.values():
            if len(parts) == 2:
                a = find(("l", i, parts[Medium.ABOVE]))
                b = find(("l", i, parts[Medium.BELOW]))
                if a != b:
                    g.add_coaxial(a, b, params.coaxial_sigma_m)
    for ka, kb in coax:
        a, b = find(ka), find(kb)
        if a != b and g.has(a) and g.has(b):
            g.add_coaxial(
                a, b, float(np.hypot(params.coaxial_sigma_m, params.cross_medium_model_sigma_m))
            )
    return g, keys
