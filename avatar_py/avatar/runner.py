"""Experiment runner: one scenario, one seed, one estimation mode.

Modes
-----
``independent``
    Each agent runs its local graph only (single-robot SLAM baseline).
``decentralized``
    Avatar SLAM v0: periodic condensed-landmark and frame-alignment exchange
    over the simulated heterogeneous network, then per-agent fused graphs.
``centralized``
    Oracle upper bound: all measurements in one graph with ground-truth data
    association (including coaxial links), unlimited communication.

All modes consume the same pre-generated measurements (paired comparison).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from avatar.agent import DEFAULT_LINK_RECEIVERS, AvatarAgent, AvatarParams, odometry_states
from avatar.backend.graph import FactorGraph, VarType
from avatar.comm.channel import ChannelModel
from avatar.comm.gateway import Gateway
from avatar.comm.network import Network
from avatar.eval.metrics import ate_rmse, frame_error, team_ate
from avatar.geometry import compose, inverse, transform_poses
from avatar.sim.measurements import SimData, generate_measurements
from avatar.sim.scenarios import SCENARIOS, Scenario
from avatar.types import Domain, LinkType, Medium

FloatArray = NDArray[np.float64]
MODES = ("independent", "decentralized", "centralized", "server")


@dataclass
class RunResult:
    """Outputs of one run (metrics are JSON-serialisable)."""

    scenario: str
    mode: str
    seed: int
    duration_s: float
    metrics: dict
    trajectories_team: dict[int, FloatArray] = field(default_factory=dict)
    trajectories_local: dict[int, FloatArray] = field(default_factory=dict)
    comm_events: list[dict] = field(default_factory=list)


def make_sim(
    scenario_name: str, seed: int, duration_s: float, params: AvatarParams, **kwargs
) -> tuple[Scenario, SimData]:
    """Build a scenario and pre-generate its measurements for ``seed``."""
    rng = np.random.default_rng(seed)
    scenario = SCENARIOS[scenario_name](rng, **kwargs)
    sim = generate_measurements(
        scenario.world,
        scenario.agents,
        duration_s,
        1.0,
        params.descriptor_dim,
        rng,
        frontend_errors=scenario.frontend_errors,
        frontend_rng=np.random.default_rng(seed + 30_000),
    )
    return scenario, sim


def _gt_frames(sim: SimData) -> dict[int, FloatArray]:
    return {i: a.T_world_from_local for i, a in sim.agents.items()}


def _gt_trajs(sim: SimData) -> dict[int, FloatArray]:
    return {i: a.gt for i, a in sim.agents.items()}


def _local_ates(trajs: dict[int, FloatArray], sim: SimData) -> dict[int, float]:
    return {i: ate_rmse(t, sim.agents[i].gt) for i, t in trajs.items()}


def _names(sim: SimData) -> dict[int, str]:
    return {i: a.config.name for i, a in sim.agents.items() if a.config.role == "slam"}


def _slam_ids(scenario: Scenario) -> list[int]:
    return [a.agent_id for a in scenario.agents if a.role == "slam"]


def _make_agents(scenario: Scenario, sim: SimData, params: AvatarParams, seed: int):
    obj_ids = np.array([p.object_id for p in sim.world.parts], dtype=np.int64)
    zrng = np.random.default_rng(seed + 10_000)
    agents = {}
    for cfg in scenario.agents:
        if cfg.role != "slam":
            continue
        z0 = float(sim.agents[cfg.agent_id].gt[0, 2] + zrng.normal(0.0, params.initial_z_sigma_m))
        agents[cfg.agent_id] = AvatarAgent(
            cfg, params, obj_ids, z0, np.random.default_rng(seed * 1000 + cfg.agent_id)
        )
        agents[cfg.agent_id].link_receivers = link_receivers(scenario, cfg.agent_id)
    return agents


def link_receivers(scenario: Scenario, sender_id: int) -> dict[LinkType, tuple[Domain, ...]]:
    """Domains of the SLAM agents a sender reaches on each of its links.

    A packet reaches the other SLAM agents on the same link and, through a
    gateway that carries this link, the SLAM agents on the gateway's other links.
    The VoI scheduler (``avatar.comm.scheduler``) uses this to weight records by
    whether any receiver can match them.
    """
    sender = next(a for a in scenario.agents if a.agent_id == sender_id)
    slam = [a for a in scenario.agents if a.role == "slam" and a.agent_id != sender_id]
    bridges = [set(g.comm) for g in scenario.agents if g.role == "gateway"]
    out: dict[LinkType, tuple[Domain, ...]] = {}
    for link in sender.comm:
        reach = {link}.union(*[b for b in bridges if link in b])
        doms = {a.domain for a in slam if reach & set(a.comm)}
        out[link] = tuple(sorted(doms, key=lambda d: d.value)) or DEFAULT_LINK_RECEIVERS[link]
    return out


@dataclass
class TokenBuckets:
    """One node's send budget per link: each exchange adds its allowance, and unused bytes carry
    over, capped at four exchanges' worth (or one MTU)."""

    channels: dict[LinkType, ChannelModel]
    shares: dict[LinkType, float]  # the node's fraction of each link's bit rate
    period_s: float
    carry: dict[LinkType, float] = field(default_factory=dict)

    def budget(self, link: LinkType) -> tuple[float, float]:
        """This exchange's allowance plus the carry-over, and the cap of the carry-over [B]."""
        ch = self.channels[link]
        tick = ch.budget_B(self.period_s, self.shares[link])
        return tick + self.carry.get(link, 0.0), max(4.0 * tick, float(ch.mtu_B))


def _fit(packets: list[bytes], avail: float) -> list[bytes]:
    """The leading packets whose bytes fit in ``avail``."""
    out, used = [], 0
    for pkt in packets:
        if used + len(pkt) > avail:
            break
        out.append(pkt)
        used += len(pkt)
    return out


def agent_packets(
    ag: AvatarAgent, buckets: TokenBuckets, t: float, z_now: float
) -> list[tuple[LinkType, bytes]]:
    """An agent's packets of one exchange, per link: landmark digests, then frame alignments,
    within the budget (``z_now``: its depth, which tells it which media it can reach)."""
    out = []
    for link in ag.cfg.comm:
        ch = buckets.channels[link]
        avail, cap = buckets.budget(link)
        if not ch.medium_ok(z_now, z_now):
            # e.g. RF while submerged: the vehicle knows its depth and stays silent; its budget
            # keeps accruing for the next surfacing window.
            buckets.carry[link] = min(avail, cap)
            continue
        digests = _fit(ag.build_digests(t, link, int(avail), ch.mtu_B), avail)
        used = sum(map(len, digests))
        aligns = _fit(ag.build_alignment_messages(t), avail - used)
        used += sum(map(len, aligns))
        buckets.carry[link] = min(avail - used, cap)
        out += [(link, pkt) for pkt in digests + aligns]
    return out


def gateway_packets(
    gw: Gateway, comm: tuple[LinkType, ...], buckets: TokenBuckets, t: float
) -> list[tuple[LinkType, bytes]]:
    """A gateway's relayed packets of one exchange, per link, within the budget."""
    out = []
    for link in comm:
        ch = buckets.channels[link]
        avail, cap = buckets.budget(link)
        relayed = _fit(gw.build_packets(t, link, int(avail), ch.mtu_B), avail)
        buckets.carry[link] = min(avail - sum(map(len, relayed)), cap)
        out += [(link, pkt) for pkt in relayed]
    return out


def make_network(scenario: Scenario, sim: SimData, seed: int) -> Network:
    """The links of a decentralized run: the scenario's channels and memberships, ground-truth
    positions at keyframe times, losses drawn from ``seed + 20000`` (also the ROS 2 comm
    emulator's, T-S3-01)."""
    times = sim.agents[scenario.agents[0].agent_id].times

    def position(agent_id: int, t: float) -> FloatArray:
        k = int(np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 1))
        return sim.agents[agent_id].gt[k, :3]

    return Network(
        channels=scenario.channels,
        memberships={a.agent_id: a.comm for a in scenario.agents},
        position_fn=position,
        rng=np.random.default_rng(seed + 20_000),
    )


def run_independent(scenario: Scenario, sim: SimData, params: AvatarParams, seed: int) -> RunResult:
    """Single-agent SLAM for every agent (no communication).

    Same estimator and schedule as each agent's local graph in
    :func:`run_decentralized`: incremental solves every exchange period and a
    final solve. (A single batch solve of ``local_iters`` iterations can stop
    short of the optimum on long, drifting trajectories; LOG L24.)
    """
    agents = _make_agents(scenario, sim, params, seed)
    for i, ag in agents.items():
        times = sim.agents[i].times
        next_solve = params.exchange_period_s
        for kf, t in zip(sim.agents[i].keyframes, times, strict=True):
            ag.on_keyframe(kf)
            if t + 1e-9 >= next_solve:
                next_solve += params.exchange_period_s
                ag.solve_local(with_marginals=False)
        ag.solve_local(with_marginals=False)
    trajs = {i: ag.trajectory("local") for i, ag in agents.items()}
    metrics = {"ate_local_m": _local_ates(trajs, sim), "agents": _names(sim)}
    return RunResult(scenario.name, "independent", seed, sim.duration_s, metrics, {}, trajs)


def run_decentralized(
    scenario: Scenario, sim: SimData, params: AvatarParams, seed: int
) -> RunResult:
    """Avatar SLAM v0 over the simulated heterogeneous network."""
    agents = _make_agents(scenario, sim, params, seed)
    anchor_id = scenario.anchor_id
    if anchor_id not in agents:
        raise ValueError("the anchor must be a SLAM agent")
    times = sim.agents[scenario.agents[0].agent_id].times
    net = make_network(scenario, sim, seed)
    gateways = {a.agent_id: Gateway(a.agent_id) for a in scenario.agents if a.role == "gateway"}
    period = params.exchange_period_s
    next_exchange = period
    shares = {link: net.share(link) for link in scenario.channels}
    buckets = {
        a.agent_id: TokenBuckets(scenario.channels, shares, period)
        for a in scenario.agents
        if a.agent_id in agents or a.agent_id in gateways
    }
    first_align: dict[int, dict[int, float]] = {i: {} for i in agents}
    team_connected_s: float | None = None
    slam_ids = set(agents)

    def deliver(t_now: float) -> None:
        for dlv in net.pop_until(t_now):
            if dlv.receiver in gateways:
                gateways[dlv.receiver].on_packet(dlv.link_type, dlv.payload)
            else:
                agents[dlv.receiver].on_packet(dlv.payload)

    for k, t in enumerate(times):
        for i, ag in agents.items():
            ag.on_keyframe(sim.agents[i].keyframes[k])
        deliver(t)
        if t + 1e-9 >= next_exchange:
            next_exchange += period
            for ag in agents.values():
                ag.solve_local()
                ag.update_alignments()
                ag.solve_fused()
            for i, ag in agents.items():
                for j in ag.alignments:
                    first_align[i].setdefault(j, float(t))
            if team_connected_s is None:
                # chain over every consistent estimate, incl. neighbours' estimates of the anchor
                if slam_ids <= set(agents[anchor_id].team_frames(fuse=False)):
                    team_connected_s = float(t)
            for i, ag in agents.items():
                for link, pkt in agent_packets(ag, buckets[i], t, float(sim.agents[i].gt[k, 2])):
                    net.send(t, i, link, pkt)
            for g, gw in gateways.items():
                for link, pkt in gateway_packets(gw, scenario.agent(g).comm, buckets[g], t):
                    net.send(t, g, link, pkt)
    deliver(float(times[-1]))
    for ag in agents.values():
        ag.solve_local()
        ag.update_alignments()
        ag.solve_fused()

    anchor = scenario.anchor_id
    T_anchor_from = agents[anchor].team_frames()
    fused = {i: ag.trajectory("fused") for i, ag in agents.items()}
    local = {i: ag.trajectory("local") for i, ag in agents.items()}
    team = {i: transform_poses(T_anchor_from[i], fused[i]) for i in T_anchor_from if i in fused}
    gt_frames = _gt_frames(sim)
    team_err = team_ate(team, _gt_trajs(sim), list(agents))
    ferr = {}
    for j, T in T_anchor_from.items():
        if j == anchor:
            continue
        T_gt = compose(inverse(gt_frames[anchor]), gt_frames[j])
        e_xy, e_yaw = frame_error(T, T_gt)
        ferr[j] = {"xy_m": e_xy, "yaw_deg": float(np.rad2deg(e_yaw))}
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
        "ate_local_m": _local_ates(local, sim),
        "ate_fused_m": _local_ates(fused, sim),
        "ate_team_m": team_err.ate_m,
        "ate_team_per_agent_m": team_err.per_agent_m,
        "connected": list(team_err.connected),
        "disconnected": list(team_err.disconnected),
        "frame_error": ferr,
        "alignments": {
            i: {j: a.n_inliers for j, a in ag.alignments.items()} for i, ag in agents.items()
        },
        "comm": stats,
        "decode_errors": sum(ag.decode_errors for ag in agents.values()),
        "first_alignment_s": first_align,
        "vetoed_alignments": sum(len(ag.vetoed) for ag in agents.values()),
        # own alignments not used: vetoed by the cycle check or weak and unconfirmed (T-F3-05)
        "unused_alignments": {i: sorted(ag.vetoed | ag.unconfirmed) for i, ag in agents.items()},
        # Error of every accepted pairwise alignment vs ground truth (diagnostic).
        "alignment_errors": {
            i: {
                j: dict(
                    zip(
                        ("xy_m", "yaw_rad"),
                        frame_error(
                            a.T_mine_from_remote,
                            compose(inverse(_gt_frames(sim)[i]), _gt_frames(sim)[j]),
                        ),
                        strict=True,
                    )
                )
                for j, a in ag.alignments.items()
            }
            for i, ag in agents.items()
        },
        "team_connected_s": team_connected_s,
        "gateways": {g: dict(gw.stats) for g, gw in gateways.items()},
    }
    # Express team trajectories in world frame for visualisation (anchor GT frame).
    team_world = {i: transform_poses(gt_frames[anchor], tr) for i, tr in team.items()}
    return RunResult(
        scenario.name,
        "decentralized",
        seed,
        sim.duration_s,
        metrics,
        team_world,
        local,
        net.events,
    )


def run_centralized(
    scenario: Scenario, sim: SimData, params: AvatarParams, seed: int, robust: bool = False
) -> RunResult:
    """Oracle: one graph, ground-truth association, unlimited communication.

    ``robust`` applies the agents' landmark-observation kernel (GNC-TLS,
    ``params.point_obs_gnc``) in the oracle too. Tier-1 detections of a true part
    have no gross errors, so the default stays off there; Tier-2 detections of a
    true part can still be off by up to the labelling radius (partial views).
    """
    g = FactorGraph()
    if robust and params.point_obs_gnc is not None:
        g.set_kernel("point_obs", "gnc", params.point_obs_gnc)
    gt_frames = _gt_frames(sim)
    anchor = scenario.anchor_id
    parts = sim.world.parts
    part_key = {i: ("l", p.object_id, int(p.medium)) for i, p in enumerate(parts)}
    for i, ad in sim.agents.items():
        if ad.config.role != "slam":
            continue
        est = None
        for k, kf in enumerate(ad.keyframes):
            key = ("x", i, k)
            if k == 0:
                init = gt_frames[i].copy()
                init[2] = kf.abs_z if kf.abs_z is not None else ad.gt[0, 2]
                g.add_variable(key, VarType.POSE4, init)
                if i == anchor:
                    g.add_pose_prior(key, init, [1e-3, 1e-3, params.initial_z_sigma_m, 1e-3])
                else:  # initial frames known only for initialisation (weak prior)
                    g.add_pose_prior(key, init, [1e3, 1e3, params.initial_z_sigma_m, 1e3])
                est = init
            else:
                est = compose(est, kf.odom)
                if kf.abs_z is not None:
                    est[2] = kf.abs_z
                g.add_variable(key, VarType.POSE4, est)
                odo = odometry_states(g, ad.config, params, i)
                if odo is not None:
                    dist = float(np.linalg.norm(kf.odom[:3]))
                    b, s, sg = odo
                    g.add_between_bias(
                        ("x", i, k - 1), key, b, kf.odom, dist, kf.odom_sigmas, s, sg
                    )
                else:
                    g.add_between(("x", i, k - 1), key, kf.odom, kf.odom_sigmas)
            if kf.abs_z is not None:
                g.add_z_prior(key, kf.abs_z, kf.abs_z_sigma)
            for det in kf.detections:
                true_part = det.part_index if det.true_part_index is None else det.true_part_index
                if true_part < 0:
                    continue  # clutter: the oracle's association knows it is not a part
                lk = part_key[true_part]
                if not g.has(lk):
                    g.add_variable(lk, VarType.POINT3, compose(est, [*det.p_body, 0.0])[:3])
                g.add_point_obs(key, lk, det.p_body, det.sigmas)
    for s in sim.world.structures:
        a, b = ("l", s.object_id, int(Medium.ABOVE)), ("l", s.object_id, int(Medium.BELOW))
        if g.has(a) and g.has(b):
            g.add_coaxial(a, b, params.coaxial_sigma_m)
    g.optimize(max_iters=30)
    trajs = {
        i: np.array([g.value(("x", i, k)) for k in range(len(ad.keyframes))])
        for i, ad in sim.agents.items()
        if ad.config.role == "slam"
    }
    team_err = team_ate(trajs, _gt_trajs(sim), list(trajs))
    local = {i: transform_poses(inverse(gt_frames[i]), tr) for i, tr in trajs.items()}
    metrics = {
        "agents": _names(sim),
        "ate_local_m": _local_ates(trajs, sim),
        "ate_team_m": team_err.ate_m,
        "ate_team_per_agent_m": team_err.per_agent_m,
        "connected": list(team_err.connected),
        "disconnected": [],
    }
    return RunResult(scenario.name, "centralized", seed, sim.duration_s, metrics, trajs, local)


def run_server(scenario: Scenario, sim: SimData, params: AvatarParams, seed: int) -> RunResult:
    """*A&B*-style centralized server (``avatar.baselines.centralized_server``)."""
    from avatar.baselines.centralized_server import run_server as _run

    return _run(scenario, sim, params, seed)


RUNNERS = {
    "independent": run_independent,
    "decentralized": run_decentralized,
    "centralized": run_centralized,
    "server": run_server,
}


def run(
    scenario_name: str,
    mode: str,
    seed: int = 0,
    duration_s: float = 300.0,
    params: AvatarParams | None = None,
    **scenario_kwargs,
) -> RunResult:
    """Build, simulate and estimate; returns metrics (and wall time in ``metrics``)."""
    if mode not in RUNNERS:
        raise ValueError(f"mode must be one of {MODES}")
    params = params or AvatarParams()
    scenario, sim = make_sim(scenario_name, seed, duration_s, params, **scenario_kwargs)
    t0 = time.perf_counter()
    res = RUNNERS[mode](scenario, sim, params, seed)
    res.metrics["wall_time_s"] = time.perf_counter() - t0
    return res
