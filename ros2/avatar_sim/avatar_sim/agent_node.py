"""Agent node (T-S3-02): one robot's Avatar backbone, ``avatar.agent.AvatarAgent``, in ROS 2.

In: its keyframes (``/avatar/<agent>/keyframe``, ``avatar_msgs/Keyframe``), the packets the links
deliver to it (``/avatar/<agent>/rx``) and ``/clock``. Out: its wire packets on
``/avatar/comm/tx``. Every ``exchange_period_s`` of simulated time it solves its graphs, updates
its alignments and sends what its budget allows, with the offline runner's code
(``avatar.runner.agent_packets``). Time runs free (ADR-0010): it acts on what has arrived.

At ``end_s`` plus ``finish_delay_s`` (the last keyframe may still be in transit) it solves once
more and, with ``out``, writes its estimate as JSON: the frames of the
agents it knows in its own frame (``team_frames``), its fused and local trajectories.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from avatar_msgs.msg import EncodedPacket, Keyframe
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from avatar.agent import AvatarParams
from avatar.runner import TokenBuckets, _make_agents, agent_packets, make_network
from avatar_sim.common import QOS, keyframe_data, scenario_of, seconds, set_stamp


class AgentNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("agent", **kwargs)
        scenario, sim, seed = scenario_of(self)
        name = self.declare_parameter("agent", "").value
        self.out = self.declare_parameter("out", "").value
        cfg = next(a for a in scenario.agents if a.name == name)
        times = sim.agents[cfg.agent_id].times
        self.end_s = float(self.declare_parameter("end_s", float(times[-1])).value) + float(
            self.declare_parameter("finish_delay_s", 2.0).value
        )
        params = AvatarParams()
        self.ag = _make_agents(scenario, sim, params, seed)[cfg.agent_id]
        net = make_network(scenario, sim, seed)
        self.buckets = TokenBuckets(
            scenario.channels, {lk: net.share(lk) for lk in scenario.channels},
            params.exchange_period_s,
        )  # fmt: skip
        self.period = params.exchange_period_s
        self.next_exchange = self.period
        self.z_now = float(sim.agents[cfg.agent_id].gt[0, 2])  # until a depth arrives
        self.done = False
        self.exchanges = 0
        self.tx = self.create_publisher(EncodedPacket, "/avatar/comm/tx", QOS)
        self.create_subscription(Keyframe, f"/avatar/{name}/keyframe", self._on_keyframe, QOS)
        self.create_subscription(EncodedPacket, f"/avatar/{name}/rx", self._on_rx, QOS)
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_keyframe(self, msg: Keyframe) -> None:
        if msg.index != self.ag.k + 1:
            self.get_logger().warning(f"keyframe {msg.index} after {self.ag.k}: skipped")
            return
        self.ag.on_keyframe(keyframe_data(msg))
        if msg.has_abs_z:
            self.z_now = msg.abs_z

    def _on_rx(self, msg: EncodedPacket) -> None:
        self.ag.on_packet(bytes(msg.payload))

    def _on_clock(self, msg: Clock) -> None:
        t = seconds(msg.clock)
        if self.done or self.ag.k < 0:
            return
        if t + 1e-9 >= self.next_exchange:
            while self.next_exchange <= t + 1e-9:  # a late node skips, it does not catch up
                self.next_exchange += self.period
            self._solve()
            for link, pkt in agent_packets(self.ag, self.buckets, t, self.z_now):
                out = EncodedPacket()
                set_stamp(out.header.stamp, t)
                out.sender_id, out.link_type, out.payload = self.ag.id, int(link), list(pkt)
                self.tx.publish(out)
            self.exchanges += 1
        if t >= self.end_s:
            self.finish()

    def _solve(self) -> None:
        self.ag.solve_local()
        self.ag.update_alignments()
        self.ag.solve_fused()

    def finish(self) -> None:
        """Last solve; write the estimate (once)."""
        if self.done or self.ag.k < 0:
            return
        self.done = True
        self._solve()
        if self.out:
            est = {
                "agent": self.ag.cfg.name,
                "agent_id": self.ag.id,
                "keyframes": self.ag.k + 1,
                "exchanges": self.exchanges,
                "team_frames": {
                    str(j): [float(v) for v in T] for j, T in self.ag.team_frames().items()
                },
                "fused": self.ag.trajectory("fused").tolist(),
                "local": self.ag.trajectory("local").tolist(),
            }
            Path(self.out).parent.mkdir(parents=True, exist_ok=True)
            Path(self.out).write_text(json.dumps(est))


def main() -> None:
    rclpy.init()
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()
