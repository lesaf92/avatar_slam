"""Gateway node (T-S3-02): the quay-side relay, ``avatar.comm.gateway.Gateway``, in ROS 2.

In: the packets delivered to it (``/avatar/<gateway>/rx``) and ``/clock``. Out: relayed packets on
``/avatar/comm/tx``, every ``exchange_period_s`` within its budget, with the offline runner's code
(``avatar.runner.gateway_packets``).
"""

from __future__ import annotations

import rclpy
from avatar_msgs.msg import EncodedPacket
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from avatar.agent import AvatarParams
from avatar.comm.gateway import Gateway
from avatar.runner import TokenBuckets, gateway_packets, make_network
from avatar.types import LinkType
from avatar_sim.common import QOS, scenario_of, seconds, set_stamp


class GatewayNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gateway", **kwargs)
        scenario, sim, seed = scenario_of(self)
        name = self.declare_parameter("agent", "gw_0").value
        self.cfg = next(a for a in scenario.agents if a.name == name)
        self.gw = Gateway(self.cfg.agent_id)
        period = AvatarParams().exchange_period_s
        net = make_network(scenario, sim, seed)
        self.buckets = TokenBuckets(
            scenario.channels, {lk: net.share(lk) for lk in scenario.channels}, period
        )
        self.period, self.next_exchange = period, period
        self.tx = self.create_publisher(EncodedPacket, "/avatar/comm/tx", QOS)
        self.create_subscription(EncodedPacket, f"/avatar/{name}/rx", self._on_rx, QOS)
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_rx(self, msg: EncodedPacket) -> None:
        self.gw.on_packet(LinkType(msg.link_type), bytes(msg.payload))

    def _on_clock(self, msg: Clock) -> None:
        t = seconds(msg.clock)
        if t + 1e-9 < self.next_exchange:
            return
        while self.next_exchange <= t + 1e-9:
            self.next_exchange += self.period
        for link, pkt in gateway_packets(self.gw, self.cfg.comm, self.buckets, t):
            out = EncodedPacket()
            set_stamp(out.header.stamp, t)
            out.sender_id, out.link_type, out.payload = self.cfg.agent_id, int(link), list(pkt)
            self.tx.publish(out)


def main() -> None:
    rclpy.init()
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
