"""ROS 2 comm emulator (task T-S3-01): Avatar's inter-agent links in simulation, as a node.

A thin ``rclpy`` wrapper (ADR-0009) around :class:`avatar.comm.network.Network`, the link model of
the Tier-1 runner (``avatar.runner.make_network``). Every agent publishes its wire packets on
``/avatar/comm/tx`` (``avatar_msgs/EncodedPacket``: ``sender_id``, ``link_type``, ``header.stamp``
= transmission time). The node applies the scenario's channels (medium gating, range, loss,
latency, airtime with TDMA shares, half-duplex queueing) and publishes each delivery on
``/avatar/<receiver>/rx`` once ``/clock`` passes its arrival time (stamp = arrival time).

Positions are the scenario's ground truth (``make_sim``), where the kinematic Gazebo rigs are
placed (ADR-0007). One shared ``tx`` topic keeps the transmissions in one order, so with the same
seed the losses are those of the Tier-1 run that sent the same packets (``test/``).

    ros2 run avatar_sim comm_emulator --ros-args -p scenario:=harbor_fleet -p seed:=0 \\
        -p scenario_args:="{acoustic: m64}" -p use_sim_time:=true

Parameters: ``scenario`` (default ``harbor_fleet``), ``scenario_args`` (a YAML mapping, as the
``args`` of ``experiments/scenarios/*.yaml``), ``seed``, ``duration_s``.
"""

from __future__ import annotations

import rclpy
import yaml
from avatar_msgs.msg import EncodedPacket
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from avatar.agent import AvatarParams
from avatar.runner import make_network, make_sim
from avatar.types import LinkType

# Reliable and deep: a dropped tx message would be a loss the channel model did not draw.
QOS = QoSProfile(depth=10_000, reliability=ReliabilityPolicy.RELIABLE)


def seconds(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


class CommEmulator(Node):
    """``/avatar/comm/tx`` -> channel models -> ``/avatar/<name>/rx``."""

    def __init__(self, **kwargs) -> None:
        super().__init__("comm_emulator", **kwargs)
        name = self.declare_parameter("scenario", "harbor_fleet").value
        args = yaml.safe_load(self.declare_parameter("scenario_args", "{}").value) or {}
        seed = int(self.declare_parameter("seed", 0).value)
        duration = float(self.declare_parameter("duration_s", 600.0).value)
        scenario, sim = make_sim(name, seed, duration, AvatarParams(), **args)
        self.net = make_network(scenario, sim, seed)
        self.received = 0
        self.dropped = 0  # transmissions the scenario does not allow (no such link, too long)
        self._rx = {
            a.agent_id: self.create_publisher(EncodedPacket, f"/avatar/{a.name}/rx", QOS)
            for a in scenario.agents
        }
        self.create_subscription(EncodedPacket, "/avatar/comm/tx", self._on_tx, QOS)
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_tx(self, msg: EncodedPacket) -> None:
        self.received += 1
        try:
            self.net.send(seconds(msg.header.stamp), msg.sender_id, LinkType(msg.link_type),
                          bytes(msg.payload))  # fmt: skip
        except ValueError as err:
            self.dropped += 1
            self.get_logger().warning(f"dropped a packet of agent {msg.sender_id}: {err}")

    def _on_clock(self, msg: Clock) -> None:
        for d in self.net.pop_until(seconds(msg.clock)):
            out = EncodedPacket()
            t = float(d.t_arrival)
            out.header.stamp.sec = int(t)
            out.header.stamp.nanosec = round((t - int(t)) * 1e9) % 10**9
            out.sender_id = d.sender
            out.link_type = int(d.link_type)
            out.payload = list(d.payload)
            self._rx[d.receiver].publish(out)

    def stats(self) -> dict[str, dict[str, int]]:
        """Per link: packets and bytes sent, deliveries, bytes delivered, losses."""
        return {lt.name: dict(vars(st)) for lt, st in self.net.stats.items()}


def main() -> None:
    rclpy.init()
    node = CommEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"link statistics: {node.stats()}")
        node.destroy_node()
        rclpy.try_shutdown()
