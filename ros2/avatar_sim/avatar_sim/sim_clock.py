"""Simulated clock (T-S3-02): ``/clock`` at a fixed rate, free-running (ADR-0010).

Publishes ``/clock`` from 0 in steps of ``step_s`` simulated seconds at ``rate`` simulated seconds
per wall second, up to ``end_s`` plus ``tail_s`` (time for the last packets and solves), then
exits. It starts once ``subscribers`` nodes listen to ``/clock`` (a message sent before a
subscriber is matched is lost to it). In Gazebo, Gazebo's clock replaces it. The rate must be
low enough that every node keeps up: a node that falls behind skips exchanges, as a slow robot
would.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from avatar_sim.common import QOS, set_stamp


class SimClock(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("sim_clock", **kwargs)
        self.step = float(self.declare_parameter("step_s", 0.5).value)
        rate = float(self.declare_parameter("rate", 5.0).value)
        self.stop = float(self.declare_parameter("end_s", 600.0).value) + float(
            self.declare_parameter("tail_s", 30.0).value
        )
        self.subscribers = int(self.declare_parameter("subscribers", 0).value)
        self.t = 0.0
        self.pub = self.create_publisher(Clock, "/clock", QOS)
        self.create_timer(self.step / rate, self._tick)

    @property
    def finished(self) -> bool:
        return self.t > self.stop

    def _tick(self) -> None:
        if self.finished or self.pub.get_subscription_count() < self.subscribers:
            return
        msg = Clock()
        set_stamp(msg.clock, self.t)
        self.pub.publish(msg)
        self.t += self.step


def main() -> None:
    rclpy.init()
    node = SimClock()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
