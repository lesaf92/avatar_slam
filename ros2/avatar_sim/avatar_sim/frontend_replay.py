"""Replay front-end (T-S3-02): one agent's keyframes, as an offline run computed them, in ROS 2.

Publishes ``/avatar/<agent>/keyframe`` (``avatar_msgs/Keyframe``) for every keyframe whose time
``/clock`` has reached: a Tier-1 simulation's detections, or with ``run_dir`` a Tier-2 recording's
offline front-end output (``tracking``, ``sonar``). A live front-end node publishes the same
message from sensor topics (ADR-0010).
"""

from __future__ import annotations

import rclpy
from avatar_msgs.msg import Keyframe
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from avatar_sim.common import QOS, keyframe_msg, scenario_of, seconds


class FrontendReplay(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("frontend_replay", **kwargs)
        scenario, sim, _ = scenario_of(self, detections=True)
        name = self.declare_parameter("agent", "").value
        aid = next(a.agent_id for a in scenario.agents if a.name == name)
        self.kfs, self.times = sim.agents[aid].keyframes, sim.agents[aid].times
        self.next = 0
        self.pub = self.create_publisher(Keyframe, f"/avatar/{name}/keyframe", QOS)
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_clock(self, msg: Clock) -> None:
        if self.pub.get_subscription_count() == 0:
            return  # hold the keyframes until the agent listens: each is relative to the last
        t = seconds(msg.clock)
        while self.next < len(self.kfs) and self.times[self.next] <= t + 1e-9:
            self.pub.publish(keyframe_msg(self.kfs[self.next], self.next))
            self.next += 1


def main() -> None:
    rclpy.init()
    node = FrontendReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
