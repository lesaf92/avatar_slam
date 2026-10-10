"""Sensor replay (T-S3-03): a Tier-2 recording's sensor frames as ROS 2 images, like a rosbag.

Publishes ``/avatar/<agent>/<sensor>/image`` (``sensor_msgs/Image``, ``32FC1``: a LiDAR's range
scan, (vertical, horizontal) rays, or a depth image [m]) for every keyframe whose time ``/clock``
has reached. A live front-end (``frontend_live``) subscribes to them; a node that drives the
Gazebo rigs, or a robot's drivers, publishes the same topics.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from avatar.tier2.dataset import load_meta
from avatar_sim.common import QOS, image_msg, scenario_of, seconds


class SensorReplay(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("sensor_replay", **kwargs)
        scenario, sim, _ = scenario_of(self)
        run_dir = self.get_parameter("run_dir").value
        name = self.declare_parameter("agent", "").value
        aid = next(a.agent_id for a in scenario.agents if a.name == name)
        self.times = sim.agents[aid].times
        raw = np.load(Path(run_dir) / "raw.npz")
        self.frames = {s: raw[f"{name}/{s}"] for s in load_meta(run_dir)["rigs"][name]}
        self.pubs = {
            s: self.create_publisher(Image, f"/avatar/{name}/{s}/image", QOS) for s in self.frames
        }
        self.next = 0
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_clock(self, msg: Clock) -> None:
        if any(p.get_subscription_count() == 0 for p in self.pubs.values()):
            return  # hold the frames until the front-end listens
        t = seconds(msg.clock)
        while self.next < len(self.times) and self.times[self.next] <= t + 1e-9:
            for s, pub in self.pubs.items():
                pub.publish(image_msg(self.frames[s][self.next], self.times[self.next]))
            self.next += 1


def main() -> None:
    rclpy.init()
    node = SensorReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
