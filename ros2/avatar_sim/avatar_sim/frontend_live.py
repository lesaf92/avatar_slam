"""Live front-end (T-S3-03): one robot's Tier-2 front-end on its sensor topics, in ROS 2.

Subscribes to ``/avatar/<agent>/<sensor>/image`` for each of its rig's sensors (``sensor_replay``
or the Gazebo rigs), runs :class:`avatar.tier2.frontend.AgentFrontEnd` on each keyframe once all
its images are in, and publishes ``/avatar/<agent>/keyframe``. With the EKF tracker a detection
counts once its landmark has proved static: a keyframe leaves at once with the detections already
released, and the others follow when they are, as amendments (a ``Keyframe`` with an earlier
index and no odometry; :class:`~avatar.tier2.frontend.LateRelease`).

Odometry and depth are those of the Tier-1 simulation of the run, as offline; ground truth only
labels detections (the simulated classifier). Simulation only: on a robot, odometry comes from
its own nodes.
"""

from __future__ import annotations

import numpy as np
import rclpy
from avatar_msgs.msg import Keyframe
from rclpy.node import Node
from sensor_msgs.msg import Image

from avatar.sim.measurements import KeyframeData
from avatar.tier2.dataset import TRACK_ID_STRIDE
from avatar.tier2.frontend import AgentFrontEnd, FrontEndParams, LateRelease
from avatar.tier2.sdf import GZ_SENSORS, rig_sensors
from avatar_sim.common import QOS, image_array, keyframe_msg, scenario_of, seconds


class FrontendLive(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("frontend_live", **kwargs)
        scenario, sim, seed = scenario_of(self)
        tracking = self.get_parameter("tracking").value
        name = self.declare_parameter("agent", "").value
        aid = next(a.agent_id for a in scenario.agents if a.name == name)
        self.ad = sim.agents[aid]
        sensors = rig_sensors(scenario)[name]
        fe = AgentFrontEnd(
            self.ad, {s: GZ_SENSORS[s] for s in sensors}, sim.world, sim.instance_descriptors,
            len(sim.world.parts) + TRACK_ID_STRIDE * (aid + 1), FrontEndParams(tracking=tracking),
            np.random.default_rng(seed + 40_000 + aid),
        )  # fmt: skip
        self.fe = fe
        self.release = LateRelease(fe)
        self.frames: dict[int, dict[str, np.ndarray]] = {}
        self.sensors, self.next_k, self.outbox, self.sent = sensors, 0, [], 0
        self.pub = self.create_publisher(Keyframe, f"/avatar/{name}/keyframe", QOS)
        for s in sensors:
            self.create_subscription(
                Image, f"/avatar/{name}/{s}/image", lambda m, s=s: self._on_image(s, m), QOS
            )
        self.create_timer(0.2, self._send)  # keyframes held for a late agent still leave

    def _on_image(self, sensor: str, msg: Image) -> None:
        k = int(np.searchsorted(self.ad.times, seconds(msg.header.stamp) - 1e-6))
        self.frames.setdefault(k, {})[sensor] = image_array(msg)
        while len(self.frames.get(self.next_k, {})) == len(self.sensors):
            k = self.next_k
            self.outbox += self.release.push(k, self.fe.step(k, self.frames.pop(k)))
            self.next_k += 1
        self._send()

    def _send(self) -> None:
        if not self.outbox or self.pub.get_subscription_count() == 0:
            return  # hold the keyframes until the agent listens: each is relative to the last
        for k, dets in self.outbox:
            kf = self.ad.keyframes[k]
            if k < self.sent:  # an amendment: detections of a keyframe already sent
                data = KeyframeData(kf.t, None, None, dets, None, None)
            else:
                data = KeyframeData(kf.t, kf.odom, kf.odom_sigmas, dets, kf.abs_z, kf.abs_z_sigma)
                self.sent = k + 1
            self.pub.publish(keyframe_msg(data, k))
        self.outbox = []


def main() -> None:
    rclpy.init()
    node = FrontendLive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
