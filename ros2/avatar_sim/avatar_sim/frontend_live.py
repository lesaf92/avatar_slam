"""Live front-end (T-S3-03): one robot's Tier-2 front-end on its sensor topics, in ROS 2.

Subscribes to ``/avatar/<agent>/<sensor>/image`` for each of its rig's sensors (``sensor_replay``
or the Gazebo rigs), runs :class:`avatar.tier2.frontend.AgentFrontEnd` on each keyframe once all
its images are in, and publishes ``/avatar/<agent>/keyframe``. With the EKF tracker a keyframe
leaves ``release_delay_kf`` keyframes later, with the detections whose landmark has proved static
by then (:class:`~avatar.tier2.frontend.DelayedRelease`). Odometry and depth are those of the
Tier-1 simulation of the run, as offline; ground truth only labels detections (the simulated
classifier). Simulation only: on a robot, odometry comes from its own nodes.
"""

from __future__ import annotations

import numpy as np
import rclpy
from avatar_msgs.msg import Keyframe
from rclpy.node import Node
from sensor_msgs.msg import Image

from avatar.sim.measurements import KeyframeData
from avatar.tier2.dataset import TRACK_ID_STRIDE, load_meta
from avatar.tier2.frontend import AgentFrontEnd, DelayedRelease, FrontEndParams
from avatar.tier2.sdf import GZ_SENSORS
from avatar_sim.common import QOS, image_array, keyframe_msg, scenario_of, seconds


class FrontendLive(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("frontend_live", **kwargs)
        scenario, sim, seed = scenario_of(self)
        run_dir = self.get_parameter("run_dir").value
        tracking = self.get_parameter("tracking").value
        name = self.declare_parameter("agent", "").value
        delay = int(self.declare_parameter("release_delay_kf", 10).value)
        aid = next(a.agent_id for a in scenario.agents if a.name == name)
        self.ad = sim.agents[aid]
        sensors = load_meta(run_dir)["rigs"][name]
        fe = AgentFrontEnd(
            self.ad, {s: GZ_SENSORS[s] for s in sensors}, sim.world, sim.instance_descriptors,
            len(sim.world.parts) + TRACK_ID_STRIDE * (aid + 1), FrontEndParams(tracking=tracking),
            np.random.default_rng(seed + 40_000 + aid),
        )  # fmt: skip
        self.fe = fe
        self.release = DelayedRelease(fe, delay if tracking in ("ekf", "ekf_truth") else 0)
        self.frames: dict[int, dict[str, np.ndarray]] = {}
        self.sensors, self.next_k, self.outbox = sensors, 0, []
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
            if self.next_k == len(self.ad.keyframes):
                self.outbox += self.release.flush()
        self._send()

    def _send(self) -> None:
        if not self.outbox or self.pub.get_subscription_count() == 0:
            return  # hold the keyframes until the agent listens: each is relative to the last
        for k, dets in self.outbox:
            kf = self.ad.keyframes[k]
            self.pub.publish(keyframe_msg(KeyframeData(
                kf.t, kf.odom, kf.odom_sigmas, dets, kf.abs_z, kf.abs_z_sigma
            ), k))  # fmt: skip
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
