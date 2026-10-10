"""Live front-end (T-S3-03): one robot's Tier-2 front-end on its sensor topics, in ROS 2.

Subscribes to ``/avatar/<agent>/<sensor>/image`` for each of its rig's sensors (``sensor_replay``
or the Gazebo rigs), runs :class:`avatar.tier2.frontend.AgentFrontEnd` on each keyframe once all
its images are in, and publishes ``/avatar/<agent>/keyframe``. With the EKF tracker a detection
counts once its landmark has proved static: a keyframe leaves at once with the detections already
released, and the others follow when they are, as amendments (a ``Keyframe`` with an earlier
index and no odometry; :class:`~avatar.tier2.frontend.LateRelease`).

With ``sonar_live`` its DAVE sonars come from ``gazebo_sonar`` instead
(``/avatar/<agent>/<sensor>/sonar``, echo-level codes, and the latched ``sonar_geometry``) and are
detected as images, as offline from ``sonar.npz`` (T-S3-04).

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
from std_msgs.msg import Float64MultiArray

from avatar.sim.measurements import KeyframeData
from avatar.tier2.dataset import TRACK_ID_STRIDE
from avatar.tier2.frontend import AgentFrontEnd, FrontEndParams, LateRelease
from avatar.tier2.sdf import DAVE_SONARS, GZ_SENSORS, SONAR_DB_MAX, SONAR_DB_MIN, rig_sensors
from avatar.tier2.sonar_image import SonarFrames
from avatar_sim.common import QOS, image_array, keyframe_msg, scenario_of, seconds
from avatar_sim.gazebo_sonar import LATCHED


class FrontendLive(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("frontend_live", **kwargs)
        scenario, sim, seed = scenario_of(self)
        tracking = self.get_parameter("tracking").value
        name = self.declare_parameter("agent", "").value
        aid = next(a.agent_id for a in scenario.agents if a.name == name)
        self.ad = sim.agents[aid]
        sensors = rig_sensors(scenario)[name]
        sonar_live = bool(self.declare_parameter("sonar_live", False).value)
        self.sonars = {s for s in sensors if sonar_live and s in DAVE_SONARS}
        specs = {s: DAVE_SONARS[s].image_spec() if s in self.sonars else GZ_SENSORS[s]
                 for s in sensors}  # fmt: skip
        self.geometry: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        fe = AgentFrontEnd(
            self.ad, specs, sim.world, sim.instance_descriptors,
            len(sim.world.parts) + TRACK_ID_STRIDE * (aid + 1), FrontEndParams(tracking=tracking),
            np.random.default_rng(seed + 40_000 + aid),
        )  # fmt: skip
        self.fe = fe
        self.release = LateRelease(fe)
        self.frames: dict[int, dict[str, np.ndarray]] = {}
        self.sensors, self.next_k, self.outbox, self.sent = sensors, 0, [], 0
        self.pub = self.create_publisher(Keyframe, f"/avatar/{name}/keyframe", QOS)
        for s in sensors:
            topic = f"/avatar/{name}/{s}/" + ("sonar" if s in self.sonars else "image")
            self.create_subscription(Image, topic, lambda m, s=s: self._on_image(s, m), QOS)
        for s in self.sonars:
            self.create_subscription(Float64MultiArray, f"/avatar/{name}/{s}/sonar_geometry",
                                     lambda m, s=s: self._on_geometry(s, m), LATCHED)  # fmt: skip
        self.create_timer(0.2, self._send)  # keyframes held for a late agent still leave

    def _on_geometry(self, sensor: str, msg: Float64MultiArray) -> None:
        d = np.asarray(msg.data)
        rows, beams = int(d[0]), int(d[1])
        self.geometry[sensor] = (d[2 : 2 + rows], d[2 + rows : 2 + rows + beams])
        self._process()

    def _on_image(self, sensor: str, msg: Image) -> None:
        k = int(np.searchsorted(self.ad.times, seconds(msg.header.stamp) - 1e-6))
        if sensor in self.sonars:  # echo-level codes, made an image once its geometry is in
            frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
        else:
            frame = image_array(msg)
        self.frames.setdefault(k, {})[sensor] = frame
        self._process()

    def _process(self) -> None:
        if len(self.geometry) < len(self.sonars):
            return
        while len(self.frames.get(self.next_k, {})) == len(self.sensors):
            k = self.next_k
            scans = self.frames.pop(k)
            for s in self.sonars:
                rng, az = self.geometry[s]
                scans[s] = SonarFrames(scans[s][None], rng, az, SONAR_DB_MIN, SONAR_DB_MAX)[0]
            self.outbox += self.release.push(k, self.fe.step(k, scans))
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
