"""Gazebo sonar (T-S3-04): the BlueROV2s' sonars rendered live by DAVE, as ROS 2 images.

Writes the acoustic world and the sonar pass's plan to ``work_dir`` (under the repository, which
the container mounts), starts Gazebo and ``sonar_driver.py --stream`` in the ``avatar-dave``
image (``experiments/gazebo/sonar_stream.sh``), and for every keyframe that ``/clock`` reaches
sets the models at the rigs' ground-truth poses and publishes each sonar's image on
``/avatar/<agent>/<sensor>/sonar`` (``mono8``: the recorder's uint8 echo-level codes, rows =
range bins, columns = beams; a Ping360's fans stitched into one turn). Its geometry (range bins
and beam azimuths) is published once on ``/avatar/<agent>/<sensor>/sonar_geometry``
(``std_msgs/Float64MultiArray``: rows, beams, ranges, azimuths), latched.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import avatar
from avatar.tier2.sdf import DAVE_SONARS, sonar_pass_plan, sonar_rigs
from avatar.tier2.sonar_image import stitch_fans
from avatar_sim.common import QOS, scenario_of, seconds, set_stamp

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class GazeboSonar(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gazebo_sonar", **kwargs)
        scenario, sim, seed = scenario_of(self)
        repo = Path(avatar.__file__).resolve().parents[2]
        work = Path(self.declare_parameter("work_dir", f"results/ros2_sonar/seed{seed}").value)
        work = (work if work.is_absolute() else repo / work).resolve()
        image = self.declare_parameter("image", "avatar-dave").value
        timeout = float(self.declare_parameter("timeout_s", 30.0).value)
        names = {a.agent_id: a.name for a in scenario.agents}
        gt = {names[i]: ad.gt for i, ad in sim.agents.items()}
        self.times = next(iter(sim.agents.values())).times
        rigs = sonar_rigs(scenario)
        multi = any(len(DAVE_SONARS[s].fan_yaws_rad) > 1 for ss in rigs.values() for s in ss)
        warmup = int(self.declare_parameter("warmup_calls", 200 if multi else 60).value)
        world, plan = sonar_pass_plan(scenario, gt, 0, warmup, timeout)
        work.mkdir(parents=True, exist_ok=True)
        (work / "world.sdf").write_text(world)
        (work / "plan.json").write_text(json.dumps(plan))
        self.models = [(a, yaw) for _, a, yaw in plan["models"]]
        self.gt = gt
        cmd = [
            "docker", "run", "-i", "--rm", "--gpus", "all",
            "--user", f"{os.getuid()}:{os.getgid()}", "-v", f"{repo}:/work", "-w", "/work", image,
            "sh", "experiments/gazebo/sonar_stream.sh", str(work.relative_to(repo)),
        ]  # fmt: skip
        self.log = open(work / "stream.log", "w")  # open for the node's life
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=self.log)  # fmt: skip
        if self._read(4) != b"RDY\n":
            raise RuntimeError(f"the DAVE sonar driver did not start; see {work / 'stream.log'}")
        self.fans = []  # per plan sensor: (rows, beams, ranges, azimuths)
        for _ in plan["sensors"]:
            rows, beams = np.frombuffer(self._read(8), dtype="<u4")
            rng = np.frombuffer(self._read(8 * int(rows)), dtype="<f8")
            az = np.frombuffer(self._read(8 * int(beams)), dtype="<f8")
            self.fans.append((int(rows), int(beams), rng, az))
        # outputs: (agent, sensor) -> indices of its fans in the plan, and their yaws
        self.outputs = {}
        i = 0
        for n, ss in rigs.items():
            for s in ss:
                fans = DAVE_SONARS[s].fans(s)
                self.outputs[(n, s)] = (list(range(i, i + len(fans))), [y for _, y in fans])
                i += len(fans)
        self.pubs = {}
        for (n, s), (idx, yaws) in self.outputs.items():
            self.pubs[(n, s)] = self.create_publisher(Image, f"/avatar/{n}/{s}/sonar", QOS)
            geo = self.create_publisher(
                Float64MultiArray, f"/avatar/{n}/{s}/sonar_geometry", LATCHED
            )
            rng = self.fans[idx[0]][2]
            _, az = self._stitch([np.zeros((1, self.fans[j][1]), np.uint8) for j in idx], idx, yaws)
            geo.publish(Float64MultiArray(data=[float(len(rng)), float(len(az)), *rng, *az]))
        self.next = 0
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _read(self, n: int) -> bytes:
        data = self.proc.stdout.read(n)
        if len(data) != n:
            raise RuntimeError("the DAVE sonar driver stopped")
        return data

    def _stitch(self, images, idx, yaws):
        if len(idx) == 1:
            return images[0], self.fans[idx[0]][3]
        return stitch_fans(images, [self.fans[j][3] for j in idx], yaws)

    def _on_clock(self, msg: Clock) -> None:
        if any(p.get_subscription_count() == 0 for p in self.pubs.values()):
            return  # hold until the front-ends listen
        t = seconds(msg.clock)
        while self.next < len(self.times) and self.times[self.next] <= t + 1e-9:
            k = self.next
            vals = []
            for a, yaw in self.models:
                p = self.gt[a][k]
                vals += [float(p[0]), float(p[1]), float(p[2]), float(p[3] + yaw)]
            self.proc.stdin.write((" ".join(f"{v:.6f}" for v in vals) + "\n").encode())
            self.proc.stdin.flush()
            frames = [
                np.frombuffer(self._read(rows * beams), dtype=np.uint8).reshape(rows, beams)
                for rows, beams, _, _ in self.fans
            ]
            for (n, s), (idx, yaws) in self.outputs.items():
                img, _ = self._stitch([frames[j] for j in idx], idx, yaws)
                out = Image()
                set_stamp(out.header.stamp, self.times[k])
                out.height, out.width = img.shape
                out.encoding, out.is_bigendian, out.step = "mono8", 0, img.shape[1]
                out.data = np.ascontiguousarray(img).tobytes()
                self.pubs[(n, s)].publish(out)
            self.next += 1

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.stdin.close()  # end of input: the driver exits, the container stops
            try:
                self.proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.log.close()


def main() -> None:
    rclpy.init()
    node = GazeboSonar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
