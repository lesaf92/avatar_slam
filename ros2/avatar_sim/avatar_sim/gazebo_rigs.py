"""Gazebo rigs (T-S3-03): the Tier-2 sensor rigs rendered live, as ROS 2 images.

Writes the run's world and the recorder's plan to ``work_dir`` (under the repository, which the
container mounts), starts Gazebo and ``gz_recorder --stream`` in the ``avatar-tier2`` image
(``experiments/gazebo/stream.sh``), and for every keyframe that ``/clock`` reaches sets the rigs
at their ground-truth poses (ADR-0007), steps, and publishes each sensor's frame on
``/avatar/<agent>/<sensor>/image`` as ``sensor_replay`` does (``agents``: those agents' rigs,
comma-separated; default all). It subscribes to ``/clock`` only
once Gazebo is up, so the clock does not start without it.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

import avatar
from avatar.tier2.sdf import GZ_SENSORS, rig_sensors, sensor_topic, world_sdf
from avatar_sim.common import QOS, image_msg, scenario_of, seconds

WORLD = "avatar_tier2"


class GazeboRigs(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gazebo_rigs", **kwargs)
        scenario, sim, seed = scenario_of(self)
        repo = Path(avatar.__file__).resolve().parents[2]
        work = Path(self.declare_parameter("work_dir", f"results/ros2_gazebo/seed{seed}").value)
        work = (work if work.is_absolute() else repo / work).resolve()
        image = self.declare_parameter("image", "avatar-tier2").value
        timeout = float(self.declare_parameter("timeout_s", 10.0).value)
        names = {a.name: a.agent_id for a in scenario.agents}
        every = {n: s for n, s in rig_sensors(scenario).items() if s}
        only = {s for s in self.declare_parameter("agents", "").value.split(",") if s}
        self.rigs = {n: s for n, s in every.items() if not only or n in only}
        self.gt = {n: sim.agents[names[n]].gt for n in self.rigs}
        initial = {n: sim.agents[names[n]].gt[0] for n in every}  # the world holds every rig
        self.times = next(iter(sim.agents.values())).times
        self.keys = [(n, s) for n, ss in self.rigs.items() for s in ss]
        work.mkdir(parents=True, exist_ok=True)
        (work / "world.sdf").write_text(world_sdf(scenario, initial, WORLD))
        plan = [f"{WORLD} 0 {len(self.rigs)} {len(self.keys)}"]
        for n, s in self.keys:
            spec = GZ_SENSORS[s]
            kind = "depth" if spec.kind == "depth" else "scan"
            plan.append(f"{sensor_topic(n, s)} {kind} {spec.h_samples * spec.v_samples}")
        (work / "plan.txt").write_text("\n".join([*plan, *self.rigs]) + "\n")
        rel = work.relative_to(repo)
        cmd = [
            "docker", "run", "-i", "--rm", "--gpus", "all",
            "--user", f"{os.getuid()}:{os.getgid()}", "-v", f"{repo}:/work", "-w", "/work", image,
            "sh", "experiments/gazebo/stream.sh", str(rel), str(timeout),
        ]  # fmt: skip
        self.log = open(work / "stream.log", "w")  # open for the node's life
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=self.log)  # fmt: skip
        if self.proc.stdout.read(4) != b"RDY\n":
            raise RuntimeError(f"the Gazebo recorder did not start; see {work / 'stream.log'}")
        self.pubs = {
            (n, s): self.create_publisher(Image, f"/avatar/{n}/{s}/image", QOS)
            for n, s in self.keys
        }
        self.next = 0
        self.create_subscription(Clock, "/clock", self._on_clock, QOS)

    def _on_clock(self, msg: Clock) -> None:
        if any(p.get_subscription_count() == 0 for p in self.pubs.values()):
            return  # hold until the front-ends listen
        t = seconds(msg.clock)
        while self.next < len(self.times) and self.times[self.next] <= t + 1e-9:
            k = self.next
            poses = "".join(
                " ".join(f"{float(v):.6f}" for v in g[k]) + "\n" for g in self.gt.values()
            )
            self.proc.stdin.write(poses.encode())
            self.proc.stdin.flush()
            for n, s in self.keys:
                spec = GZ_SENSORS[s]
                nbytes = 4 * spec.h_samples * spec.v_samples
                raw = self.proc.stdout.read(nbytes)
                if len(raw) != nbytes:
                    raise RuntimeError(f"the Gazebo recorder stopped at keyframe {k}")
                frame = np.frombuffer(raw, dtype=np.float32).reshape(spec.v_samples, spec.h_samples)
                self.pubs[(n, s)].publish(image_msg(frame, self.times[k]))
            self.next += 1

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.stdin.close()  # end of input: the recorder exits, the container stops
            try:
                self.proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.log.close()


def main() -> None:
    rclpy.init()
    node = GazeboRigs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
