#!/usr/bin/python3
"""Sonar-pass driver: steps the acoustic world keyframe by keyframe and stores DAVE's sonar images.

Runs with the **system** Python of the avatar-dave image (rclpy, gz.transport13; its NumPy is
1.x), so it does I/O and arithmetic that does not depend on the NumPy version, nothing else.
Started by ``record_sonar.py``, which writes the plan (ADR-0008, docs/LOG.md L34)::

    /usr/bin/python3 sonar_driver.py plan.json out.npz

Per keyframe: ``set_pose`` of every rig, two 1 ms iterations (the sonars run at 500 Hz, so
each renders exactly once, at the second iteration, after the pose update), then wait for
one ``ProjectedSonarImage`` per sonar. An image is stored as uint8: rows averaged in dB in
groups of ``pool`` (arithmetic only), mapped from [db_min, db_max] to 0..255.
"""

from __future__ import annotations

import json
import math
import sys
import time

import numpy as np
import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzNode
from marine_acoustic_msgs.msg import ProjectedSonarImage
from rclpy.node import Node as RosNode


class Listener(RosNode):
    def __init__(self, sensors: list[dict]) -> None:
        super().__init__("sonar_pass")
        self.count = {s["key"]: 0 for s in sensors}
        self.last: dict[str, ProjectedSonarImage] = {}
        for s in sensors:
            self.create_subscription(
                ProjectedSonarImage,
                s["topic"] + "/sonar_image_raw",
                lambda m, k=s["key"]: self._on_image(k, m),
                10,
            )

    def _on_image(self, key: str, msg: ProjectedSonarImage) -> None:
        self.count[key] += 1
        self.last[key] = msg


def step(gz: GzNode, world: str, n: int) -> bool:
    req = WorldControl()
    req.pause = True
    req.multi_step = n
    ok, rep = gz.request(f"/world/{world}/control", req, WorldControl, Boolean, 5000)
    return bool(ok and rep.data)


def set_pose(gz: GzNode, world: str, name: str, p: list[float]) -> bool:
    req = Pose()
    req.name = name
    req.position.x, req.position.y, req.position.z = p[0], p[1], p[2]
    req.orientation.z, req.orientation.w = math.sin(0.5 * p[3]), math.cos(0.5 * p[3])
    ok, rep = gz.request(f"/world/{world}/set_pose", req, Pose, Boolean, 5000)
    return bool(ok and rep.data)


def quantise(msg: ProjectedSonarImage, s: dict, plan: dict) -> np.ndarray:
    """(bins, beams) uint8 image of one message; checks the size against the plan."""
    nr, nb = len(msg.ranges), msg.image.beam_count
    if (nr, nb) != (s["raw_bins"], s["beams"]):
        raise SystemExit(
            f"{s['key']}: image is {nr} x {nb}, expected {s['raw_bins']} x {s['beams']}"
        )
    img = np.frombuffer(bytes(msg.image.data), dtype=np.float32).reshape(nr, nb)
    pool = plan["pool"]
    n = (nr // pool) * pool
    db = img[:n].reshape(n // pool, pool, nb).mean(axis=1)
    scale = np.float32(255.0 / (plan["db_max"] - plan["db_min"]))
    q = np.rint((db - np.float32(plan["db_min"])) * scale)
    return np.clip(q, 0, 255).astype(np.uint8)


def main(plan_path: str, out_path: str) -> None:
    plan = json.loads(open(plan_path).read())
    world, sensors, rigs = plan["world"], plan["sensors"], plan["rigs"]
    poses = plan["poses"]  # [keyframe][rig] -> [x, y, z, yaw]
    rclpy.init()
    ros = Listener(sensors)
    gz = GzNode()

    def spin(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(ros, timeout_sec=0.01)

    def wait_frames(target: dict[str, int], timeout_s: float) -> None:
        end = time.time() + timeout_s
        while any(ros.count[key] < n for key, n in target.items()):
            if time.time() > end:
                raise SystemExit(f"sonar timeout: frames {ros.count}, expected {target}")
            rclpy.spin_once(ros, timeout_sec=0.05)

    # The world is up when the control service answers. The sonars render at every second
    # iteration from then on, but ROS discovery makes the first frames go unheard, so the world
    # is stepped to a fixed simulation time before keyframe 0: the speckle seed depends on
    # the simulation time, and the recording is then reproducible.
    t0 = time.time()
    calls = 0
    while calls < plan["warmup_calls"]:
        if step(gz, world, 2):
            calls += 1
            spin(0.05)
        else:
            time.sleep(0.5)
            if time.time() - t0 > 180:
                raise SystemExit("gazebo world did not answer within 180 s")
    spin(2.0)
    base = dict(ros.count)
    if any(c < 1 for c in base.values()):
        raise SystemExit(f"sonars silent after {calls} warm-up steps: {base}; raise warmup_calls")
    print(f"[sonar_driver] warm-up: {calls} steps of 2 ms, frames heard {base}", flush=True)

    n_kf = len(poses)
    out = {
        s["key"]: np.zeros((n_kf, s["raw_bins"] // plan["pool"], s["beams"]), np.uint8)
        for s in sensors
    }
    geometry: dict[str, np.ndarray] = {}
    t_rec = time.time()
    for k in range(n_kf):
        for r, name in enumerate(rigs):
            if not set_pose(gz, world, name, poses[k][r]):
                raise SystemExit(f"set_pose failed for {name} at keyframe {k}")
        if not step(gz, world, 2):
            raise SystemExit(f"step failed at keyframe {k}")
        target = {key: base[key] + k + 1 for key in base}
        wait_frames(target, float(plan["timeout_s"]))
        spin(0.02)  # a second frame for this keyframe would show up here
        for s in sensors:
            if ros.count[s["key"]] != target[s["key"]]:
                key = s["key"]
                raise SystemExit(
                    f"{key}: {ros.count[key]} frames after keyframe {k}, expected {target[key]}"
                )
            msg = ros.last[s["key"]]
            out[s["key"]][k] = quantise(msg, s, plan)
            if s["key"] not in geometry:
                pool = plan["pool"]
                n = (len(msg.ranges) // pool) * pool
                rng = np.asarray(msg.ranges, dtype=np.float64)[:n].reshape(-1, pool).mean(axis=1)
                geometry[s["key"]] = rng
                geometry[s["key"] + "#azimuth"] = np.array(
                    [math.atan2(v.y, v.x) for v in msg.beam_directions], dtype=np.float64
                )
        if k % 20 == 0:
            print(f"[sonar_driver] keyframe {k}/{n_kf}  {time.time() - t_rec:.0f} s", flush=True)
    arrays = dict(out)
    for key, arr in geometry.items():
        arrays[key.replace("#azimuth", "/azimuth_rad") if "#" in key else key + "/range_m"] = arr
    arrays["warmup_calls"] = np.array(calls)
    np.savez_compressed(out_path, **arrays)
    print(f"[sonar_driver] done in {time.time() - t_rec:.0f} s", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
