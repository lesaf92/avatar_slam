#!/usr/bin/python3
"""Sonar-pass driver: steps the acoustic world keyframe by keyframe and stores DAVE's sonar images.

Runs with the **system** Python of the avatar-dave image (rclpy, gz.transport13; its NumPy is
1.x), so it does I/O and arithmetic that does not depend on the NumPy version, nothing else.
Started by ``record_sonar.py``, which writes the plan (ADR-0008, docs/LOG.md L34)::

    /usr/bin/python3 sonar_driver.py plan.json out.npz
    /usr/bin/python3 sonar_driver.py --stream plan.json      # poses on stdin, images on stdout

Per keyframe: ``set_pose`` of every rig, then two single 1 ms iterations, each followed by
its frame (the sonars run at 1000 Hz); the second frame, strictly after the pose update, is
kept (docs/LOG.md L35). An image is stored as uint8: rows averaged in dB in
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


def setup(plan: dict):
    """ROS listener, gz node, and the world warmed up; returns (ros, gz, frame counts, spin)."""
    rclpy.init()
    ros = Listener(plan["sensors"])
    gz = GzNode()

    def spin(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(ros, timeout_sec=0.01)

    # The world is up when the control service answers. The sonars render at every second
    # iteration from then on, but ROS discovery makes the first frames go unheard, so the world
    # is stepped to a fixed simulation time before keyframe 0: the speckle seed depends on
    # the simulation time, and the recording is then reproducible.
    world = plan["world"]
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
    log(f"[sonar_driver] warm-up: {calls} steps of 2 ms, frames heard {base}")
    return ros, gz, base, spin, calls


def step_keyframe(
    ros, gz, plan: dict, poses: list, base: dict, k: int, spin, relative: bool = False
) -> None:
    """Set the models' poses for keyframe ``k`` and step until each sonar's frame of the last
    iteration is in (``ros.last``). ``relative`` (stream mode): the world idles between
    keyframes and DAVE may publish then, so frames are counted from just before this keyframe's
    steps, after draining those, rather than from the warm-up."""
    world, fpk = plan["world"], int(plan.get("frames_per_keyframe", 1))
    if relative:
        spin(0.05)
        base = {key: ros.count[key] - fpk * k for key in base}
    for r, name in enumerate(plan["rigs"]):
        if not set_pose(gz, world, name, poses[r]):
            raise SystemExit(f"set_pose failed for {name} at keyframe {k}")
    # One iteration at a time, each frame awaited before the next step: run back to back,
    # a slow CUDA frame makes the sensors system skip the next one. The last frame (of the
    # second iteration) is strictly after the pose update.
    target = {}
    for it in range(fpk):
        if not step(gz, world, 1):
            raise SystemExit(f"step failed at keyframe {k}")
        target = {key: base[key] + fpk * k + it + 1 for key in base}
        end = time.time() + float(plan["timeout_s"])
        while any(ros.count[key] < n for key, n in target.items()):
            if time.time() > end:
                raise SystemExit(f"sonar timeout: frames {ros.count}, expected {target}")
            rclpy.spin_once(ros, timeout_sec=0.05)
    spin(0.02)  # a second frame for this keyframe would show up here
    for key, n in target.items():
        if ros.count[key] != n and not relative:
            raise SystemExit(f"{key}: {ros.count[key]} frames after keyframe {k}, expected {n}")


def geometry(msg: ProjectedSonarImage, pool: int) -> tuple[np.ndarray, np.ndarray]:
    """(range bin centres [m], beam azimuths [rad]) of a stored image."""
    n = (len(msg.ranges) // pool) * pool
    rng = np.asarray(msg.ranges, dtype=np.float64)[:n].reshape(-1, pool).mean(axis=1)
    az = np.array([math.atan2(v.y, v.x) for v in msg.beam_directions], dtype=np.float64)
    return rng, az


def log(text: str) -> None:
    print(text, file=sys.stderr, flush=True)


def main(plan_path: str, out_path: str) -> None:
    plan = json.loads(open(plan_path).read())
    sensors = plan["sensors"]
    poses = plan["poses"]  # [keyframe][rig] -> [x, y, z, yaw]
    ros, gz, base, spin, calls = setup(plan)
    n_kf = len(poses)
    out = {
        s["key"]: np.zeros((n_kf, s["raw_bins"] // plan["pool"], s["beams"]), np.uint8)
        for s in sensors
    }
    geom: dict[str, tuple] = {}
    t_rec = time.time()
    for k in range(n_kf):
        step_keyframe(ros, gz, plan, poses[k], base, k, spin)
        for s in sensors:
            msg = ros.last[s["key"]]
            out[s["key"]][k] = quantise(msg, s, plan)
            if s["key"] not in geom:
                geom[s["key"]] = geometry(msg, plan["pool"])
        if k % 20 == 0:
            print(f"[sonar_driver] keyframe {k}/{n_kf}  {time.time() - t_rec:.0f} s", flush=True)
    arrays = dict(out)
    for key, (rng, az) in geom.items():
        arrays[key + "/range_m"] = rng
        arrays[key + "/azimuth_rad"] = az
    arrays["warmup_calls"] = np.array(calls)
    np.savez_compressed(out_path, **arrays)
    print(f"[sonar_driver] done in {time.time() - t_rec:.0f} s", flush=True)
    rclpy.shutdown()


def stream(plan_path: str) -> None:
    """Stream mode (T-S3-04): after warm-up write b"RDY\\n", then per sensor (plan order) its
    geometry: uint32 rows, uint32 beams, float64 ranges [rows], float64 azimuths [beams]. Then for
    every line of poses (all models, "x y z yaw" each, space-separated) on stdin, step one keyframe
    and write each sensor's uint8 image (rows x beams). Ends at end of input."""
    plan = json.loads(open(plan_path).read())
    sensors, out = plan["sensors"], sys.stdout.buffer
    ros, gz, base, spin, _ = setup(plan)
    out.write(b"RDY\n")
    for s in sensors:
        rng, az = geometry(ros.last[s["key"]], plan["pool"])
        out.write(np.array([len(rng), len(az)], dtype="<u4").tobytes())
        out.write(rng.astype("<f8").tobytes() + az.astype("<f8").tobytes())
    out.flush()
    n_models = len(plan["rigs"])
    for k, line in enumerate(sys.stdin):
        v = [float(x) for x in line.split()]
        if len(v) != 4 * n_models:
            raise SystemExit(f"keyframe {k}: {len(v)} pose values, expected {4 * n_models}")
        poses = [v[4 * i : 4 * i + 4] for i in range(n_models)]
        step_keyframe(ros, gz, plan, poses, base, k, spin, relative=True)
        for s in sensors:
            out.write(quantise(ros.last[s["key"]], s, plan).tobytes())
        out.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    if sys.argv[1] == "--stream":
        stream(sys.argv[2])
    else:
        main(sys.argv[1], sys.argv[2])
