"""What the avatar_sim nodes share: the scenario they run, time stamps, keyframe messages."""

from __future__ import annotations

import numpy as np
import yaml
from avatar_msgs.msg import Detection as DetectionMsg
from avatar_msgs.msg import Keyframe
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.sim.measurements import KeyframeData, SimData
from avatar.sim.scenarios import Scenario
from avatar.sim.sensors import Detection
from avatar.types import LandmarkFlags, Medium

# Reliable and deep: a dropped message would be a loss or a gap no model drew.
QOS = QoSProfile(depth=10_000, reliability=ReliabilityPolicy.RELIABLE)


def seconds(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


def set_stamp(stamp, t: float) -> None:
    t = float(t)
    stamp.sec = int(t)
    stamp.nanosec = round((t - int(t)) * 1e9) % 10**9


def scenario_of(node: Node, detections: bool = False) -> tuple[Scenario, SimData, int]:
    """The scenario, its simulation data and the seed a node runs on, from its parameters.

    ``scenario``, ``scenario_args`` (a YAML mapping), ``seed`` and ``duration_s`` give a Tier-1
    simulation (``make_sim``). ``run_dir`` instead names a Tier-2 recording, whose ``meta.json``
    gives them; with ``detections`` its keyframes carry the offline front-end's detections
    (``tracking``: ``oracle``, ``ekf`` ...; ``sonar``: a sonar recording such as ``sonar360``).
    """
    p = node.declare_parameter
    run_dir = p("run_dir", "").value
    tracking, sonar = p("tracking", "oracle").value, p("sonar", "").value
    name, args = p("scenario", "harbor_fleet").value, p("scenario_args", "{}").value
    seed, duration = int(p("seed", 0).value), float(p("duration_s", 600.0).value)
    params = AvatarParams()
    if not run_dir:
        scenario, sim = make_sim(name, seed, duration, params, **(yaml.safe_load(args) or {}))
        return scenario, sim, seed
    from avatar.tier2.dataset import build_tier2_sim, load_meta
    from avatar.tier2.frontend import FrontEndParams

    meta = load_meta(run_dir)
    if detections:
        fe = FrontEndParams(tracking=tracking)
        scenario, sim, _ = build_tier2_sim(run_dir, params, fe, sonar=sonar or None)
    else:  # the world, the ground truth and the odometry are those of make_sim
        scenario, sim = make_sim(
            meta["scenario"], int(meta["seed"]), float(meta["duration_s"]), params,
            **meta["scenario_args"],
        )  # fmt: skip
    return scenario, sim, int(meta["seed"])


def keyframe_msg(kf: KeyframeData, index: int) -> Keyframe:
    """``KeyframeData`` -> ``avatar_msgs/Keyframe`` (the ground-truth labels stay behind)."""
    m = Keyframe()
    set_stamp(m.header.stamp, kf.t)
    m.index = index
    m.has_odom = kf.odom is not None
    if m.has_odom:
        m.odom = [float(v) for v in kf.odom]
        m.odom_sigma = [float(v) for v in kf.odom_sigmas]
    m.has_abs_z = kf.abs_z is not None
    if m.has_abs_z:
        m.abs_z, m.abs_z_sigma = float(kf.abs_z), float(kf.abs_z_sigma or 0.0)
    for d in kf.detections:
        dm = DetectionMsg()
        dm.track_id = int(d.part_index)
        dm.object_key = -1 if d.object_key is None else int(d.object_key)
        dm.medium, dm.modality, dm.class_id = int(d.medium), int(d.modality), int(d.class_id)
        dm.position.x, dm.position.y, dm.position.z = (float(v) for v in d.p_body)
        dm.sigma.x, dm.sigma.y, dm.sigma.z = (float(v) for v in d.sigmas)
        dm.extent.x, dm.extent.y, dm.extent.z = (float(v) for v in d.extent)
        dm.descriptor = [float(v) for v in d.descriptor]
        m.detections.append(dm)
    return m


def keyframe_data(m: Keyframe) -> KeyframeData:
    """``avatar_msgs/Keyframe`` -> ``KeyframeData``."""
    dets = [
        Detection(
            part_index=d.track_id,
            medium=Medium(d.medium),
            modality=LandmarkFlags(d.modality),
            p_body=np.array([d.position.x, d.position.y, d.position.z]),
            sigmas=np.array([d.sigma.x, d.sigma.y, d.sigma.z]),
            extent=np.array([d.extent.x, d.extent.y, d.extent.z]),
            class_id=d.class_id,
            descriptor=np.array(d.descriptor, dtype=np.float64),
            object_key=None if d.object_key < 0 else d.object_key,
        )
        for d in m.detections
    ]
    return KeyframeData(
        t=seconds(m.header.stamp),
        odom=np.array(m.odom, dtype=np.float64) if m.has_odom else None,
        odom_sigmas=np.array(m.odom_sigma, dtype=np.float64) if m.has_odom else None,
        detections=dets,
        abs_z=m.abs_z if m.has_abs_z else None,
        abs_z_sigma=m.abs_z_sigma if m.has_abs_z else None,
    )


def image_msg(arr, t: float) -> Image:
    """A range scan or depth image [m] as a ``32FC1`` ``sensor_msgs/Image`` (rows, columns as
    stored: a LiDAR scan is (vertical, horizontal) rays, ``avatar.tier2.rays``)."""
    a = np.ascontiguousarray(arr, dtype=np.float32)
    m = Image()
    set_stamp(m.header.stamp, t)
    m.height, m.width = a.shape
    m.encoding, m.is_bigendian, m.step = "32FC1", 0, 4 * a.shape[1]
    m.data = a.tobytes()
    return m


def image_array(m: Image) -> np.ndarray:
    """``32FC1`` ``sensor_msgs/Image`` -> (rows, columns) float32."""
    return np.frombuffer(bytes(m.data), dtype=np.float32).reshape(m.height, m.width)
