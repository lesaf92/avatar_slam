"""T-S3-03: the live front-end on a recording's sensor frames (testdata/tier2, 61 keyframes);
T-S3-04: on DAVE sonar codes, as ``gazebo_sonar`` publishes them (a local recording)."""

import time
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from avatar_msgs.msg import Keyframe  # noqa: E402
from avatar_sim.common import QOS, keyframe_data, set_stamp  # noqa: E402
from avatar_sim.frontend_live import FrontendLive  # noqa: E402
from avatar_sim.gazebo_sonar import LATCHED  # noqa: E402
from avatar_sim.sensor_replay import SensorReplay  # noqa: E402
from avatar_sim.sim_clock import SimClock  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from avatar.agent import AvatarParams  # noqa: E402
from avatar.runner import make_sim  # noqa: E402
from avatar.tier2.dataset import TRACK_ID_STRIDE, load_meta  # noqa: E402
from avatar.tier2.frontend import AgentFrontEnd, FrontEndParams, LateRelease  # noqa: E402
from avatar.tier2.sdf import DAVE_SONARS, GZ_SENSORS, SONAR_DB_MAX, SONAR_DB_MIN  # noqa: E402
from avatar.tier2.sonar_image import SonarFrames  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
FIXTURE = REPO / "testdata" / "tier2"
AGENT = "uav_0"  # the sparse sensor: many detections leave late
SONAR_RUN = REPO / "results" / "tier2" / "harbor_fleet_seed0"  # DAVE: not in the repository


def _expected(run=FIXTURE, agent=AGENT, scans=None, specs=None, n=None) -> list:
    """The same front-end and release rule, run directly on the stored frames (``scans(k)``)."""
    meta = load_meta(run)
    _, sim = make_sim(meta["scenario"], meta["seed"], meta["duration_s"], AvatarParams(),
                      **meta["scenario_args"])  # fmt: skip
    aid = next(i for i, a in sim.agents.items() if a.config.name == agent)
    ad, sensors = sim.agents[aid], meta["rigs"][agent]
    if scans is None:
        raw = np.load(run / "raw.npz")
        scans = lambda k: {s: raw[f"{agent}/{s}"][k] for s in sensors}  # noqa: E731
    fe = AgentFrontEnd(ad, specs or {s: GZ_SENSORS[s] for s in sensors}, sim.world,
                       sim.instance_descriptors, len(sim.world.parts) + TRACK_ID_STRIDE * (aid + 1),
                       FrontEndParams(tracking="ekf"),
                       np.random.default_rng(meta["seed"] + 40_000 + aid))  # fmt: skip
    rel, out = LateRelease(fe), []
    for k in range(n or len(ad.keyframes)):
        out += rel.push(k, fe.step(k, scans(k)))
    return out


def _same(got: list, want: list) -> None:
    assert [m.index for m in got] == [k for k, _ in want]  # keyframes, then amendments
    for m, (_, dets) in zip(got, want, strict=True):
        back = keyframe_data(m)
        assert [(d.part_index, d.p_body.tolist()) for d in back.detections] == [
            (d.part_index, d.p_body.tolist()) for d in dets
        ]


def test_live_front_end_gives_the_direct_pass():
    params = [
        Parameter("run_dir", value=str(FIXTURE)),
        Parameter("tracking", value="ekf"),
        Parameter("agent", value=AGENT),
    ]
    rclpy.init()
    try:
        ex = SingleThreadedExecutor()
        live = FrontendLive(parameter_overrides=params)
        sensors = SensorReplay(parameter_overrides=params)
        got = []
        sink = rclpy.create_node("sink")
        sink.create_subscription(Keyframe, f"/avatar/{AGENT}/keyframe", got.append, QOS)
        clock = SimClock(parameter_overrides=[
            Parameter("end_s", value=60.0), Parameter("rate", value=50.0),
            Parameter("subscribers", value=1),
        ])  # fmt: skip
        for n in (live, sensors, sink, clock):
            ex.add_node(n)
        want = _expected()
        deadline = time.monotonic() + 300
        while len(got) < len(want):
            assert time.monotonic() < deadline, f"timed out with {len(got)} keyframes"
            ex.spin_once(timeout_sec=0.05)
    finally:
        rclpy.try_shutdown()
    _same(got, want)
    seen, amended = set(), 0
    for m in got:  # an amendment carries no odometry, a new keyframe does (but the first)
        amended += m.index in seen
        assert (m.index in seen) == (not m.has_odom and m.index > 0) or m.index == 0
        seen.add(m.index)
    assert amended > 0


@pytest.mark.skipif(not (SONAR_RUN / "sonar.npz").exists(), reason="needs a local DAVE recording")
def test_live_sonar_front_end_gives_the_direct_pass():
    """``sonar_live``: echo-level codes and a latched geometry, as gazebo_sonar publishes them;
    the images wait for the geometry."""
    agent, s, n = "uuv_0", "gemini_720s", 30
    z = np.load(SONAR_RUN / "sonar.npz")
    rng, az = z[f"{agent}/{s}/range_m"], z[f"{agent}/{s}/azimuth_rad"]
    codes = z[f"{agent}/{s}"][:n]
    frames = SonarFrames(codes, rng, az, SONAR_DB_MIN, SONAR_DB_MAX)
    want = _expected(SONAR_RUN, agent, lambda k: {s: frames[k]},
                     {s: DAVE_SONARS[s].image_spec()}, n)  # fmt: skip
    assert any(dets for _, dets in want)
    rclpy.init()
    try:
        ex = SingleThreadedExecutor()
        live = FrontendLive(parameter_overrides=[
            Parameter("run_dir", value=str(SONAR_RUN)), Parameter("tracking", value="ekf"),
            Parameter("agent", value=agent), Parameter("sonar_live", value=True),
        ])  # fmt: skip
        src = rclpy.create_node("src")
        img = src.create_publisher(Image, f"/avatar/{agent}/{s}/sonar", QOS)
        geo = src.create_publisher(
            Float64MultiArray, f"/avatar/{agent}/{s}/sonar_geometry", LATCHED
        )
        got = []
        sink = rclpy.create_node("sink")
        sink.create_subscription(Keyframe, f"/avatar/{agent}/keyframe", got.append, QOS)
        for x in (live, src, sink):
            ex.add_node(x)
        deadline = time.monotonic() + 300
        while img.get_subscription_count() == 0 or live.pub.get_subscription_count() == 0:
            assert time.monotonic() < deadline, "no subscriber"
            ex.spin_once(timeout_sec=0.05)
        for k in range(n):
            m = Image(height=codes.shape[1], width=codes.shape[2], encoding="mono8",
                      step=codes.shape[2], data=codes[k].tobytes())  # fmt: skip
            set_stamp(m.header.stamp, live.ad.times[k])
            img.publish(m)
        for _ in range(20):
            ex.spin_once(timeout_sec=0.05)
        assert not got  # no geometry yet
        geo.publish(Float64MultiArray(data=[float(len(rng)), float(len(az)), *rng, *az]))
        while len(got) < len(want):
            assert time.monotonic() < deadline, f"timed out with {len(got)} keyframes"
            ex.spin_once(timeout_sec=0.05)
    finally:
        rclpy.try_shutdown()
    _same(got, want)
