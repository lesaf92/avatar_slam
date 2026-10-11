"""T-S3-03: the live front-end on a recording's sensor frames (testdata/tier2, 61 keyframes)."""

import time
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from avatar_msgs.msg import Keyframe  # noqa: E402
from avatar_sim.common import QOS, keyframe_data  # noqa: E402
from avatar_sim.frontend_live import FrontendLive  # noqa: E402
from avatar_sim.sensor_replay import SensorReplay  # noqa: E402
from avatar_sim.sim_clock import SimClock  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from avatar.agent import AvatarParams  # noqa: E402
from avatar.runner import make_sim  # noqa: E402
from avatar.tier2.dataset import TRACK_ID_STRIDE, load_meta  # noqa: E402
from avatar.tier2.frontend import AgentFrontEnd, FrontEndParams, LateRelease  # noqa: E402
from avatar.tier2.sdf import GZ_SENSORS  # noqa: E402

FIXTURE = Path(__file__).resolve().parents[3] / "testdata" / "tier2"
AGENT = "uav_0"  # the sparse sensor: many detections leave late


def _expected() -> list:
    """The same front-end and release rule, run directly on the stored frames."""
    meta = load_meta(FIXTURE)
    _, sim = make_sim(meta["scenario"], meta["seed"], meta["duration_s"], AvatarParams(),
                      **meta["scenario_args"])  # fmt: skip
    aid = next(i for i, a in sim.agents.items() if a.config.name == AGENT)
    ad, sensors = sim.agents[aid], meta["rigs"][AGENT]
    raw = np.load(FIXTURE / "raw.npz")
    fe = AgentFrontEnd(ad, {s: GZ_SENSORS[s] for s in sensors}, sim.world,
                       sim.instance_descriptors, len(sim.world.parts) + TRACK_ID_STRIDE * (aid + 1),
                       FrontEndParams(tracking="ekf"),
                       np.random.default_rng(meta["seed"] + 40_000 + aid))  # fmt: skip
    rel, out = LateRelease(fe), []
    for k in range(len(ad.keyframes)):
        out += rel.push(k, fe.step(k, {s: raw[f"{AGENT}/{s}"][k] for s in sensors}))
    return out


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
    assert [m.index for m in got] == [k for k, _ in want]  # keyframes, then amendments
    for m, (_, dets) in zip(got, want, strict=True):
        back = keyframe_data(m)
        assert [(d.part_index, d.p_body.tolist()) for d in back.detections] == [
            (d.part_index, d.p_body.tolist()) for d in dets
        ]
    seen, amended = set(), 0
    for m in got:  # an amendment carries no odometry, a new keyframe does (but the first)
        amended += m.index in seen
        assert (m.index in seen) == (not m.has_odom and m.index > 0) or m.index == 0
        seen.add(m.index)
    assert amended > 0
