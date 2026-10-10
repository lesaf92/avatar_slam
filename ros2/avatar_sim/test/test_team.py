"""T-S3-02: keyframe messages, and the team in ROS 2 with free-running time (ADR-0010)."""

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from avatar_sim.agent_node import AgentNode  # noqa: E402
from avatar_sim.comm_emulator import CommEmulator  # noqa: E402
from avatar_sim.common import keyframe_data, keyframe_msg  # noqa: E402
from avatar_sim.frontend_replay import FrontendReplay  # noqa: E402
from avatar_sim.gateway_node import GatewayNode  # noqa: E402
from avatar_sim.sim_clock import SimClock  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from avatar.agent import AvatarParams  # noqa: E402
from avatar.eval.metrics import frame_error  # noqa: E402
from avatar.geometry import compose, inverse  # noqa: E402
from avatar.runner import make_sim  # noqa: E402
from avatar.sim.sensors import Detection  # noqa: E402
from avatar.types import LandmarkFlags, Medium  # noqa: E402

# 1 kbit/s acoustic links: the team merges within a short run
SCENARIO_ARGS, SEED, DURATION = "{acoustic: acoustic_generic}", 1, 150.0


def test_keyframe_message_round_trip():
    _, sim = make_sim("harbor_fleet", SEED, 20.0, AvatarParams())
    for ad in sim.agents.values():
        for k, kf in enumerate(ad.keyframes):
            back = keyframe_data(keyframe_msg(kf, k))
            assert back.t == kf.t and back.abs_z == kf.abs_z
            for a, b in ((back.odom, kf.odom), (back.odom_sigmas, kf.odom_sigmas)):
                assert (a is None and b is None) or np.array_equal(a, b)
            assert len(back.detections) == len(kf.detections)
            for d, e in zip(back.detections, kf.detections, strict=True):
                assert (d.part_index, d.medium, d.modality, d.class_id, d.object_key) == (
                    e.part_index, e.medium, e.modality, e.class_id, e.object_key
                )  # fmt: skip
                for f in ("p_body", "sigmas", "extent", "descriptor"):
                    assert np.array_equal(getattr(d, f), getattr(e, f))
    # a Tier-2 detection carries its front-end's structure key
    kf.detections.append(
        Detection(7, Medium.BELOW, LandmarkFlags.SONAR, np.ones(3), np.ones(3), np.zeros(3), 0,
                  np.zeros(0), object_key=123)
    )  # fmt: skip
    assert keyframe_data(keyframe_msg(kf, 0)).detections[-1].object_key == 123


def test_team_merges_in_ros2():
    """Every node in one process; the clock runs free. The anchor's estimates of the others'
    frames meet gate G1, as offline (the offline run of this seed merges at 1 kbit/s)."""
    common = [
        Parameter("scenario_args", value=SCENARIO_ARGS),
        Parameter("seed", value=SEED),
        Parameter("duration_s", value=DURATION),
    ]
    scenario, sim = make_sim(
        "harbor_fleet", SEED, DURATION, AvatarParams(), acoustic="acoustic_generic"
    )
    end_s = float(next(iter(sim.agents.values())).times[-1])
    rclpy.init()
    try:
        ex = SingleThreadedExecutor()
        agents = {}
        for a in scenario.agents:
            who = [Parameter("agent", value=a.name)]
            if a.role == "slam":
                ex.add_node(FrontendReplay(parameter_overrides=common + who))
                agents[a.agent_id] = AgentNode(parameter_overrides=common + who)
                ex.add_node(agents[a.agent_id])
            else:
                ex.add_node(GatewayNode(parameter_overrides=common + who))
        ex.add_node(CommEmulator(parameter_overrides=common))
        clock = SimClock(
            parameter_overrides=[Parameter("end_s", value=end_s), Parameter("rate", value=30.0)]
        )
        ex.add_node(clock)
        deadline = time.monotonic() + 600
        while not clock.finished:
            assert time.monotonic() < deadline, "timed out"
            ex.spin_once(timeout_sec=0.05)
        for node in agents.values():
            node.finish()
    finally:
        rclpy.try_shutdown()
    assert all(node.ag.k == len(sim.agents[i].keyframes) - 1 for i, node in agents.items())
    assert all(node.exchanges >= 5 for node in agents.values())
    anchor = agents[scenario.anchor_id].ag
    frames = anchor.team_frames()
    assert set(frames) == set(agents)  # the whole team in the anchor's frame
    gt = {i: a.T_world_from_local for i, a in sim.agents.items()}
    for j, T in frames.items():
        e_xy, _ = frame_error(T, compose(inverse(gt[scenario.anchor_id]), gt[j]))
        assert e_xy < 1.0, (j, e_xy)
