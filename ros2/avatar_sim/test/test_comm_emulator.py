"""T-S3-01 acceptance: the comm emulator gives Tier 1's link statistics on a replay.

A Tier-1 decentralized run records every transmission; the same packets, in the same order, go to
the node on ``/avatar/comm/tx``, ``/clock`` jumps to the end of the run, and the node's statistics
and deliveries must equal the run's.
"""

import time

import pytest

rclpy = pytest.importorskip("rclpy")

from avatar_msgs.msg import EncodedPacket  # noqa: E402
from avatar_sim.comm_emulator import QOS, CommEmulator  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

from avatar.agent import AvatarParams  # noqa: E402
from avatar.comm.network import Network  # noqa: E402
from avatar.runner import make_sim, run_decentralized  # noqa: E402

SEED, DURATION = 3, 300.0


def _stamp(msg_stamp, t: float) -> None:
    t = float(t)
    msg_stamp.sec = int(t)
    msg_stamp.nanosec = round((t - int(t)) * 1e9) % 10**9


def _spin_until(ex, cond, timeout_s: float = 60.0) -> None:
    end = time.monotonic() + timeout_s
    while not cond():
        assert time.monotonic() < end, "timed out"
        ex.spin_once(timeout_sec=0.05)


def test_replay_gives_tier1_link_statistics(monkeypatch):
    sent = []
    send = Network.send

    def record(self, t, sender, link, payload):
        sent.append((t, sender, int(link), payload))
        return send(self, t, sender, link, payload)

    monkeypatch.setattr(Network, "send", record)
    params = AvatarParams()
    scenario, sim = make_sim("harbor_fleet", SEED, DURATION, params)
    ref = run_decentralized(scenario, sim, params, SEED).metrics["comm"]
    monkeypatch.undo()
    t_end = float(next(iter(sim.agents.values())).times[-1])
    assert len(sent) > 50 and ref["ACOUSTIC"]["losses"] > 0  # both links busy, some losses

    rclpy.init()
    try:
        emu = CommEmulator(
            parameter_overrides=[
                Parameter("seed", value=SEED),
                Parameter("duration_s", value=DURATION),
            ]
        )
        drv = rclpy.create_node("replay")
        tx = drv.create_publisher(EncodedPacket, "/avatar/comm/tx", QOS)
        clock = drv.create_publisher(Clock, "/clock", QOS)
        got = []
        for a in scenario.agents:
            drv.create_subscription(
                EncodedPacket,
                f"/avatar/{a.name}/rx",
                lambda m, n=a.agent_id: got.append((n, m)),
                QOS,
            )
        ex = SingleThreadedExecutor()
        ex.add_node(emu)
        ex.add_node(drv)
        _spin_until(ex, lambda: tx.get_subscription_count() and clock.get_subscription_count())
        for t, sender, link, payload in sent:
            m = EncodedPacket()
            _stamp(m.header.stamp, t)
            m.sender_id, m.link_type, m.payload = sender, link, list(payload)
            tx.publish(m)
        _spin_until(ex, lambda: emu.received == len(sent))
        c = Clock()
        _stamp(c.clock, t_end)
        clock.publish(c)
        n_ref = sum(st["deliveries"] for st in ref.values())
        _spin_until(ex, lambda: len(got) == n_ref)
        assert emu.dropped == 0
        assert emu.stats() == ref
        # every delivery is a packet its sender sent on that link, to another agent
        sent_by = {(s, link, bytes(p)) for _, s, link, p in sent}
        assert all((m.sender_id, m.link_type, bytes(m.payload)) in sent_by for _, m in got)
        assert all(n != m.sender_id for n, m in got)
    finally:
        rclpy.try_shutdown()
