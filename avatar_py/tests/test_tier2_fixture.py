"""Tier-2 front-end on a small real recording (T-I1-05): CI without Gazebo.

``testdata/tier2`` holds the first 61 keyframes of one Gazebo recording (raw ranges of the
reference fleet's sensors, under 1 MB) and the statistics the front-end produced on
them (``golden.json``, written by ``tools/gen_tier2_fixture.py``). A change to
segmentation, gating or the trackers moves these numbers; if the change is intended,
regenerate the fixture and say why in the commit.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from avatar.agent import AvatarParams
from avatar.tier2.dataset import build_tier2_sim
from avatar.tier2.frontend import FrontEndParams

FIXTURE = Path(__file__).resolve().parents[2] / "testdata" / "tier2"
GOLDEN = json.loads((FIXTURE / "golden.json").read_text())
KEYS = ("clusters", "matched", "spurious", "tracks", "wrong_track", "dropped_ambiguous")


def _close(got: int, want: int) -> bool:
    """Equal up to floating-point differences across NumPy/SciPy versions (about 3 %)."""
    return abs(got - want) <= max(2, round(0.03 * want))


@pytest.fixture(scope="module")
def runs():
    out = {}
    for mode in GOLDEN:
        _, sim, stats = build_tier2_sim(
            FIXTURE, AvatarParams(), FrontEndParams(tracking=mode), cache=False
        )
        det = {
            a.config.name: sum(len(kf.detections) for kf in a.keyframes)
            for a in sim.agents.values()
            if a.config.role == "slam"
        }
        out[mode] = (stats, det)
    return out


@pytest.mark.parametrize("mode", sorted(GOLDEN))
def test_front_end_statistics_match_the_golden_values(runs, mode):
    stats, det = runs[mode]
    for agent, want in GOLDEN[mode].items():
        for key in KEYS:
            assert _close(int(stats[agent].get(key, 0)), want[key]), (mode, agent, key)
        assert _close(det[agent], want["detections"]), (mode, agent, "detections")


def test_every_robot_sees_something_and_the_fixture_is_small():
    assert (FIXTURE / "raw.npz").stat().st_size < 1_000_000
    for agent, want in GOLDEN["oracle"].items():
        assert want["clusters"] > 0 and want["detections"] > 0, agent


def test_ekf_tracker_is_no_worse_than_the_nearest_neighbour_tracker(runs):
    """On the recorded window the EKF tracker makes at most as many wrong tracks."""
    nn, ekf = runs["nn"][0], runs["ekf"][0]
    assert sum(v["wrong_track"] for v in ekf.values()) <= sum(v["wrong_track"] for v in nn.values())
