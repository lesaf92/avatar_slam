#!/usr/bin/env python3
"""Per-robot evaluation of the EKF tracker over parameter grids (task T-F3-02, LOG L29).

For one robot and a set of seeds, runs the Tier-2 front-end with the EKF tracker and
that robot's single-agent SLAM (no other robot is simulated), and prints the robot's
ATE and the association statistics for every combination of the given
:class:`~avatar.frontend.ekf_tracker.EkfTrackerParams` values. Much faster than a
team study, and it isolates one robot's association.

    python experiments/tier2_ekf_sweep.py uuv_1 0 1 2 3 4 -- \
        landmark_density_per_m2=0.01,0.003 new_landmark_margin=1.0,0.0

``TIER2_RUNS`` selects another recording directory (default ``results/tier2``).
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import dataclasses
import itertools
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.eval.metrics import ate_rmse
from avatar.frontend.ekf_tracker import EkfTrackerParams
from avatar.runner import _make_agents, make_sim
from avatar.tier2.dataset import load_meta
from avatar.tier2.frontend import FrontEndParams, detections_for_agent
from avatar.tier2.sdf import GZ_SENSORS


def _parse(default: object, text: str) -> object:
    """``text`` as the type of ``default`` (booleans: 0/1/true/false)."""
    if isinstance(default, bool):
        return text.lower() in ("1", "true", "yes")
    return type(default)(text)


RUNS = Path(os.environ.get("TIER2_RUNS", Path(__file__).resolve().parents[1] / "results" / "tier2"))


def _one(args: tuple[str, int, dict]) -> tuple[dict, float, dict]:
    who, seed, overrides = args
    params = AvatarParams()
    run = RUNS / f"harbor_fleet_seed{seed}"
    meta = load_meta(run)
    scenario, sim = make_sim(meta["scenario"], seed, float(meta["duration_s"]), params)
    raw = np.load(run / "raw.npz")
    aid = next(a for a, ad in sim.agents.items() if ad.config.name == who)
    ad = sim.agents[aid]
    specs = {s: GZ_SENSORS[s] for s in meta["rigs"][who]}
    data = {s: raw[f"{who}/{s}"] for s in specs}
    fe = FrontEndParams(tracking="ekf", ekf=EkfTrackerParams(**overrides))
    rng = np.random.default_rng(seed + 40_000)
    dets, stats = detections_for_agent(
        ad, data, specs, sim.world, sim.instance_descriptors, 10_000, fe, rng
    )
    kfs = [dataclasses.replace(kf, detections=d) for kf, d in zip(ad.keyframes, dets, strict=True)]
    agent = _make_agents(scenario, sim, params, seed)[aid]
    next_solve = params.exchange_period_s
    for kf, t in zip(kfs, ad.times, strict=True):
        agent.on_keyframe(kf)
        if t + 1e-9 >= next_solve:
            next_solve += params.exchange_period_s
            agent.solve_local(with_marginals=False)
    agent.solve_local(with_marginals=False)
    return overrides, ate_rmse(agent.trajectory("local"), ad.gt), stats


def main() -> None:
    who = sys.argv[1]
    sep = sys.argv.index("--")
    seeds = [int(s) for s in sys.argv[2:sep]]
    defaults = EkfTrackerParams()
    grid = {}
    for tok in sys.argv[sep + 1 :]:
        key, vals = tok.split("=")
        grid[key] = [_parse(getattr(defaults, key), v) for v in vals.split(",")]
    combos = [dict(zip(grid, c, strict=True)) for c in itertools.product(*grid.values())] or [{}]
    jobs = [(who, s, c) for c in combos for s in seeds]
    with ProcessPoolExecutor(max_workers=12) as ex:
        results = list(ex.map(_one, jobs))
    for combo in combos:
        rs = [r for r in results if r[0] == combo]
        ate = np.array([r[1] for r in rs])
        label = " ".join(f"{k}={v}" for k, v in combo.items()) or "defaults"
        print(
            f"{label:60s} solo ATE mean {ate.mean():.2f} median {np.median(ate):.2f} "
            f"max {ate.max():.2f} (>0.5 m: {(ate > 0.5).sum()}/{len(ate)}) | "
            f"wrong {sum(r[2]['wrong_track'] for r in rs)} "
            f"tracks {sum(r[2]['tracks'] for r in rs)} "
            f"dropped {sum(r[2]['dropped_ambiguous'] for r in rs)} "
            f"of {sum(r[2]['matched'] for r in rs)} matched"
        )


if __name__ == "__main__":
    main()
