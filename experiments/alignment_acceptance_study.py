#!/usr/bin/env python3
"""Acceptance of few-inlier alignments (task T-F3-05, docs/LOG.md L37).

``AvatarParams.confirm_weak_inliers``: a frame estimate with fewer inliers is used only when the
other direction of the pair agrees. Runs the development seeds for every perception the paper
uses and reports merges, gate G1, the wrong alignments (> 2 m or 5 degrees) among those used, by
pair type, and the time to merge, for each threshold (0 = off):

* ``T1``: Tier 1 (the recording's scenario and seed);
* Tier 2, reference fleet (``results/tier2``): proxy and DAVE sonar, ground-truth ids and EKF;
* Tier 2, UAV-LiDAR fleet (``results/tier2_uavlidar``): proxy, ground-truth ids and EKF.

    python experiments/alignment_acceptance_study.py --jobs 28
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import collections
import dataclasses
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_decentralized
from avatar.tier2.dataset import build_tier2_sim, load_meta
from avatar.tier2.frontend import FrontEndParams

RESULTS = Path(__file__).resolve().parent.parent / "results"
# (label, runs directory, tracking or None for Tier 1, sonar recording)
CONFIGS = [
    ("T1", "tier2", None, None),
    ("proxy GT ids", "tier2", "oracle", None),
    ("proxy EKF", "tier2", "ekf", None),
    ("sonar GT ids", "tier2", "oracle", "sonar"),
    ("sonar EKF", "tier2", "ekf", "sonar"),
    ("LiDAR fleet GT ids", "tier2_uavlidar", "oracle", None),
    ("LiDAR fleet EKF", "tier2_uavlidar", "ekf", None),
]


def _kind(a: str, b: str) -> str:
    if a.startswith("uuv") and b.startswith("uuv"):
        return "uuv-uuv"
    return "above" if "uuv" not in a + b else "cross"


def run_one(job: tuple[int, int, int, str, str, int]) -> dict:
    seed, ci, weak, results, policy, clique_seeds = job
    label, runs, tracking, sonar = CONFIGS[ci]
    run = Path(results) / runs / f"harbor_fleet_seed{seed}"
    base = AvatarParams()
    params = dataclasses.replace(
        base, confirm_weak_inliers=weak, confirm_weak_policy=policy,
        association=dataclasses.replace(base.association, clique_seeds=clique_seeds),
    )  # fmt: skip
    if tracking is None:
        meta = load_meta(run)
        scenario, sim = make_sim(
            meta["scenario"],
            seed,
            float(meta["duration_s"]),
            AvatarParams(),
            **meta["scenario_args"],
        )
    else:
        scenario, sim, _ = build_tier2_sim(
            run, AvatarParams(), FrontEndParams(tracking=tracking), sonar=sonar
        )
    names = {a.agent_id: a.name for a in scenario.agents}
    m = run_decentralized(scenario, sim, params, seed).metrics
    wrong = collections.Counter()
    unused = m["unused_alignments"]
    for i, d in m["alignment_errors"].items():
        for j, e in d.items():
            if j in unused.get(i, []):  # computed but not used (vetoed or unconfirmed)
                continue
            k = _kind(names[i], names[j])
            wrong[k + "_n"] += 1
            wrong[k + "_w"] += bool(e["xy_m"] > 2.0 or e["yaw_rad"] > np.deg2rad(5.0))
    ferr = [v["xy_m"] for v in m["frame_error"].values()]
    n_slam = sum(1 for a in scenario.agents if a.role == "slam")
    t = m["team_connected_s"]
    return {
        "config": label, "seed": seed, "weak": weak,
        "merged": len(m["connected"]) == n_slam,
        "g1": bool(len(ferr) == n_slam - 1 and max(ferr) < 1.0),
        "wrong": dict(wrong), "merge_s": None if t is None or not np.isfinite(t) else float(t),
        "ate_team_m": float(m["ate_team_m"]),
        "frame_err_m": {names[i]: float(v["xy_m"]) for i, v in m["frame_error"].items()},
    }  # fmt: skip


def _build(job: tuple[int, int, str]) -> None:
    seed, ci, results = job
    _, runs, tracking, sonar = CONFIGS[ci]
    if tracking is not None:
        run = Path(results) / runs / f"harbor_fleet_seed{seed}"
        build_tier2_sim(run, AvatarParams(), FrontEndParams(tracking=tracking), sonar=sonar)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--results", default=str(RESULTS))
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    ap.add_argument("--weak", type=int, nargs="+", default=[0, 6, 8, 10, 13])
    ap.add_argument("--configs", nargs="+", default=[c[0] for c in CONFIGS])
    ap.add_argument("--policy", default="veto", choices=["confirm", "veto"])
    ap.add_argument("--clique-seeds", type=int, default=AvatarParams().association.clique_seeds)
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", default=None, help="JSON lines of every run")
    args = ap.parse_args()
    cis = [i for i, c in enumerate(CONFIGS) if c[0] in args.configs]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        list(ex.map(_build, [(s, ci, args.results) for ci in cis for s in args.seeds]))
        jobs = [
            (s, ci, w, args.results, args.policy, args.clique_seeds)
            for ci in cis
            for w in args.weak
            for s in args.seeds
        ]
        rows = list(ex.map(run_one, jobs))
    if args.out:
        Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    for ci in cis:
        label = CONFIGS[ci][0]
        for w in args.weak:
            rs = [r for r in rows if r["config"] == label and r["weak"] == w]
            tot: collections.Counter = collections.Counter()
            for r in rs:
                tot.update(r["wrong"])
            ts = [r["merge_s"] for r in rs if r["merge_s"] is not None]
            print(
                f"{label:19s} weak<{w:2d}: merged {sum(r['merged'] for r in rs):2d}/{len(rs)} "
                f"G1 {sum(r['g1'] for r in rs):2d}/{len(rs)}  wrong: "
                f"cross {tot['cross_w']}/{tot['cross_n']}"
                f" uuv-uuv {tot['uuv-uuv_w']}/{tot['uuv-uuv_n']}"
                f" above {tot['above_w']}/{tot['above_n']}"
                f"  merge median {np.median(ts) if ts else float('nan'):.0f} s"
                f"  team ATE median {np.median([r['ate_team_m'] for r in rs]):.2f} m"
            )


if __name__ == "__main__":
    main()
