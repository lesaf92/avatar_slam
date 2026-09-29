#!/usr/bin/env python3
"""Why the Tier-2 nearest-neighbour tracker fails (tasks T-F3-01, T-F3-02; docs/LOG.md L28).

For one recorded run, prints per SLAM agent:

1. the raw dead-reckoning error (start-aligned, horizontal) that the tracker's
   gate has to absorb;
2. the tracker's tracks, and how many tracks each true part received;
3. the wrong-track events by kind (a spurious cluster started the track, or two
   different true parts share it, with the distance between them);
4. with ``--sweep``, the agents' single-agent ATE for a grid of gate settings.

    python experiments/tier2_tracker_diagnostics.py results/tier2/harbor_fleet_seed0
    python experiments/tier2_tracker_diagnostics.py results/tier2/harbor_fleet_seed1 --sweep
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import collections
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from avatar.agent import AvatarParams
from avatar.geometry import compose
from avatar.runner import run_independent
from avatar.tier2.dataset import build_tier2_sim, load_meta
from avatar.tier2.frontend import FrontEndParams


def dead_reckoning_error(agent) -> np.ndarray:
    """Horizontal error [m] per keyframe of the raw odometry chain vs. ground truth.

    Both are expressed in the agent's first pose, which is what a tracker in the
    dead-reckoning frame sees (no SLAM correction).
    """
    dr = np.zeros(4)
    xy = [dr[:2].copy()]
    for kf in agent.keyframes[1:]:
        dr = compose(dr, kf.odom)
        xy.append(dr[:2].copy())
    gt = agent.gt
    c, s = np.cos(gt[0, 3]), np.sin(gt[0, 3])
    rel = (gt[:, :2] - gt[0, :2]) @ np.array([[c, s], [-s, c]]).T
    return np.linalg.norm(np.array(xy) - rel, axis=1)


def track_report(agent, world) -> dict:
    """Track counts and wrong-track events of one agent's detections."""
    first: dict[int, int] = {}
    kinds: collections.Counter[str] = collections.Counter()
    dists: list[float] = []
    parts_of_track: dict[int, collections.Counter] = collections.defaultdict(collections.Counter)
    for kf in agent.keyframes:
        for d in kf.detections:
            parts_of_track[d.part_index][d.true_part_index] += 1
            f = first.setdefault(d.part_index, d.true_part_index)
            if f == d.true_part_index:
                continue
            if f < 0:
                kinds["seeded by a spurious cluster"] += 1
            elif d.true_part_index < 0:
                kinds["absorbed a spurious cluster"] += 1
            else:
                kinds["two different parts"] += 1
                pf, pc = world.parts[f], world.parts[d.true_part_index]
                dists.append(float(np.hypot(*(pf.position[:2] - pc.position[:2]))))
    tracks_per_part: collections.Counter[int] = collections.Counter()
    for counts in parts_of_track.values():
        for t in counts:
            if t >= 0:
                tracks_per_part[t] += 1
    return {
        "tracks": len(parts_of_track),
        "parts_seen": len(tracks_per_part),
        "tracks_per_part_median": float(np.median(list(tracks_per_part.values())))
        if tracks_per_part
        else 0.0,
        "tracks_per_part_max": max(tracks_per_part.values(), default=0),
        "kinds": dict(kinds),
        "confusion_distance_m": (float(np.median(dists)), min(dists), max(dists))
        if dists
        else None,
    }


def _sweep_one(args: tuple[str, float, float, str]) -> tuple[float, float, dict, dict]:
    run_dir, gate, growth, mode = args
    params = AvatarParams()
    fe = FrontEndParams(tracking=mode, track_gate_m=gate, track_drift_per_m=growth)
    scenario, sim, stats = build_tier2_sim(run_dir, params, fe, cache=False)
    seed = int(load_meta(run_dir)["seed"])
    m = run_independent(scenario, sim, params, seed).metrics
    names = {i: a.config.name for i, a in sim.agents.items() if a.config.role == "slam"}
    ate = {n: m["ate_local_m"][i] for i, n in names.items()}
    wrong = {n: (stats[n]["wrong_track"], stats[n]["tracks"]) for n in names.values()}
    return gate, growth, ate, wrong


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir")
    ap.add_argument("--mode", default="nn", choices=["nn", "registration"])
    ap.add_argument("--sweep", action="store_true", help="grid over gate and gate growth")
    ap.add_argument("--jobs", type=int, default=8)
    args = ap.parse_args()

    _, sim, stats = build_tier2_sim(
        args.run_dir, AvatarParams(), FrontEndParams(tracking=args.mode)
    )
    print(f"{args.run_dir}, tracker '{args.mode}'")
    for ad in sim.agents.values():
        if ad.config.role != "slam":
            continue
        err = dead_reckoning_error(ad)
        rep = track_report(ad, sim.world)
        n = ad.config.name
        print(
            f"\n{n}: dead-reckoning error peak {err.max():.1f} m, end {err[-1]:.1f} m; "
            f"{rep['tracks']} tracks for {rep['parts_seen']} parts seen "
            f"(per part: median {rep['tracks_per_part_median']:.0f}, max "
            f"{rep['tracks_per_part_max']}); wrong-track detections "
            f"{stats[n]['wrong_track']}; dropped as ambiguous {stats[n]['dropped_ambiguous']}"
        )
        print(f"   events: {rep['kinds']}")
        if rep["confusion_distance_m"]:
            med, lo, hi = rep["confusion_distance_m"]
            print(f"   two-part confusions {med:.1f} m apart (median; min {lo:.1f}, max {hi:.1f})")
    if args.sweep:
        grid = [
            (args.run_dir, g, d, args.mode) for g in (1.0, 0.7) for d in (0.02, 0.01, 0.005, 0.0)
        ]
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            results = list(ex.map(_sweep_one, grid))
        print("\nsingle-agent ATE [m] by gate [m] and gate growth [m per m travelled]:")
        for gate, growth, ate, wrong in results:
            cells = "  ".join(f"{n} {a:.2f}" for n, a in ate.items())
            w = " ".join(f"{n}:{wt[0]}/{wt[1]}" for n, wt in wrong.items())
            print(f"  gate {gate:.1f} growth {growth:.3f} | {cells} | wrong/tracks {w}")


if __name__ == "__main__":
    main()
