#!/usr/bin/env python3
"""Tier-1 vs. Tier-2 (Gazebo) parity study and gate G1 (tasks T-S2-*, T-G1).

For every recorded seed (``experiments/gazebo/record.py``), runs the same
scenario, seed, odometry and estimator on two perception tiers:

* **T1**: the abstract landmark-part sensor models (``avatar.sim.sensors``);
* **T2**: Gazebo Harmonic ray-cast range data through the geometric front-end
  (``avatar.tier2``), with ground-truth intra-agent association as in Tier 1;
* **T2nn**: the same, with the nearest-neighbour tracker instead (realistic
  intra-agent association).

Modes: independent (single-agent SLAM), Avatar (decentralized) and the
centralized oracle (ground-truth association). On T2 the oracle is also run
with the agents' GNC kernel (``oracle_robust``), because T2 detections of a
true part can still be off by up to the labelling radius.

An accepted pairwise alignment counts as **wrong** if its frame error exceeds
2 m or 5° (a pile spacing is 6 m, so aliasing errors are far larger). Gate G1
(docs/PLAN.md §5): frame error of every robot in the anchor's frame < 1 m.

    python experiments/tier2_study.py --runs results/tier2 --seeds 0 1 2 \
        --out results/tier2_study.csv
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_decentralized, run_independent
from avatar.tier2.dataset import build_tier2_sim, load_meta
from avatar.tier2.frontend import FrontEndParams

# Perception tiers: Tier 1; Tier 2 with ground-truth intra-agent association (as
# Tier 1); Tier 2 with the nearest-neighbour tracker (realistic front-end).
TIERS = {"T1": None, "T2": "oracle", "T2nn": "nn"}

WRONG_XY_M = 2.0
WRONG_YAW_RAD = np.deg2rad(5.0)
G1_XY_M = 1.0


def _one(run_dir: str, tier: str) -> dict:
    params = AvatarParams()
    meta = load_meta(run_dir)
    seed = int(meta["seed"])
    t0 = time.perf_counter()
    fe = {}
    if TIERS[tier] is not None:
        fe_params = FrontEndParams(tracking=TIERS[tier])
        scenario, sim, fe = build_tier2_sim(run_dir, params, fe_params)
    else:
        scenario, sim = make_sim(
            meta["scenario"], seed, float(meta["duration_s"]), params, **meta["scenario_args"]
        )
    names = {i: a.config.name for i, a in sim.agents.items() if a.config.role == "slam"}
    ind = run_independent(scenario, sim, params, seed).metrics
    dec = run_decentralized(scenario, sim, params, seed).metrics
    cen = run_centralized(scenario, sim, params, seed).metrics
    cen_r = run_centralized(scenario, sim, params, seed, robust=True).metrics
    al = [
        (e["xy_m"], e["yaw_rad"]) for per in dec["alignment_errors"].values() for e in per.values()
    ]
    wrong = sum(1 for xy, yaw in al if xy > WRONG_XY_M or yaw > WRONG_YAW_RAD)
    ferr = [v["xy_m"] for v in dec["frame_error"].values()]
    n_det = sum(len(kf.detections) for a in sim.agents.values() for kf in a.keyframes)
    row = {
        "tier": tier,
        "seed": seed,
        "duration_s": meta["duration_s"],
        "team_ate_avatar_m": dec["ate_team_m"],
        "team_ate_oracle_m": cen["ate_team_m"],
        "team_ate_oracle_robust_m": cen_r["ate_team_m"],
        "team_connected_s": dec["team_connected_s"],
        "n_connected": len(dec["connected"]),
        "n_slam": len(names),
        "alignments": len(al),
        "wrong_alignments": wrong,
        "frame_err_max_m": max(ferr) if len(ferr) == len(names) - 1 else float("nan"),
        "g1_pass": bool(len(ferr) == len(names) - 1 and max(ferr) < G1_XY_M),
        "detections": n_det,
        "wall_s": time.perf_counter() - t0,
    }
    for i, n in names.items():
        row[f"ate_solo_{n}_m"] = ind["ate_local_m"][i]
        row[f"ate_fused_{n}_m"] = dec["ate_fused_m"][i]
        row[f"ate_team_{n}_m"] = dec["ate_team_per_agent_m"].get(i, float("nan"))
    for n, st in fe.items():
        row[f"fe_{n}"] = json.dumps(st, sort_keys=True)
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", default="results/tier2")
    ap.add_argument("--scenario", default="harbor_fleet")
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--tiers", nargs="+", default=list(TIERS), choices=list(TIERS))
    ap.add_argument("--out", default="results/tier2_study.csv")
    args = ap.parse_args()
    rev = commit()
    dirs = [str(Path(args.runs) / f"{args.scenario}_seed{s}") for s in args.seeds]
    for d in dirs:
        if not (Path(d) / "raw.npz").exists():
            raise SystemExit(f"{d}: not recorded (experiments/gazebo/record.py)")
    # Build the T2 front-end caches first (sequentially: one cache file per run).
    for d in dirs:
        for t in args.tiers:
            if TIERS[t] is not None:
                build_tier2_sim(d, AvatarParams(), FrontEndParams(tracking=TIERS[t]))
    jobs = [(d, t) for d in dirs for t in args.tiers]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        rows = list(ex.map(_one, *zip(*jobs, strict=True)))
    keys: list[str] = []
    for r in rows:
        r["commit"] = rev
        keys += [k for k in r if k not in keys]
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    for tier in args.tiers:
        rs = [r for r in rows if r["tier"] == tier]
        print(
            f"{tier}: team ATE Avatar {np.mean([r['team_ate_avatar_m'] for r in rs]):.3f} m, "
            f"oracle {np.mean([r['team_ate_oracle_m'] for r in rs]):.3f} m, "
            f"oracle+GNC {np.mean([r['team_ate_oracle_robust_m'] for r in rs]):.3f} m; "
            f"merged {sum(r['n_connected'] == r['n_slam'] for r in rs)}/{len(rs)}; "
            f"wrong alignments {sum(r['wrong_alignments'] for r in rs)}"
            f"/{sum(r['alignments'] for r in rs)}; G1 {sum(r['g1_pass'] for r in rs)}/{len(rs)}"
        )
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
