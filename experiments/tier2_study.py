#!/usr/bin/env python3
"""Tier-1 vs. Tier-2 (Gazebo) parity study and gate G1 (tasks T-S2-*, T-G1).

For every recorded seed (``experiments/gazebo/record.py``), runs the same
scenario, seed, odometry and estimator on two perception tiers:

* **T1**: the abstract landmark-part sensor models (``avatar.sim.sensors``);
* **T2**: Gazebo Harmonic ray-cast range data through the geometric front-end
  (``avatar.tier2``), with ground-truth intra-agent association as in Tier 1;
* **T2nn**: the same, with the nearest-neighbour tracker instead (realistic
  intra-agent association);
* **T2reg**: the same, with keyframe-to-map registration before the gated
  association (a negative result, docs/LOG.md L28);
* **T2ekf**: the same, with the EKF tracker (covariance gating, T-F3-02);
* **T2nn-uuv / T2nn-land**: controlled ablations of T2nn, in which only the
  BlueROV2s (``uuv``) or only the Husky and the Tarot (``land``) use the
  nearest-neighbour tracker and the others keep ground-truth tracks.

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
import dataclasses
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
# Tier 1); Tier 2 with the nearest-neighbour tracker (realistic front-end); Tier 2
# with registration-aided tracking.
TIERS: dict[str, str | dict[str, str] | None] = {
    "T1": None,
    "T2": "oracle",
    "T2nn": "nn",
    "T2reg": "registration",
    "T2ekf": "ekf",
    "T2nn-uuv": {"uuv": "nn", "*": "oracle"},
    "T2nn-land": {"ugv": "nn", "uav": "nn", "*": "oracle"},
}

WRONG_XY_M = 2.0
WRONG_YAW_RAD = np.deg2rad(5.0)
G1_XY_M = 1.0


def _modes(tier: str) -> list[str]:
    """Distinct tracking modes a Tier-2 tier uses (one front-end cache per mode)."""
    spec = TIERS[tier]
    if spec is None:
        return []
    return sorted({spec} if isinstance(spec, str) else set(spec.values()))


def _build(run_dir: str, mode: str) -> None:
    """Fill the front-end cache of one (run, mode); each pair has its own cache file."""
    build_tier2_sim(run_dir, AvatarParams(), FrontEndParams(tracking=mode))


def _tier2_sim(run_dir: str, params: AvatarParams, tier: str):
    """Scenario, SimData and front-end statistics of one Tier-2 tier.

    A mixed tier takes each agent's detections from the sim of its own tracking
    mode (matched on the agent name prefix, ``*`` for the rest).
    """
    spec = TIERS[tier]
    if isinstance(spec, str):
        return build_tier2_sim(run_dir, params, FrontEndParams(tracking=spec))
    built = {m: build_tier2_sim(run_dir, params, FrontEndParams(tracking=m)) for m in _modes(tier)}
    scenario, base, _ = next(iter(built.values()))
    agents, stats = dict(base.agents), {}
    for aid, ad in base.agents.items():
        if ad.config.role != "slam":
            continue
        name = ad.config.name
        mode = next((m for k, m in spec.items() if k != "*" and name.startswith(k)), spec["*"])
        _, sim_m, st_m = built[mode]
        agents[aid] = sim_m.agents[aid]
        if name in st_m:
            stats[name] = st_m[name]
    return scenario, dataclasses.replace(base, agents=agents), stats


def _one(run_dir: str, tier: str) -> dict:
    params = AvatarParams()
    meta = load_meta(run_dir)
    seed = int(meta["seed"])
    t0 = time.perf_counter()
    fe = {}
    if TIERS[tier] is not None:
        scenario, sim, fe = _tier2_sim(run_dir, params, tier)
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
    jobs = [(d, t) for d in dirs for t in args.tiers]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        # Front-end caches first (segmentation is the slow part): one cache file
        # per (run, tracking mode), so the builds are independent.
        built = sorted({(d, m) for d, t in jobs for m in _modes(t)})
        if built:
            list(ex.map(_build, *zip(*built, strict=True)))
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
