#!/usr/bin/env python3
"""Front-end errors (clutter, identity switches) vs. single-agent and team accuracy (T-S1-04).

For each preset, error level, robust-kernel setting and seed, runs on the same
measurements: independent single-agent SLAM, Avatar decentralized, and the
centralized oracle (ground-truth association, so it ignores the errors).

    python experiments/realism_study.py --seeds 0 1 2 --out results/realism_study.csv
"""

from __future__ import annotations

import os

# Small sparse solves run 5-10x slower with multi-threaded BLAS, and much worse
# when several runs share the CPU (docs/LOG.md L10). Must precede the NumPy import.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
import subprocess
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_decentralized, run_independent

HERE = Path(__file__).resolve().parent
LEVELS = {
    "none": {},
    "low": {"id_switch_prob": 0.05, "clutter_per_kf": 0.5},
    "high": {"id_switch_prob": 0.15, "clutter_per_kf": 1.0},
}


def commit() -> str:
    try:
        return subprocess.run(
            ["git", "describe", "--always", "--dirty", "--abbrev=7"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def run_one(job: tuple[str, str, float | None, int]) -> dict:
    preset, level, robust_k, seed = job
    doc = yaml.safe_load((HERE / "scenarios" / f"{preset}.yaml").read_text())
    duration = float(doc.get("duration_s", 600))
    params = AvatarParams(point_obs_robust_k=robust_k)
    args = {**doc["args"], "frontend_errors": LEVELS[level]}
    scenario, sim = make_sim(doc["scenario"], seed, duration, params, **args)
    ind = run_independent(scenario, sim, params, seed).metrics
    dec = run_decentralized(scenario, sim, params, seed).metrics
    cen = run_centralized(scenario, sim, params, seed).metrics
    row = {
        "preset": preset,
        "errors": level,
        "robust_k": robust_k if robust_k is not None else "",
        "seed": seed,
        "team_ate_dec_m": dec["ate_team_m"],
        "team_ate_cen_m": cen["ate_team_m"],
        "n_connected_dec": len(dec["connected"]),
        "team_connected_s": dec["team_connected_s"],
    }
    for i, name in dec["agents"].items():
        row[f"alone_{name}_m"] = ind["ate_local_m"][i]
        row[f"oracle_{name}_m"] = cen["ate_team_per_agent_m"].get(i, float("nan"))
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--presets", nargs="+", default=["fleet_default", "fleet_exploration"])
    ap.add_argument("--levels", nargs="+", default=list(LEVELS), choices=list(LEVELS))
    ap.add_argument("--robust", nargs="+", default=["none", "3"], help="Huber k or 'none'")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="results/realism_study.csv")
    args = ap.parse_args()
    ks = [None if r == "none" else float(r) for r in args.robust]
    jobs = [
        (p, lv, k, s) for p in args.presets for lv in args.levels for k in ks for s in args.seeds
    ]
    rev = commit()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = [r | {"commit": rev} for r in pool.map(run_one, jobs)]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cols = list(dict.fromkeys(k for r in rows for k in r))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    agents = [c[len("alone_") : -2] for c in cols if c.startswith("alone_")]
    print(f"{'preset':>18} {'errors':>6} {'k':>4} | " + " ".join(f"{a:>13}" for a in agents)
          + " | team dec | team cen | all conn")  # fmt: skip
    for p in args.presets:
        for lv in args.levels:
            for k in ks:
                sel = [r for r in rows if r["preset"] == p and r["errors"] == lv
                       and r["robust_k"] == (k if k is not None else "")]  # fmt: skip

                def m(key: str, sel=sel) -> float:
                    return float(np.nanmean([r[key] for r in sel]))

                per = " ".join(
                    f"{m(f'alone_{a}_m'):6.3f}/{m(f'oracle_{a}_m'):6.3f}" for a in agents
                )
                full = sum(r["n_connected_dec"] == len(agents) for r in sel)
                print(
                    f"{p:>18} {lv:>6} {k or '-':>4} | {per} | {m('team_ate_dec_m'):8.3f} | "
                    f"{m('team_ate_cen_m'):8.3f} | {full}/{len(sel)}"
                )
    print("(per agent: alone / oracle team ATE [m])")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
