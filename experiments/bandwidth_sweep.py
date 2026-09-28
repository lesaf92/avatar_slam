#!/usr/bin/env python3
"""Acoustic bandwidth sweep × digest scheduler (T-C5-01, evidence for H2 / T-C2-01).

Runs Avatar decentralized on the same measurements for each acoustic bit rate
and scheduler (``fifo``: first observed, first sent; ``quality``: v0
heuristic; ``voi``: expected D-optimal alignment gain per byte), and records
when the team frame graph connects, the UUV frame errors in the anchor's
frame, the team ATE, and the acoustic bytes.

    python experiments/bandwidth_sweep.py --rates 16 32 64 128 256 512 1024 \\
        --seeds 0 1 2 3 4 --out results/bandwidth_sweep.csv
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
from avatar.runner import make_sim, run_decentralized
from avatar.types import Domain

HERE = Path(__file__).resolve().parent
SCHEDULERS = ("fifo", "quality", "voi")


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


def run_one(job: tuple[str, float, str, int]) -> dict:
    preset, bps, scheduler, seed = job
    doc = yaml.safe_load((HERE / "scenarios" / f"{preset}.yaml").read_text())
    duration = float(doc.get("duration_s", 600))
    params = AvatarParams(scheduler=scheduler)
    args = {**doc["args"], "acoustic_bps": bps}
    scenario, sim = make_sim(doc["scenario"], seed, duration, params, **args)
    m = run_decentralized(scenario, sim, params, seed).metrics
    under = [a.agent_id for a in scenario.agents if a.domain == Domain.UNDERWATER]
    ferr = [m["frame_error"][j]["xy_m"] for j in under if j in m["frame_error"]]
    n_slam = len(m["agents"])
    return {
        "preset": preset,
        "acoustic_bps": bps,
        "scheduler": scheduler,
        "seed": seed,
        "team_connected_s": m["team_connected_s"],
        "team_merged": len(m["connected"]) == n_slam,
        "team_ate_m": m["ate_team_m"],
        "uuv_frame_err_xy_m": float(np.mean(ferr)) if len(ferr) == len(under) else "",
        "acoustic_bytes": m["comm"].get("ACOUSTIC", {}).get("bytes_sent", 0),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--preset", default="fleet_default")
    ap.add_argument("--rates", type=float, nargs="+", default=[16, 32, 64, 128, 256, 512, 1024])
    ap.add_argument("--schedulers", nargs="+", default=list(SCHEDULERS), choices=SCHEDULERS)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="results/bandwidth_sweep.csv")
    args = ap.parse_args()
    jobs = [
        (args.preset, r, s, seed)
        for r in args.rates
        for s in args.schedulers
        for seed in args.seeds
    ]
    rev = commit()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = [r | {"commit": rev} for r in pool.map(run_one, jobs)]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(
        f"{'bps':>6} {'scheduler':>8} | merged | connected s (median) | UUV frame err m | "
        "team ATE m"
    )
    for r_ in args.rates:
        for s in args.schedulers:
            sel = [r for r in rows if r["acoustic_bps"] == r_ and r["scheduler"] == s]
            conn = [r["team_connected_s"] for r in sel if r["team_connected_s"] is not None]
            fe = [r["uuv_frame_err_xy_m"] for r in sel if r["uuv_frame_err_xy_m"] != ""]
            merged = [r for r in sel if r["team_merged"]]
            print(
                f"{r_:6.0f} {s:>8} | {len(merged):>2}/{len(sel):<3} | "
                f"{np.median(conn) if conn else float('nan'):20.0f} | "
                f"{np.mean(fe) if fe else float('nan'):15.3f} | "
                f"{np.mean([r['team_ate_m'] for r in merged]) if merged else float('nan'):10.3f}"
            )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
