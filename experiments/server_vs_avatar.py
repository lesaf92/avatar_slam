#!/usr/bin/env python3
"""Avatar SLAM vs. an *A&B*-style centralized server with every byte counted (T-E2-05).

For each acoustic profile and seed, runs on the same measurements: Avatar
decentralized, the centralized server baseline at several acoustic keyframe
strides, and the centralized oracle (ground-truth association, unlimited comm).

    python experiments/server_vs_avatar.py --seeds 0 1 2 3 4 \\
        --profiles m64 x150 acoustic_generic --out results/server_vs_avatar.csv

Scratch output goes to ``results/``. Paper numbers go through ``--out paper/data/...``.
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
from avatar.baselines.centralized_server import ServerParams, run_server
from avatar.runner import make_sim, run_centralized, run_decentralized
from avatar.types import Domain

HERE = Path(__file__).resolve().parent


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


def run_one(job: tuple[str, str, int, list[int], float | None]) -> list[dict]:
    preset, profile, seed, strides, duration_s = job
    doc = yaml.safe_load((HERE / "scenarios" / f"{preset}.yaml").read_text())
    args = {**doc["args"], "acoustic": profile}
    duration = duration_s or float(doc.get("duration_s", 600))
    params = AvatarParams()
    scenario, sim = make_sim(doc["scenario"], seed, duration, params, **args)
    under = [a.agent_id for a in scenario.agents if a.domain == Domain.UNDERWATER]
    base = {"preset": preset, "acoustic": profile, "seed": seed, "duration_s": duration}
    runs = [("avatar", None, run_decentralized(scenario, sim, params, seed).metrics)]
    for s in strides:
        m = run_server(scenario, sim, params, seed, ServerParams(acoustic_stride=s)).metrics
        runs.append(("server", s, m))
    runs.append(("oracle", None, run_centralized(scenario, sim, params, seed).metrics))
    rows = []
    for method, stride, m in runs:
        comm = m.get("comm", {})
        delivered = m.get("delivered_fraction", {})
        rows.append(
            {
                **base,
                "method": method,
                "acoustic_stride": stride if stride is not None else "",
                "team_ate_m": m["ate_team_m"],
                "n_connected": len(m["connected"]),
                "team_connected_s": m.get("team_connected_s", ""),
                "acoustic_bytes": comm.get("ACOUSTIC", {}).get("bytes_sent", ""),
                "rf_bytes": comm.get("RF", {}).get("bytes_sent", ""),
                "uuv_delivered_fraction": (
                    float(np.mean([delivered[i] for i in under])) if delivered else ""
                ),
            }
        )
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--preset", default="fleet_default")
    ap.add_argument("--profiles", nargs="+", default=["m64", "x150", "acoustic_generic"])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--strides", type=int, nargs="+", default=[10, 30])
    ap.add_argument("--duration", type=float, default=None, help="override the preset's")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="results/server_vs_avatar.csv")
    args = ap.parse_args()
    jobs = [
        (args.preset, p, s, args.strides, args.duration) for p in args.profiles for s in args.seeds
    ]
    rev = commit()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = [r | {"commit": rev} for rs in pool.map(run_one, jobs) for r in rs]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"{'acoustic':>16} | {'method':>10} | team ATE m | all connected | connected s | UUV kf")
    for p in args.profiles:
        for method, stride in [("avatar", ""), *[("server", s) for s in args.strides],
                               ("oracle", "")]:  # fmt: skip
            sel = [
                r for r in rows
                if r["acoustic"] == p and r["method"] == method and r["acoustic_stride"] == stride
            ]  # fmt: skip
            n_agents = max(r["n_connected"] for r in rows)
            full = sum(r["n_connected"] == n_agents for r in sel)
            conn = [r["team_connected_s"] for r in sel if r["team_connected_s"] not in ("", None)]
            uuv = [r["uuv_delivered_fraction"] for r in sel if r["uuv_delivered_fraction"] != ""]
            label = f"{method}{'/' + str(stride) if stride != '' else ''}"
            print(
                f"{p:>16} | {label:>10} | {np.mean([r['team_ate_m'] for r in sel]):10.3f} | "
                f"{full:>6}/{len(sel)}     | "
                f"{np.median(conn) if conn else float('nan'):11.1f} | "
                f"{np.mean(uuv) if uuv else float('nan'):6.2f}"
            )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
