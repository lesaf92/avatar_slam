#!/usr/bin/env python3
"""Digest scheduler comparison (T-C2-01): v0 quality ordering vs VoI-per-byte.

Runs Avatar decentralized with each scheduler on the same measurements
(same preset, same seed) and records when the team frame graph connects, when
the first RF agent is aligned with an underwater agent, the team ATE, and the
bytes spent per link.

    python experiments/scheduler_comparison.py --seeds 0 1 2 3 4 \\
        --out results/scheduler_comparison.csv

Scratch output goes to ``results/``. Paper numbers go through ``--out paper/data/...``.
"""

from __future__ import annotations

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
SCHEDULERS = ("quality", "voi")


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


def run_one(job: tuple[str, int, str, float | None, str | None]) -> dict:
    preset, seed, scheduler, duration_s, acoustic = job
    doc = yaml.safe_load((HERE / "scenarios" / f"{preset}.yaml").read_text())
    args = dict(doc["args"])
    if acoustic:
        args["acoustic"] = acoustic
    duration = duration_s or float(doc.get("duration_s", 600))
    params = AvatarParams(scheduler=scheduler)
    scenario, sim = make_sim(doc["scenario"], seed, duration, params, **args)
    m = run_decentralized(scenario, sim, params, seed).metrics
    under = {a.agent_id for a in scenario.agents if a.domain == Domain.UNDERWATER}
    rf_side = {a.agent_id for a in scenario.agents if a.role == "slam"} - under
    cross = [
        t for i, d in m["first_alignment_s"].items() if i in rf_side for j, t in d.items()
        if j in under
    ]  # fmt: skip
    return {
        "preset": preset,
        "acoustic": args.get("acoustic", ""),
        "seed": seed,
        "scheduler": scheduler,
        "duration_s": duration,
        "team_connected_s": m["team_connected_s"],
        "first_rf_uuv_alignment_s": min(cross) if cross else None,
        "team_ate_m": m["ate_team_m"],
        "acoustic_bytes": m["comm"].get("ACOUSTIC", {}).get("bytes_sent", 0),
        "rf_bytes": m["comm"].get("RF", {}).get("bytes_sent", 0),
        "aligned_pairs": sum(len(d) for d in m["first_alignment_s"].values()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--presets", nargs="+", default=["fleet_default", "fleet_exploration"])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--acoustic", default=None, help="override the presets' acoustic profile")
    ap.add_argument("--duration", type=float, default=None, help="override the preset's")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="results/scheduler_comparison.csv")
    args = ap.parse_args()
    jobs = [
        (p, s, sch, args.duration, args.acoustic)
        for p in args.presets
        for s in args.seeds
        for sch in SCHEDULERS
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_one, jobs))
    rev = commit()
    for r in rows:
        r["commit"] = rev
        print(
            f"{r['preset']:>18} {r['acoustic']:>5} seed {r['seed']} {r['scheduler']:>7}: "
            f"connected {r['team_connected_s']}  rf-uuv {r['first_rf_uuv_alignment_s']}  "
            f"team {r['team_ate_m']:.3f}  acoustic {r['acoustic_bytes']} B",
            flush=True,
        )
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    def stat(sel: list[dict], key: str) -> str:
        v = np.array([np.nan if r[key] is None else r[key] for r in sel], dtype=float)
        done = int(np.isfinite(v).sum())
        return f"{np.nanmedian(v) if done else float('nan'):7.1f} ({done}/{len(v)})"

    print(f"\n{'preset':>18} | scheduler | connected s (n) | first RF-UUV s (n) | team ATE m")
    for p in args.presets:
        for sch in SCHEDULERS:
            sel = [r for r in rows if r["preset"] == p and r["scheduler"] == sch]
            ate = np.mean([r["team_ate_m"] for r in sel])
            print(
                f"{p:>18} | {sch:>9} | {stat(sel, 'team_connected_s'):>15} | "
                f"{stat(sel, 'first_rf_uuv_alignment_s'):>18} | {ate:.3f}"
            )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
