#!/usr/bin/env python3
"""Reference fleet (harbor_fleet) across acoustic modem profiles and seeds.

For each (acoustic profile, seed) this runs the decentralized system and the
centralized oracle on the same measurements, and records team ATE, time until
the anchor's frame chain covers the whole team, and bytes per link type.

    python experiments/fleet_acoustic_sweep.py --seeds 0 1 2 3 4 --duration 600 \\
        --profiles m64 x150 acoustic_generic --out results/fleet_acoustic_sweep.csv

Scratch output goes to ``results/`` (git-ignored). Paper numbers must be
regenerated into ``paper/data/`` with ``--out paper/data/...`` (AGENTS.md §5).
"""

from __future__ import annotations

import os

# Small sparse solves run 5-10x slower with multi-threaded BLAS, and much worse
# when several runs share the CPU (docs/LOG.md L10). Must precede the NumPy import.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
from pathlib import Path

import numpy as np
from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_decentralized


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--profiles", nargs="+", default=["m64", "x150", "acoustic_generic"])
    ap.add_argument("--out", default="results/fleet_acoustic_sweep.csv")
    args = ap.parse_args()
    params = AvatarParams()
    rev = commit()
    rows = []
    for profile in args.profiles:
        for seed in args.seeds:
            scenario, sim = make_sim("harbor_fleet", seed, args.duration, params, acoustic=profile)
            dec = run_decentralized(scenario, sim, params, seed).metrics
            cen = run_centralized(scenario, sim, params, seed).metrics
            row = {
                "commit": rev,
                "profile": profile,
                "seed": seed,
                "duration_s": args.duration,
                "team_ate_dec_m": dec["ate_team_m"],
                "team_ate_cen_m": cen["ate_team_m"],
                "team_connected_s": dec["team_connected_s"],
                "disconnected": ";".join(dec["agents"][i] for i in dec["disconnected"]),
                "rf_bytes": dec["comm"].get("RF", {}).get("bytes_sent", 0),
                "acoustic_bytes": dec["comm"].get("ACOUSTIC", {}).get("bytes_sent", 0),
                "acoustic_losses": dec["comm"].get("ACOUSTIC", {}).get("losses", 0),
            }
            for i, name in dec["agents"].items():
                row[f"ate_team_{name}_m"] = dec["ate_team_per_agent_m"].get(i, float("nan"))
            rows.append(row)
            print(
                f"{profile:>16} seed {seed}: team ATE dec {row['team_ate_dec_m']:.3f} m, "
                f"cen {row['team_ate_cen_m']:.3f} m, connected at {row['team_connected_s']} s, "
                f"acoustic {row['acoustic_bytes']} B",
                flush=True,
            )
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cols = list(dict.fromkeys(k for r in rows for k in r))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    for profile in args.profiles:
        sel = [r for r in rows if r["profile"] == profile]
        conn = [r["team_connected_s"] for r in sel if r["team_connected_s"] is not None]
        print(
            f"{profile:>16}: team ATE dec {np.mean([r['team_ate_dec_m'] for r in sel]):.3f} "
            f"vs cen {np.mean([r['team_ate_cen_m'] for r in sel]):.3f} m; connected in "
            f"{len(conn)}/{len(sel)} runs, median {np.median(conn) if conn else float('nan'):.0f} s"
        )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
