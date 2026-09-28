#!/usr/bin/env python3
"""Impact of trajectory shape and height on single-agent drift and team consistency.

Runs each YAML preset (``experiments/scenarios/*.yaml``) for several seeds in
three modes on the same measurements: independent (single-agent SLAM), Avatar
decentralized, and the centralized oracle.

    python experiments/trajectory_study.py --seeds 0 1 2 --out results/trajectory_study.csv

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
from pathlib import Path

import numpy as np
import yaml
from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_centralized, run_decentralized, run_independent

HERE = Path(__file__).resolve().parent
DEFAULT_PRESETS = [
    "fleet_default",
    "fleet_complex_turns",
    "fleet_heights",
    "fleet_surfacing",
    "fleet_exploration",
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--presets", nargs="+", default=DEFAULT_PRESETS)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--duration", type=float, default=None, help="override the preset's")
    ap.add_argument("--out", default="results/trajectory_study.csv")
    ap.add_argument("--align-window", type=int, default=0, help="AvatarParams.align_window_kf")
    ap.add_argument("--no-heading-bias", dest="heading_bias", action="store_false",
                    help="do not model the heading bias (default: modelled, D9)")  # fmt: skip
    ap.add_argument("--gnc", type=float, default=4.03,
                    help="GNC-TLS bound on landmark obs (<= 0 disables)")  # fmt: skip
    args = ap.parse_args()
    params = AvatarParams(
        align_window_kf=args.align_window,
        model_heading_bias=args.heading_bias,
        point_obs_gnc=args.gnc if args.gnc > 0 else None,
    )
    rev = commit()
    rows = []
    for name in args.presets:
        doc = yaml.safe_load((HERE / "scenarios" / f"{name}.yaml").read_text())
        duration = args.duration or float(doc.get("duration_s", 600))
        for seed in args.seeds:
            scenario, sim = make_sim(doc["scenario"], seed, duration, params, **doc["args"])
            ind = run_independent(scenario, sim, params, seed).metrics
            dec = run_decentralized(scenario, sim, params, seed).metrics
            cen = run_centralized(scenario, sim, params, seed).metrics
            row = {
                "commit": rev,
                "preset": name,
                "align_window_kf": args.align_window,
                "heading_bias": args.heading_bias,
                "gnc": args.gnc if args.gnc is not None else "",
                "seed": seed,
                "duration_s": duration,
                "team_ate_dec_m": dec["ate_team_m"],
                "team_ate_cen_m": cen["ate_team_m"],
                "team_connected_s": dec["team_connected_s"],
                "rf_bytes": dec["comm"].get("RF", {}).get("bytes_sent", 0),
                "acoustic_bytes": dec["comm"].get("ACOUSTIC", {}).get("bytes_sent", 0),
            }
            for i, agent in dec["agents"].items():
                row[f"ate_alone_{agent}_m"] = ind["ate_local_m"][i]
                row[f"ate_fused_{agent}_m"] = dec["ate_fused_m"][i]
                row[f"ate_team_{agent}_m"] = dec["ate_team_per_agent_m"].get(i, float("nan"))
                row[f"ate_oracle_{agent}_m"] = cen["ate_team_per_agent_m"].get(i, float("nan"))
            rows.append(row)
            alone = {a: round(ind["ate_local_m"][i], 3) for i, a in dec["agents"].items()}
            print(
                f"{name:>20} seed {seed}: alone {alone}  team dec {row['team_ate_dec_m']:.3f} "
                f"cen {row['team_ate_cen_m']:.3f}  connected {row['team_connected_s']}",
                flush=True,
            )
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cols = list(dict.fromkeys(k for r in rows for k in r))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"\n{'preset':>20} | alone uuv_0 | alone uuv_1 | alone uav_0 | team dec | team cen")
    for name in args.presets:
        sel = [r for r in rows if r["preset"] == name]

        def m(key: str, sel=sel) -> float:
            return float(np.nanmean([r.get(key, np.nan) for r in sel]))

        print(
            f"{name:>20} | {m('ate_alone_uuv_0_m'):11.3f} | {m('ate_alone_uuv_1_m'):11.3f} | "
            f"{m('ate_alone_uav_0_m'):11.3f} | {m('team_ate_dec_m'):8.3f} | "
            f"{m('team_ate_cen_m'):8.3f}"
        )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
