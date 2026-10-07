#!/usr/bin/env python3
"""Does collaboration hurt the drifting AUV? Solo vs fused vs the centralized oracle (T-X1-04).

For each seed of a preset and each variant, runs the three modes on the same measurements and
writes every agent's ATE alone (independent SLAM), fused (Avatar, its own fused graph) and in the
centralized oracle (its own trajectory, aligned per agent like the other two), plus team ATE:

- ``base``: the defaults.
- ``scale``: the estimators also carry the translation and gyro scale-error states
  (``AvatarParams.model_odometry_scale``).
- ``nosys``: the defaults on a simulation without the platforms' scale errors (the draws are
  kept, at σ = 1e-12, so every other random stream is unchanged): is the harm caused by them?

    python experiments/never_hurt_study.py --seeds 0 1 2 3 4 5 6 7 8 9 --out results/never_hurt.csv

Scratch output goes to ``results/`` (LOG L46).
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
import dataclasses
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import yaml
from _provenance import commit

import avatar.runner as runner
from avatar.agent import AvatarParams
from avatar.eval.metrics import ate_rmse
from avatar.sim import agents as sim_agents

HERE = Path(__file__).resolve().parent
VARIANTS = {
    "base": AvatarParams(),
    "scale": AvatarParams(model_odometry_scale=True),
    "nosys": AvatarParams(),
}


def run_one(preset: str, seed: int, variant: str) -> dict:
    doc = yaml.safe_load((HERE / "scenarios" / f"{preset}.yaml").read_text())
    params = VARIANTS[variant]
    saved = dict(sim_agents.PLATFORM_ODOMETRY)
    if variant == "nosys":
        for k, n in saved.items():
            sim_agents.PLATFORM_ODOMETRY[k] = dataclasses.replace(
                n,
                scale_bias_std=1e-12 if n.scale_bias_std else 0.0,
                yaw_scale_bias_std=1e-12 if n.yaw_scale_bias_std else 0.0,
            )
    try:
        duration = float(doc.get("duration_s", 600))
        scenario, sim = runner.make_sim(doc["scenario"], seed, duration, params, **doc["args"])
    finally:
        sim_agents.PLATFORM_ODOMETRY.update(saved)
    ind = runner.run_independent(scenario, sim, params, seed).metrics
    dec = runner.run_decentralized(scenario, sim, params, seed).metrics
    cen = runner.run_centralized(scenario, sim, params, seed)
    row = {
        "preset": preset,
        "variant": variant,
        "seed": seed,
        "team_ate_dec_m": dec["ate_team_m"],
        "team_ate_cen_m": cen.metrics["ate_team_m"],
    }
    for i, name in dec["agents"].items():
        row[f"ate_alone_{name}_m"] = ind["ate_local_m"][i]
        row[f"ate_fused_{name}_m"] = dec["ate_fused_m"][i]
        row[f"ate_oracle_own_{name}_m"] = ate_rmse(cen.trajectories_team[i], sim.agents[i].gt)
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--presets", nargs="+", default=["fleet_transit_anchored"])
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(10)))
    ap.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=list(VARIANTS))
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--out", default="results/never_hurt.csv")
    args = ap.parse_args()
    rev = commit()
    jobs = [(p, s, v) for p in args.presets for v in args.variants for s in args.seeds]
    with ProcessPoolExecutor(args.jobs) as ex:
        rows = [{"commit": rev, **r} for r in ex.map(run_one, *zip(*jobs, strict=True))]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)
    for p in args.presets:
        for v in args.variants:
            rs = [r for r in rows if r["preset"] == p and r["variant"] == v]
            ch = [r["ate_fused_uuv_1_m"] / r["ate_alone_uuv_1_m"] - 1 for r in rs]
            orc = [r["ate_oracle_own_uuv_1_m"] / r["ate_alone_uuv_1_m"] - 1 for r in rs]
            print(
                f"{p} {v:6s} uuv_1 fused vs alone "
                + " ".join(f"{r['seed']}:{100 * c:+.0f}%" for r, c in zip(rs, ch, strict=True))
                + f" | worse {sum(c > 0 for c in ch)}"
                f" | oracle vs alone worse {sum(c > 0 for c in orc)}"
                f" | team dec {sum(r['team_ate_dec_m'] for r in rs) / len(rs):.3f}"
                f" cen {sum(r['team_ate_cen_m'] for r in rs) / len(rs):.3f}"
            )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
