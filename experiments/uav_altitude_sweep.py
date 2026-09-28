#!/usr/bin/env python3
"""UAV altitude vs. its ability to join the team (D435i pitched 30° down, ≤ 6 m).

Flies the default stand-off rectangle at several fixed heights and records how
many structures the UAV sees, how many it shares with the UGV, when it first
aligns with anyone, and when the team connects.

    python experiments/uav_altitude_sweep.py --heights 3 4.5 6 7 --seeds 0 1 2 \\
        --out results/uav_altitude_sweep.csv
"""

from __future__ import annotations

import os

# Small sparse solves run 5-10x slower with multi-threaded BLAS, and much worse
# when several runs share the CPU (docs/LOG.md L10). Must precede the NumPy import.
for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim, run_decentralized


def run_one(job: tuple[float, int, float]) -> dict:
    z, seed, duration = job
    params = AvatarParams()
    sc, sim = make_sim("harbor_fleet", seed, duration, params, paths={"uav_0": {"z": z}})
    ids = {a.name: a.agent_id for a in sc.agents}
    seen = {
        name: Counter(d.part_index for kf in sim.agents[i].keyframes for d in kf.detections)
        for name, i in ids.items()
        if name in ("ugv_0", "uav_0")
    }
    uav = {p for p, n in seen["uav_0"].items() if n >= 3}
    ugv = {p for p, n in seen["ugv_0"].items() if n >= 3}
    m = run_decentralized(sc, sim, params, seed).metrics
    fa = m["first_alignment_s"]
    uav_id = ids["uav_0"]
    times = [t for t in fa[uav_id].values()] + [d[uav_id] for d in fa.values() if uav_id in d]
    return {
        "uav_z_m": z,
        "seed": seed,
        "uav_parts_seen": len(seen["uav_0"]),
        "uav_ugv_common_parts_3x": len(uav & ugv),
        "uav_first_alignment_s": min(times) if times else None,
        "team_connected_s": m["team_connected_s"],
        "team_ate_m": m["ate_team_m"],
        "n_connected": len(m["connected"]),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--heights", type=float, nargs="+", default=[3.0, 4.5, 6.0, 7.0])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="results/uav_altitude_sweep.csv")
    args = ap.parse_args()
    rev = commit()
    jobs = [(z, s, args.duration) for z in args.heights for s in args.seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = [r | {"commit": rev} for r in pool.map(run_one, jobs)]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(r)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
