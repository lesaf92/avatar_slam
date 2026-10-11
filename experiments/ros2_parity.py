#!/usr/bin/env python3
"""The team in ROS 2 against the offline pipeline, seed by seed (T-S3-02, ADR-0010, goal G-5).

For each seed, ``ros2 launch avatar_sim replay.launch.py`` runs the team (one process per node,
free-running clock; ``--live``: those agents' front-ends run live on the sensor frames, rendered
by Gazebo with ``--gazebo``, T-S3-03)
on a Tier-2 recording (``--runs``, ``--tracking``, ``--sonar``) or on a Tier-1
scenario (``--runs ''``); the anchor's estimate of every robot's frame gives gate G1 (frame error
below 1 m). The offline decentralized run on the same seed and detections gives the reference.
Time runs free, so the comparison is statistical: G1 counts and frame errors.

Needs ROS 2 Jazzy and a colcon install of ``ros2/`` (``--ws``). Parallel runs use separate ROS
domains on localhost.

    python experiments/ros2_parity.py --ws <colcon install> --runs results/tier2 --tracking ekf \\
        --seeds $(seq 0 19) --jobs 4 --out results/ros2_parity.csv
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import csv
import json
import queue
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from _provenance import commit

from avatar.agent import AvatarParams
from avatar.eval.metrics import frame_error
from avatar.geometry import compose, inverse
from avatar.runner import make_sim, run_decentralized

REPO = Path(__file__).resolve().parents[1]
G1_XY_M = 1.0


def _g1(frames: dict[int, np.ndarray], scenario, sim) -> tuple[bool, float]:
    """(gate G1 holds, largest frame error [m]) of an anchor's frame estimates."""
    slam = [a.agent_id for a in scenario.agents if a.role == "slam"]
    gt = {i: a.T_world_from_local for i, a in sim.agents.items()}
    a0 = scenario.anchor_id
    errs = [
        frame_error(frames[j], compose(inverse(gt[a0]), gt[j]))[0] if j in frames else np.inf
        for j in slam
        if j != a0
    ]
    return bool(max(errs) < G1_XY_M), float(max(errs))


def _sim(args, seed: int):
    """Scenario and SimData of a seed, with the offline front-end's detections for Tier 2."""
    if not args.runs:
        return make_sim("harbor_fleet", seed, args.duration, AvatarParams(), **args.scenario_args)
    from avatar.tier2.dataset import build_tier2_sim
    from avatar.tier2.frontend import FrontEndParams

    run = Path(args.runs) / f"harbor_fleet_seed{seed}"
    sc, sim, _ = build_tier2_sim(run, AvatarParams(), FrontEndParams(tracking=args.tracking),
                                 sonar=args.sonar or None)  # fmt: skip
    return sc, sim


def run_one(args, seed: int, domains: queue.Queue) -> dict:
    scenario, sim = _sim(args, seed)
    off = run_decentralized(scenario, sim, AvatarParams(), seed).metrics
    a0 = scenario.anchor_id
    slam = [a.agent_id for a in scenario.agents if a.role == "slam"]
    off_err = max(off["frame_error"].get(j, {"xy_m": np.inf})["xy_m"] for j in slam if j != a0)
    out = Path(args.work) / f"seed{seed}"
    launch = [
        "ros2", "launch", "avatar_sim", "replay.launch.py", f"rate:={args.rate}",
        f"out_dir:={out}", f"tracking:={args.tracking}",
    ]  # fmt: skip
    if args.sonar:  # ros2 launch refuses an empty value
        launch.append(f"sonar:={args.sonar}")
    if args.live:
        launch.append(f"live:={args.live}")
    if args.gazebo:
        launch.append("gazebo:=true")
    if args.runs:
        launch.append(f"run_dir:={Path(args.runs) / f'harbor_fleet_seed{seed}'}")
    else:
        launch += [f"seed:={seed}", f"duration_s:={args.duration}",
                   f"scenario_args:={json.dumps(args.scenario_args)}"]  # fmt: skip
    domain = domains.get()  # a ROS domain of its own, so parallel runs do not hear each other
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               PYTHONPATH=f"{REPO / 'avatar_py'}:{env.get('PYTHONPATH', '')}")  # fmt: skip
    cmd = f"source /opt/ros/jazzy/setup.bash && source {args.ws}/setup.bash && " + " ".join(
        f"'{c}'" for c in launch
    )
    log = Path(args.work) / f"seed{seed}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    try:
        with log.open("w") as f:
            rc = subprocess.run(["bash", "-c", cmd], env=env, stdout=f, stderr=subprocess.STDOUT,
                                timeout=args.timeout).returncode  # fmt: skip
    finally:
        domains.put(domain)
    anchor = out / f"{scenario.agent(a0).name}.json"
    if anchor.exists():
        est = json.loads(anchor.read_text())
        frames = {int(j): np.array(T) for j, T in est["team_frames"].items()}
        ros_g1, ros_err = _g1(frames, scenario, sim)
    else:
        ros_g1, ros_err = False, float("inf")
    return {
        "seed": seed,
        "offline_g1": bool(off_err < G1_XY_M),
        "offline_frame_err_max_m": off_err,
        "ros_g1": ros_g1,
        "ros_frame_err_max_m": ros_err,
        "launch_rc": rc,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--ws", required=True, help="colcon install directory with avatar_sim")
    ap.add_argument("--runs", default="", help="Tier-2 run directories ('' = Tier 1)")
    ap.add_argument("--tracking", default="oracle")
    ap.add_argument("--sonar", default="")
    ap.add_argument("--live", default="", help="agents with a live front-end, e.g. ugv_0,uav_0")
    ap.add_argument("--gazebo", action="store_true", help="their frames rendered live by Gazebo")
    ap.add_argument("--scenario-args", type=json.loads, default={}, help="Tier 1, as JSON")
    ap.add_argument("--duration", type=float, default=600.0, help="Tier 1")
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    ap.add_argument("--rate", type=float, default=5.0, help="simulated seconds per wall second")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--timeout", type=float, default=1800.0)
    ap.add_argument("--work", default="results/ros2_parity_work")
    ap.add_argument("--out", default="results/ros2_parity.csv")
    args = ap.parse_args()
    rev = commit()
    domains: queue.Queue = queue.Queue()
    for d in range(20, 20 + args.jobs):
        domains.put(d)
    with ThreadPoolExecutor(args.jobs) as ex:
        futs = [ex.submit(run_one, args, s, domains) for s in args.seeds]
        rows = [{"commit": rev, "tracking": args.tracking, "sonar": args.sonar, "live": args.live,
                 "gazebo": args.gazebo,
                 **f.result()} for f in futs]  # fmt: skip
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    both = sum(r["offline_g1"] and r["ros_g1"] for r in rows)
    print(f"G1 offline {sum(r['offline_g1'] for r in rows)}/{len(rows)}, "
          f"ROS 2 {sum(r['ros_g1'] for r in rows)}/{len(rows)}, both {both}; "
          f"launch failures {sum(r['launch_rc'] != 0 for r in rows)}; wrote {out}")  # fmt: skip


if __name__ == "__main__":
    main()
