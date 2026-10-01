#!/usr/bin/env python3
"""Add DAVE sonar images to a Tier-2 recording (task T-S2-05, ADR-0008, docs/LOG.md L34).

Takes a run directory made by ``record.py`` (``meta.json``: scenario, seed, duration) and writes
``sonar.npz`` next to its ``raw.npz``: for every rig with a DAVE sonar emulation
(``avatar.tier2.sdf.DAVE_SONARS``) one uint8 image per keyframe, ``<agent>/<sensor>`` with
shape ``(keyframes, range bins, beams)``, plus ``<agent>/<sensor>/range_m`` and
``.../azimuth_rad``. The world is the *acoustic* one (below the waterline only).

Run in the ``avatar-dave`` image (DAVE's sonar, CUDA); the driver uses the system Python::

    AVATAR_TIER2_IMAGE=avatar-dave docker/tier2.sh python experiments/gazebo/record_sonar.py \
        --run-dir results/tier2/harbor_fleet_seed0
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.tier2.sdf import (
    DAVE_SONARS,
    SONAR_DB_MAX,
    SONAR_DB_MIN,
    SONAR_FRAMES_PER_KEYFRAME,
    SONAR_RANGE_POOL,
    sensor_topic,
    sonar_rigs,
    sonar_world_sdf,
)

WORLD_NAME = "avatar_sonar"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--run-dir", required=True, help="recording made by record.py")
    ap.add_argument("--keyframes", type=int, default=0, help="only the first N keyframes (test)")
    ap.add_argument("--timeout", type=float, default=30.0, help="per-keyframe sonar timeout [s]")
    ap.add_argument("--name", default="sonar", help="output name: <run-dir>/<name>.npz")
    args = ap.parse_args()

    run = Path(args.run_dir).resolve()
    meta = json.loads((run / "meta.json").read_text())
    scenario, sim = make_sim(
        meta["scenario"],
        int(meta["seed"]),
        float(meta["duration_s"]),
        AvatarParams(),
        **meta["scenario_args"],
    )
    names = {a.agent_id: a.name for a in scenario.agents}
    gt = {names[i]: ad.gt for i, ad in sim.agents.items()}
    rigs = sonar_rigs(scenario)
    if not rigs:
        raise SystemExit("no rig has a DAVE sonar emulation")
    n_kf = len(next(iter(gt.values())))
    if args.keyframes:
        n_kf = min(n_kf, args.keyframes)
    sensors = [
        {
            "key": f"{n}/{s}",
            "topic": sensor_topic(n, s),
            "raw_bins": DAVE_SONARS[s].raw_range_bins,
            "beams": DAVE_SONARS[s].image_beams,
        }
        for n, ss in rigs.items()
        for s in ss
    ]
    plan = {
        "world": WORLD_NAME,
        "rigs": list(rigs),
        "sensors": sensors,
        "pool": SONAR_RANGE_POOL,
        "frames_per_keyframe": SONAR_FRAMES_PER_KEYFRAME,
        "db_min": SONAR_DB_MIN,
        "db_max": SONAR_DB_MAX,
        "timeout_s": args.timeout,
        "warmup_calls": 60,  # 120 ms of simulation time before keyframe 0
        "poses": [[[float(v) for v in gt[n][k]] for n in rigs] for k in range(n_kf)],
    }
    (run / f"{args.name}_world.sdf").write_text(
        sonar_world_sdf(scenario, {n: gt[n][0] for n in rigs}, WORLD_NAME)
    )
    (run / f"{args.name}_plan.json").write_text(json.dumps(plan))

    env = dict(os.environ)
    env.setdefault("GZ_IP", "127.0.0.1")
    # Containers on one docker network hear each other's ROS topics (DDS multicast) and the sonar
    # topics have the same names in every run: one ROS domain per run directory, on localhost.
    env["ROS_DOMAIN_ID"] = str(1 + zlib.crc32(str(run).encode()) % 100)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["HOME"] = env.get("HOME", "/tmp")
    log = open(run / f"{args.name}_gz.log", "w")
    gz = subprocess.Popen(
        ["gz", "sim", "-s", "--headless-rendering", "-v", "2", str(run / f"{args.name}_world.sdf")],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
        cwd=run,  # DAVE's sonar plugin creates debug_timings.txt in its working directory
    )  # fmt: skip
    t0 = time.time()
    try:
        drv = subprocess.run(
            [
                "/usr/bin/python3",
                str(HERE / "sonar_driver.py"),
                str(run / f"{args.name}_plan.json"),
                str(run / f"{args.name}.npz"),
            ],
            env=env,
        )
        if drv.returncode != 0:
            raise SystemExit(
                f"sonar_driver failed ({drv.returncode}); see {run}/{args.name}_gz.log"
            )
    finally:
        try:
            os.killpg(gz.pid, signal.SIGINT)
            gz.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(gz.pid, signal.SIGKILL)
        log.close()
    meta[args.name] = {
        "commit": commit(),
        "keyframes": n_kf,
        "sensors": {s: DAVE_SONARS[s].__dict__ for ss in rigs.values() for s in ss},
        "pool": SONAR_RANGE_POOL,
        "db_range": [SONAR_DB_MIN, SONAR_DB_MAX],
        "wall_time_s": time.time() - t0,
    }
    (run / "meta.json").write_text(json.dumps(meta, indent=1, default=float))
    wall = meta[args.name]["wall_time_s"]
    print(f"[record_sonar] wrote {run / (args.name + '.npz')} in {wall:.0f} s")


if __name__ == "__main__":
    main()
