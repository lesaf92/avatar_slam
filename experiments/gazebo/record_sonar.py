#!/usr/bin/env python3
"""Add DAVE sonar images to a Tier-2 recording (task T-S2-05, ADR-0008, docs/LOG.md L34).

Takes a run directory made by ``record.py`` (``meta.json``: scenario, seed, duration) and writes
``sonar.npz`` next to its ``raw.npz``: for every rig with a DAVE sonar emulation
(``avatar.tier2.sdf.DAVE_SONARS``) one uint8 image per keyframe, ``<agent>/<sensor>`` with
shape ``(keyframes, range bins, beams)``, plus ``<agent>/<sensor>/range_m`` and
``.../azimuth_rad``. The world is the *acoustic* one (below the waterline only).

A sensor rendered as several fans (the Ping360, ``DaveSonar.fan_yaws_rad``) is stored as one
image of the stitched fans. ``--only`` renders some sensors only, and ``--base`` copies the other
sensors' images from an earlier recording of the run (T-S1-12: the Ping360 next to the Gemini).

Run in the ``avatar-dave`` image (DAVE's sonar, CUDA); the driver uses the system Python::

    AVATAR_TIER2_IMAGE=avatar-dave docker/tier2.sh python experiments/gazebo/record_sonar.py \
        --run-dir results/tier2/harbor_fleet_seed0
    AVATAR_TIER2_IMAGE=avatar-dave docker/tier2.sh python experiments/gazebo/record_sonar.py \
        --run-dir results/tier2_ping360/harbor_fleet_seed0 --only ping360 --base sonar \
        --name sonar360
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

import numpy as np

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
    sonar_models,
    sonar_rigs,
    sonar_world_sdf,
)
from avatar.tier2.sonar_image import stitch_fans

WORLD_NAME = "avatar_sonar"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--run-dir", required=True, help="recording made by record.py")
    ap.add_argument("--keyframes", type=int, default=0, help="only the first N keyframes (test)")
    ap.add_argument("--timeout", type=float, default=30.0, help="per-keyframe sonar timeout [s]")
    ap.add_argument("--name", default="sonar", help="output name: <run-dir>/<name>.npz")
    ap.add_argument("--warmup-calls", type=int, default=60,
                    help="2 ms steps before keyframe 0 (Ping360: 200)")  # fmt: skip
    ap.add_argument("--only", nargs="+", default=None, help="render only these sensors")
    ap.add_argument("--base", default=None, help="copy the other sensors from <run-dir>/<base>.npz")
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
    rigs = sonar_rigs(scenario, args.only)
    models = sonar_models(scenario, args.only)
    if not rigs:
        raise SystemExit("no rig has a DAVE sonar emulation")
    n_kf = len(next(iter(gt.values())))
    if args.keyframes:
        n_kf = min(n_kf, args.keyframes)
    sensors = [
        {
            "key": f"{n}/{fan}",
            "topic": sensor_topic(n, fan),
            "raw_bins": DAVE_SONARS[s].raw_range_bins,
            "beams": DAVE_SONARS[s].image_beams,
        }
        for n, ss in rigs.items()
        for s in ss
        for fan, _ in DAVE_SONARS[s].fans(s)
    ]
    out_path = run / f"{args.name}.npz"
    if out_path.exists():
        out_path.unlink()  # never write through a hard link to another run's recording
    plan = {
        "world": WORLD_NAME,
        "rigs": [m for m, *_ in models],
        "sensors": sensors,
        "pool": SONAR_RANGE_POOL,
        "frames_per_keyframe": SONAR_FRAMES_PER_KEYFRAME,
        "db_min": SONAR_DB_MIN,
        "db_max": SONAR_DB_MAX,
        "timeout_s": args.timeout,
        "warmup_calls": args.warmup_calls,  # simulation time before keyframe 0: 2 ms each
        "poses": [
            [
                [*(float(v) for v in gt[a][k][:3]), float(gt[a][k][3] + yaw)]
                for _, a, yaw, _ in models
            ]
            for k in range(n_kf)
        ],
    }
    (run / f"{args.name}_world.sdf").write_text(
        sonar_world_sdf(scenario, {n: gt[n][0] for n in rigs}, WORLD_NAME, args.only)
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
                str(out_path),
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
    _stitch_and_merge(out_path, rigs, run / f"{args.base}.npz" if args.base else None, n_kf)
    meta[args.name] = {
        "commit": commit(),
        "keyframes": n_kf,
        "sensors": {s: DAVE_SONARS[s].__dict__ for ss in rigs.values() for s in ss},
        "pool": SONAR_RANGE_POOL,
        "db_range": [SONAR_DB_MIN, SONAR_DB_MAX],
        "wall_time_s": time.time() - t0,
        "only": args.only,
        "base": args.base,
    }
    (run / "meta.json").write_text(json.dumps(meta, indent=1, default=float))
    wall = meta[args.name]["wall_time_s"]
    print(f"[record_sonar] wrote {run / (args.name + '.npz')} in {wall:.0f} s")


def _stitch_and_merge(out: Path, rigs: dict, base: Path | None, n_kf: int) -> None:
    """Join each multi-fan sensor's fans into one image; copy the other sensors from ``base``."""
    with np.load(out) as z:
        arrays = dict(z)
    for n, ss in rigs.items():
        for s in ss:
            fans = DAVE_SONARS[s].fans(s)
            if len(fans) == 1:
                continue
            keys = [f"{n}/{fan}" for fan, _ in fans]
            ranges = [arrays.pop(k + "/range_m") for k in keys]
            if any(not np.allclose(r, ranges[0]) for r in ranges):
                raise SystemExit(f"{n}/{s}: the fans have different range bins")
            img, az = stitch_fans(
                [arrays.pop(k) for k in keys],
                [arrays.pop(k + "/azimuth_rad") for k in keys],
                [y for _, y in fans],
            )
            arrays[f"{n}/{s}"], arrays[f"{n}/{s}/range_m"], arrays[f"{n}/{s}/azimuth_rad"] = (
                img,
                ranges[0],
                az,
            )
    if base is not None:
        with np.load(base) as z:
            for k in z.files:
                if k not in arrays and k != "warmup_calls":
                    arrays[k] = z[k][:n_kf] if z[k].ndim == 3 else z[k]
    np.savez_compressed(out, **arrays)


if __name__ == "__main__":
    main()
