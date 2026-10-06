#!/usr/bin/env python3
"""Record Tier-2 (Gazebo Harmonic) range data for one scenario and seed.

The scenario is built exactly as in Tier 1 (same seed → same world, paths,
odometry). The script writes the SDF world and a ``ros_gz_bridge`` config,
starts ``gz sim`` headless (paused) and the C++ recorder ``gz_recorder``
(gz-transport only), which for every keyframe

1. moves each kinematic sensor rig to its ground-truth pose
   (``/world/<w>/set_pose``),
2. steps the world by two physics iterations (``/world/<w>/control``),
3. stores the next scan/depth image of every sensor.

Output: ``<out>/raw.npz`` (ranges as float16, one array per agent/sensor,
shape ``(n_keyframes, v, h)``; ``inf`` = no return), ``world.sdf``,
``bridge.yaml`` and ``meta.json`` (commit, seed, sensor specs, timing).

Needs ROS 2 Jazzy, ``ros_gz_bridge`` and Gazebo Harmonic in the environment
(``source ~/opt/gz_env.sh`` on the lab machine; see experiments/gazebo/README.md)::

    python experiments/gazebo/record.py --scenario harbor_fleet --seed 0 --duration 600 \
        --out results/tier2/harbor_fleet_seed0
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from _provenance import commit

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.tier2.sdf import (
    GZ_SENSORS,
    bridge_yaml,
    rig_sensors,
    sensor_topic,
    world_sdf,
)


def parse_kv(items: list[str]) -> dict:
    """``key=value`` scenario arguments (values parsed as YAML scalars)."""
    import yaml

    out: dict = {}
    for it in items:
        k, v = it.split("=", 1)
        out[k] = yaml.safe_load(v)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--scenario", default="harbor_fleet")
    ap.add_argument("--scenario-file", default=None, help="YAML preset (experiments/scenarios)")
    ap.add_argument("--scenario-arg", action="append", default=[])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=10.0, help="per-keyframe sensor timeout [s]")
    ap.add_argument("--keep-sdf-only", action="store_true", help="write world/bridge and exit")
    args = ap.parse_args()

    kwargs = parse_kv(args.scenario_arg)
    scenario_name = args.scenario
    if args.scenario_file:
        import yaml

        doc = yaml.safe_load(Path(args.scenario_file).read_text())
        scenario_name = doc["scenario"]
        kwargs = {**doc.get("args", {}), **kwargs}
    params = AvatarParams()
    scenario, sim = make_sim(scenario_name, args.seed, args.duration, params, **kwargs)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    world_name = "avatar_tier2"
    names = {a.agent_id: a.name for a in scenario.agents}
    gt = {names[i]: ad.gt for i, ad in sim.agents.items()}
    rigs = {n: s for n, s in rig_sensors(scenario).items() if s}
    (out / "world.sdf").write_text(world_sdf(scenario, {n: gt[n][0] for n in rigs}, world_name))
    (out / "bridge.yaml").write_text(bridge_yaml(scenario, world_name))
    times = next(iter(sim.agents.values())).times
    meta = {
        "commit": commit(),
        "scenario": scenario_name,
        "scenario_args": kwargs,
        "seed": args.seed,
        "duration_s": args.duration,
        "n_keyframes": len(times),
        "rigs": rigs,
        "sensors": {s: GZ_SENSORS[s].__dict__ for ss in rigs.values() for s in ss},
    }
    if args.keep_sdf_only:
        (out / "meta.json").write_text(json.dumps(meta, indent=1, default=float))
        return

    recorder = HERE / "build" / "gz_recorder"
    if not recorder.exists():
        raise SystemExit(f"{recorder} missing: run `make -C experiments/gazebo` first")
    keys = [(n, s) for n, ss in rigs.items() for s in ss]
    lines = [f"{world_name} {len(times)} {len(rigs)} {len(keys)}"]
    for n, s in keys:
        spec = GZ_SENSORS[s]
        kind = "depth" if spec.kind == "depth" else "scan"
        lines.append(f"{sensor_topic(n, s)} {kind} {spec.h_samples * spec.v_samples}")
    lines += list(rigs)
    for k in range(len(times)):
        for n in rigs:
            lines.append(" ".join(f"{float(v):.6f}" for v in gt[n][k]))
    (out / "plan.txt").write_text("\n".join(lines) + "\n")

    env = dict(os.environ)
    env.setdefault("GZ_IP", "127.0.0.1")
    log_gz = open(out / "gz.log", "w")
    gz = subprocess.Popen(
        ["gz", "sim", "-s", "--headless-rendering", "-v", "2", str(out / "world.sdf")],
        stdout=log_gz, stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )  # fmt: skip
    t_start = time.time()
    try:
        rec = subprocess.run(
            [str(recorder), str(out / "plan.txt"), str(out), str(args.timeout)], env=env
        )
        if rec.returncode != 0:
            raise SystemExit(f"gz_recorder failed ({rec.returncode}); see {out / 'gz.log'}")
    finally:
        try:
            os.killpg(gz.pid, signal.SIGINT)
            gz.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(gz.pid, signal.SIGKILL)
        log_gz.close()
    arrays = {}
    for i, (n, s) in enumerate(keys):
        spec = GZ_SENSORS[s]
        raw = np.fromfile(out / f"{i}.f32", dtype=np.float32)
        arrays[f"{n}/{s}"] = raw.reshape(len(times), spec.v_samples, spec.h_samples).astype(
            np.float16
        )
        (out / f"{i}.f32").unlink()
    np.savez_compressed(out / "raw.npz", **arrays)
    meta["wall_time_s"] = time.time() - t_start
    (out / "meta.json").write_text(json.dumps(meta, indent=1, default=float))
    print(f"[record] wrote {out / 'raw.npz'} in {meta['wall_time_s']:.0f} s")


if __name__ == "__main__":
    main()
