"""Command-line interface: ``avatar run | compare | export-viz``.

Examples
--------
    avatar run --scenario harbor --mode decentralized --duration 300 --seed 0 --out r.json
    avatar compare --scenario harbor --duration 300 --seeds 0 1 2 --csv results/cmp.csv
    avatar export-viz --scenario harbor --duration 300 --seed 0 --out viz/data/harbor.json
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from avatar import __version__
from avatar.agent import AvatarParams
from avatar.runner import MODES, RunResult, make_sim, run
from avatar.semantics import CLASS_NAMES


def _git_commit() -> str:
    try:
        out = subprocess.run(
            ["git", "describe", "--always", "--dirty", "--abbrev=7"],
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    return obj


def _provenance(args) -> dict:
    return {"avatar_version": __version__, "commit": _git_commit(), "argv": sys.argv[1:]}


def _summary_row(res: RunResult) -> dict:
    m = res.metrics
    names = m["agents"]
    row = {"mode": res.mode, "seed": res.seed, "ate_team_m": m.get("ate_team_m", float("nan"))}
    key = "ate_fused_m" if "ate_fused_m" in m else "ate_local_m"
    for i, v in m[key].items():
        row[f"ate_{names[i]}_m"] = v
    for link, st in m.get("comm", {}).items():
        row[f"bytes_{link.lower()}"] = st["bytes_sent"]
    row["wall_time_s"] = m["wall_time_s"]
    return row


def _scenario_kwargs(args) -> dict:
    """Parse repeated ``--scenario-arg KEY=VALUE`` (int, float, bool or string)."""
    out: dict = {}
    for item in args.scenario_arg or []:
        key, sep, raw = item.partition("=")
        if not sep:
            raise SystemExit(f"--scenario-arg expects KEY=VALUE, got {item!r}")
        value: object = raw
        if raw.lower() in ("true", "false"):
            value = raw.lower() == "true"
        else:
            for cast in (int, float):
                try:
                    value = cast(raw)
                    break
                except ValueError:
                    continue
        out[key] = value
    return out


def cmd_run(args) -> int:
    res = run(args.scenario, args.mode, args.seed, args.duration, **_scenario_kwargs(args))
    out = {
        "provenance": _provenance(args),
        "mode": res.mode,
        "seed": res.seed,
        "metrics": _jsonable(res.metrics),
    }
    text = json.dumps(out, indent=2)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text)
    print(json.dumps(_jsonable(_summary_row(res)), indent=2))
    return 0


def cmd_compare(args) -> int:
    rows = []
    for seed in args.seeds:
        for mode in args.modes:
            res = run(args.scenario, mode, seed, args.duration, **_scenario_kwargs(args))
            rows.append(_summary_row(res))
            print(
                f"seed {seed} {mode:>13}: team ATE {rows[-1]['ate_team_m']:.3f} m "
                f"({rows[-1]['wall_time_s']:.1f} s)",
                file=sys.stderr,
            )
    cols = sorted({k for r in rows for k in r}, key=lambda c: (c not in ("mode", "seed"), c))
    print("| " + " | ".join(cols) + " |")
    print("|" + "---|" * len(cols))
    for r in rows:
        cells = [
            f"{r.get(c):.3f}" if isinstance(r.get(c), float) else str(r.get(c, "")) for c in cols
        ]
        print("| " + " | ".join(cells) + " |")
    if args.csv:
        Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
    return 0


def cmd_export_viz(args) -> int:
    params = AvatarParams()
    kwargs = _scenario_kwargs(args)
    scenario, sim = make_sim(args.scenario, args.seed, args.duration, params, **kwargs)
    dec = run(args.scenario, "decentralized", args.seed, args.duration, params, **kwargs)
    step = max(1, int(args.subsample))
    world = [
        {
            "id": s.object_id,
            "class": s.class_name,
            "center": list(s.center_xy),
            "z_min": s.z_min,
            "z_max": s.z_max,
            "footprint": list(s.footprint),
        }
        for s in scenario.world.structures
    ]
    agents = []
    for cfg in scenario.agents:
        ad = sim.agents[cfg.agent_id]
        est = dec.trajectories_team.get(cfg.agent_id)
        agents.append(
            {
                "id": cfg.agent_id,
                "name": cfg.name,
                "domain": cfg.domain.name,
                "t": ad.times[::step].tolist(),
                "gt": np.round(ad.gt[::step], 3).tolist(),
                "est": None if est is None else np.round(est[::step], 3).tolist(),
                "sensors": list(cfg.sensors),
                "comm": [c.name for c in cfg.comm],
            }
        )
    doc = {
        "provenance": _provenance(args),
        "scenario": args.scenario,
        "seed": args.seed,
        "duration_s": args.duration,
        "shoreline_x": scenario.world.shoreline_x,
        "land_z": scenario.world.land_z,
        "seabed_z": scenario.world.seabed_z,
        "bounds_xy": list(scenario.world.bounds_xy),
        "classes": list(CLASS_NAMES),
        "structures": world,
        "agents": agents,
        "comm_events": _jsonable(dec.comm_events),
        "metrics": _jsonable(dec.metrics),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(doc, separators=(",", ":")))
    print(f"wrote {args.out} (team ATE {dec.metrics['ate_team_m']:.3f} m)")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="avatar", description="Avatar SLAM reference tools")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--scenario", default="harbor", help="harbor | harbor_fleet")
        p.add_argument(
            "--scenario-arg",
            action="append",
            metavar="KEY=VALUE",
            help="scenario option, e.g. acoustic=x150, n_uuv=3, with_usv=true (repeatable)",
        )
        p.add_argument("--duration", type=float, default=300.0, help="mission length [s]")

    p = sub.add_parser("run", help="one mode, one seed")
    common(p)
    p.add_argument("--mode", choices=MODES, default="decentralized")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", help="write full metrics JSON here")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("compare", help="all modes over several seeds (markdown table)")
    common(p)
    p.add_argument("--seeds", type=int, nargs="+", default=[0])
    p.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    p.add_argument("--csv", help="also write a CSV")
    p.set_defaults(func=cmd_compare)

    p = sub.add_parser("export-viz", help="scenario + decentralized run for viz/")
    common(p)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--subsample", type=int, default=2)
    p.add_argument("--out", default="viz/data/harbor.json")
    p.set_defaults(func=cmd_export_viz)

    args = ap.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
