#!/usr/bin/env python3
"""Cut a small Tier-2 fixture from a Gazebo recording and record its golden statistics.

The Tier-2 front-end normally needs a recording (``experiments/gazebo/record.py``,
Gazebo and a GPU), so CI cannot exercise it on real range data. This tool keeps the
first N keyframes of one recording (raw ranges, float16, compressed; well under 1 MB)
and writes the statistics the front-end produces on them for each tracking mode, so a
change to segmentation, gating or tracking shows up in ``pytest`` without Gazebo
(``avatar_py/tests/test_tier2_fixture.py``).

    python tools/gen_tier2_fixture.py results/tier2/harbor_fleet_seed0 --out testdata/tier2

Regenerate (and say why in the commit) whenever the front-end, the Tier-1 scenario or
the recorder changes on purpose. The fixture is only valid together with the scenario
code that made the recording: the ground-truth poses are regenerated from ``meta.json``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.tier2.dataset import build_tier2_sim
from avatar.tier2.frontend import FrontEndParams

MODES = ("oracle", "nn", "ekf")
KEYS = ("clusters", "matched", "spurious", "tracks", "wrong_track", "dropped_ambiguous")


def golden(fixture: Path) -> dict:
    """Front-end statistics per tracking mode and agent (plus detections per agent)."""
    out: dict = {}
    for mode in MODES:
        _, sim, stats = build_tier2_sim(
            fixture, AvatarParams(), FrontEndParams(tracking=mode), cache=False
        )
        names = {i: a.config.name for i, a in sim.agents.items()}
        out[mode] = {
            name: {
                **{k: int(stats[name].get(k, 0)) for k in KEYS},
                "detections": sum(len(kf.detections) for kf in sim.agents[i].keyframes),
            }
            for i, name in names.items()
            if name in stats
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("run_dir", help="recording directory (raw.npz, meta.json)")
    ap.add_argument("--keyframes", type=int, default=61)
    ap.add_argument("--out", default="testdata/tier2")
    args = ap.parse_args()
    src, dst = Path(args.run_dir), Path(args.out)
    meta = json.loads((src / "meta.json").read_text())
    raw = np.load(src / "raw.npz")
    n = args.keyframes
    dst.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(dst / "raw.npz", **{k: raw[k][:n] for k in raw.files})
    meta.update(duration_s=float(n - 1), n_keyframes=n, source=str(src), fixture=True)
    (dst / "meta.json").write_text(json.dumps(meta, indent=1))
    (dst / "golden.json").write_text(json.dumps(golden(dst), indent=1, sort_keys=True))
    size = sum(f.stat().st_size for f in dst.iterdir())
    print(f"{dst}: {n} keyframes, {size / 1024:.0f} KB, golden.json written")


if __name__ == "__main__":
    main()
