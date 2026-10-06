#!/usr/bin/env python3
"""Geometry check of recorded Tier-2 data (ray conventions, pose timing).

Projects every valid return to the world with the ground-truth rig pose and
measures its distance to the nearest surface of the scene primitives that
``avatar.tier2.sdf`` wrote (structures, land, seabed). If the ray ordering,
the depth-image convention, the mount pitch or the pose/scan timing were
wrong, most returns would lie far from any surface.

    python experiments/gazebo/check_geometry.py results/tier2/harbor_fleet_seed0
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.tier2.rays import body_to_world, sensor_points
from avatar.tier2.sdf import GZ_SENSORS, ROUND_CLASSES


def surface_distance(p: np.ndarray, world) -> np.ndarray:
    """Unsigned distance [m] from points ``(n, 3)`` to the nearest primitive surface."""
    best = np.full(len(p), np.inf)

    def box(center, half):
        q = np.abs(p - center) - half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(np.max(q, axis=1), 0.0)
        return np.abs(outside + inside)

    def cyl(cx, cy, r, z0, z1):
        dr = np.hypot(p[:, 0] - cx, p[:, 1] - cy) - r
        dz = np.maximum(z0 - p[:, 2], p[:, 2] - z1)
        outside = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0))
        inside = np.minimum(np.maximum(dr, dz), 0.0)
        return np.abs(outside + inside)

    for s in world.structures:
        if s.class_name in ROUND_CLASSES:
            d = cyl(s.center_xy[0], s.center_xy[1], 0.5 * s.footprint[0], s.z_min, s.z_max)
        else:
            c = np.array([s.center_xy[0], s.center_xy[1], 0.5 * (s.z_min + s.z_max)])
            h = np.array([0.5 * s.footprint[0], 0.5 * s.footprint[1], 0.5 * (s.z_max - s.z_min)])
            d = box(c, h)
        best = np.minimum(best, d)
    x0, x1, y0, y1 = world.bounds_xy
    lw, lh = world.shoreline_x - x0, world.land_z - world.seabed_z
    best = np.minimum(
        best,
        box(
            np.array([x0 + lw / 2, (y0 + y1) / 2, world.seabed_z + lh / 2]),
            np.array([lw / 2, (y1 - y0) / 2, lh / 2]),
        ),
    )
    sw = x1 - world.shoreline_x
    best = np.minimum(
        best,
        box(
            np.array([world.shoreline_x + sw / 2, (y0 + y1) / 2, world.seabed_z - 0.5]),
            np.array([sw / 2, (y1 - y0) / 2, 0.5]),
        ),
    )
    return best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir")
    ap.add_argument("--every", type=int, default=10, help="check every n-th keyframe")
    args = ap.parse_args()
    run = Path(args.run_dir)
    meta = json.loads((run / "meta.json").read_text())
    scenario, sim = make_sim(
        meta["scenario"], meta["seed"], meta["duration_s"], AvatarParams(), **meta["scenario_args"]
    )
    gt = {a.config.name: a.gt for a in sim.agents.values()}
    raw = np.load(run / "raw.npz")
    print(f"{'sensor':28s} {'returns':>9s} {'median [m]':>11s} {'p95 [m]':>8s} {'>0.2 m':>7s}")
    for key in raw.files:
        name, sensor = key.split("/")
        spec = GZ_SENSORS[sensor]
        dists = []
        for k in range(0, raw[key].shape[0], args.every):
            pb, _ = sensor_points(spec, raw[key][k])
            if len(pb):
                dists.append(surface_distance(body_to_world(gt[name][k], pb), scenario.world))
        d = np.concatenate(dists) if dists else np.zeros(0)
        if len(d) == 0:
            print(f"{key:28s} {0:9d}")
            continue
        print(
            f"{key:28s} {len(d):9d} {np.median(d):11.4f} {np.percentile(d, 95):8.4f} "
            f"{np.mean(d > 0.2):7.1%}"
        )


if __name__ == "__main__":
    main()
