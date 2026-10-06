#!/usr/bin/env python3
"""The EKF tracker on DAVE's sonar images: what goes wrong (task T-F3-06, docs/LOG.md L36).

Studies on the development seeds (recordings with ``sonar.npz``, ``make tier2-record-sonar``);
each runs the decentralized estimator and reports merges, gate G1 and wrong accepted alignments
(> 2 m or 5 degrees) by pair type (``cross``: a BlueROV2 with the UGV or the UAV; ``uuv-uuv``;
``above``: UGV with UAV):

* ``alignments``: proxy and sonar, ground-truth ids and EKF tracker, with alignments by inliers;
* ``clutter``:    the BlueROV2s' unmatched detections removed after tracking (a control: ground
                  truth used to remove them), with the EKF and with ground-truth ids;
* ``minlen``:     the BlueROV2s' tracks shorter than N detections removed (control);
* ``tracks``:     quality of the EKF's tracks on sonar (no estimator run): pile tracks against
                  the truth, and what the clutter tracks are;
* ``birth``:      the EKF's birth test for the sonar agents (``FrontEndParams.ekf_sonar``);
* ``confirm``:    an alignment adopted only after 2 or 3 agreeing attempts (``align_confirm``).

    python experiments/sonar_tracker_study.py alignments --jobs 28
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import collections
import dataclasses
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.frontend.ekf_tracker import EkfTrackerParams
from avatar.runner import run_decentralized
from avatar.tier2.dataset import build_tier2_sim
from avatar.tier2.frontend import FrontEndParams

RUNS = Path(__file__).resolve().parent.parent / "results" / "tier2"
BIRTH = {
    "default": None,
    "confirm8": {"confirm_frames": 8},
    "confirm12": {"confirm_frames": 12},
    "chi2_6": {"static_chi2": 5.99},
    "confirm8_chi2_6": {"confirm_frames": 8, "static_chi2": 5.99},
    "floor0": {"static_floor_m": 0.0},
    "confirm8_chi2_6_floor0": {"confirm_frames": 8, "static_chi2": 5.99, "static_floor_m": 0.0},
}
# (tracking, sonar recording or None for the proxy, variant)
STUDIES: dict[str, list[tuple[str, str | None, str]]] = {
    "alignments": [(t, s, "") for s in (None, "sonar") for t in ("oracle", "ekf")],
    "clutter": [("ekf", "sonar", "no_clutter"), ("oracle", "sonar", "no_clutter")],
    "minlen": [("ekf", "sonar", f"minlen:{n}") for n in (8, 12, 16, 24)],
    "birth": [("ekf", "sonar", f"birth:{k}") for k in BIRTH],
    "confirm": [
        (t, s, f"confirm:{c}") for s in (None, "sonar") for t in ("oracle", "ekf") for c in (2, 3)
    ],
}


def _kind(a: str, b: str) -> str:
    if a.startswith("uuv") and b.startswith("uuv"):
        return "uuv-uuv"
    return "above" if "uuv" not in a + b else "cross"


def run_one(job: tuple[int, str, str | None, str, str]) -> dict:
    seed, tracking, sonar, variant, runs = job
    params = AvatarParams()
    fe = FrontEndParams(tracking=tracking)
    if variant.startswith("birth:"):
        ov = BIRTH[variant.split(":")[1]]
        fe = FrontEndParams(
            tracking=tracking, ekf_sonar=None if ov is None else EkfTrackerParams(**ov)
        )
    if variant.startswith("confirm:"):
        params = dataclasses.replace(params, align_confirm=int(variant.split(":")[1]))
    # caches are keyed by the front-end parameters; birth variants rebuild without caching
    scenario, sim, _ = build_tier2_sim(
        Path(runs) / f"harbor_fleet_seed{seed}", AvatarParams(), fe, sonar=sonar,
        cache=not variant.startswith("birth:"),
    )  # fmt: skip
    names = {a.agent_id: a.name for a in scenario.agents}
    clutter = piles = 0
    for aid, ad in sim.agents.items():
        if not names[aid].startswith("uuv"):
            continue
        parts: dict = collections.defaultdict(collections.Counter)
        for kf in ad.keyframes:
            for d in kf.detections:
                parts[d.part_index][d.true_part_index] += 1
        for c in parts.values():
            if sum(c.values()) >= 4:
                clutter += c.most_common(1)[0][0] < 0
                piles += c.most_common(1)[0][0] >= 0
    lost = collections.Counter()
    if variant == "no_clutter" or variant.startswith("minlen:"):
        n_min = int(variant.split(":")[1]) if variant.startswith("minlen:") else 0
        agents = dict(sim.agents)
        for aid, ad in sim.agents.items():
            if not names[aid].startswith("uuv"):
                continue
            count = collections.Counter(d.part_index for kf in ad.keyframes for d in kf.detections)
            kfs = []
            for kf in ad.keyframes:
                keep = []
                for d in kf.detections:
                    ok = d.true_part_index >= 0 if n_min == 0 else count[d.part_index] >= n_min
                    lost[("pile" if d.true_part_index >= 0 else "clutter", ok)] += 1
                    if ok:
                        keep.append(d)
                kfs.append(dataclasses.replace(kf, detections=keep))
            agents[aid] = dataclasses.replace(ad, keyframes=kfs)
        sim = dataclasses.replace(sim, agents=agents)
    m = run_decentralized(scenario, sim, params, seed).metrics
    aligns = [
        (
            _kind(names[i], names[j]),
            int(m["alignments"][i][j]),
            bool(e["xy_m"] > 2.0 or e["yaw_rad"] > np.deg2rad(5.0)),
        )
        for i, d in m["alignment_errors"].items()
        for j, e in d.items()
    ]
    ferr = [v["xy_m"] for v in m["frame_error"].values()]
    n_slam = sum(1 for a in scenario.agents if a.role == "slam")
    return {
        "seed": seed, "tracking": tracking, "sonar": sonar or "proxy", "variant": variant,
        "merged": len(m["connected"]) == n_slam,
        "g1": bool(len(ferr) == n_slam - 1 and max(ferr) < 1.0),
        "alignments": aligns, "clutter_tracks": clutter, "pile_tracks": piles,
        "lost": {f"{k[0]}_{'kept' if k[1] else 'lost'}": v for k, v in lost.items()},
    }  # fmt: skip


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("study", choices=[*STUDIES, "tracks"])
    ap.add_argument("--runs", default=str(RUNS))
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", default=None, help="JSON lines of every run")
    args = ap.parse_args()
    if args.study == "tracks":
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            rows = [
                r
                for rs in ex.map(track_quality, [(sd, args.runs) for sd in args.seeds])
                for r in rs
            ]
        if args.out:
            Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
        report_tracks(rows)
        return
    cfgs = STUDIES[args.study]
    # fill the detection caches of the cached configurations first, one process per file
    cached = sorted({(t, s) for t, s, v in cfgs if not v.startswith("birth:")}, key=str)
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        list(
            ex.map(
                lambda_free_build,
                [(sd, t, s, args.runs) for t, s in cached for sd in args.seeds],
            )
        )
        rows = list(
            ex.map(run_one, [(sd, t, s, v, args.runs) for t, s, v in cfgs for sd in args.seeds])
        )
    if args.out:
        Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    for t, s, v in cfgs:
        rs = [r for r in rows if (r["tracking"], r["sonar"], r["variant"]) == (t, s or "proxy", v)]
        line = (
            f"{t:6s} {s or 'proxy':5s} {v:28s} merged {sum(r['merged'] for r in rs):2d}/{len(rs)} "
            f"G1 {sum(r['g1'] for r in rs):2d}/{len(rs)}"
        )
        for kind in ("cross", "uuv-uuv", "above"):
            al = [a for r in rs for a in r["alignments"] if a[0] == kind]
            line += f"  {kind} wrong {sum(a[2] for a in al)}/{len(al)}"
        if s == "sonar":
            line += (
                f"  clutter/pile tracks per run {np.mean([r['clutter_tracks'] for r in rs]):.0f}"
                f"/{np.mean([r['pile_tracks'] for r in rs]):.0f}"
            )
        print(line)
        if any(r["lost"] for r in rs):
            tot = collections.Counter()
            for r in rs:
                tot.update(r["lost"])
            for what in ("pile", "clutter"):
                n = tot[f"{what}_kept"] + tot[f"{what}_lost"]
                line += f"; {what} detections removed {100 * tot[f'{what}_lost'] / max(n, 1):.0f} %"
            print("    " + line.split("; ", 1)[1])
        if args.study == "alignments":
            for kind in ("cross", "uuv-uuv"):
                bins: dict = collections.defaultdict(lambda: [0, 0])
                for r in rs:
                    for k, n, w in r["alignments"]:
                        if k == kind:
                            b = (
                                "4-7"
                                if n < 8
                                else "8-9"
                                if n < 10
                                else "10-12"
                                if n < 13
                                else "13+"
                            )
                            bins[b][0] += 1
                            bins[b][1] += w
                print(
                    f"    {kind} by inliers (accepted/wrong): "
                    + "  ".join(f"{b}: {x[0]}/{x[1]}" for b, x in sorted(bins.items()))
                )


def track_quality(job: tuple[int, str]) -> list[dict]:
    """Every EKF track of the BlueROV2s on sonar with at least four detections, and its truth."""
    from avatar.geometry import transform_points

    seed, runs = job
    scenario, sim, _ = build_tier2_sim(
        Path(runs) / f"harbor_fleet_seed{seed}", AvatarParams(), FrontEndParams(tracking="ekf"),
        sonar="sonar",
    )  # fmt: skip
    w = scenario.world
    rows = []
    for ad in sim.agents.values():
        if not ad.config.name.startswith("uuv"):
            continue
        pts: dict = collections.defaultdict(list)
        parts: dict = collections.defaultdict(collections.Counter)
        for k, kf in enumerate(ad.keyframes):
            for d in kf.detections:
                pts[d.part_index].append(transform_points(ad.gt[k], d.p_body)[:2])
                parts[d.part_index][d.true_part_index] += 1
        for t, ps in pts.items():
            if len(ps) < 4:
                continue
            ps = np.array(ps)
            maj, cnt = parts[t].most_common(1)[0]
            ev = np.sort(np.linalg.eigvalsh(np.cov(ps.T)))[::-1]
            row = {
                "seed": seed, "agent": ad.config.name, "n": len(ps), "pile": bool(maj >= 0),
                "part": int(maj), "impurity": 1.0 - cnt / sum(parts[t].values()),
                "spread_m": float(np.sqrt(ev.sum())),
                "elongation": float(np.sqrt(ev[0] / max(ev[1], 1e-9))),
            }  # fmt: skip
            c = ps.mean(axis=0)
            if maj >= 0:
                row["centre_error_m"] = float(np.hypot(*(c - w.parts[maj].position[:2])))
            else:
                best, dmin = "nothing within 1 m", 1.0
                for st in w.structures:
                    if st.z_min >= 0:
                        continue
                    dx = max(abs(c[0] - st.center_xy[0]) - st.footprint[0] / 2, 0.0)
                    dy = max(abs(c[1] - st.center_xy[1]) - st.footprint[1] / 2, 0.0)
                    if np.hypot(dx, dy) < dmin:
                        best, dmin = st.class_name, float(np.hypot(dx, dy))
                if abs(c[0] - w.shoreline_x) < min(1.0, dmin):
                    best = "quay face"
                row["near"] = best
            rows.append(row)
    return rows


def report_tracks(rows: list[dict]) -> None:
    pile = [r for r in rows if r["pile"]]
    clut = [r for r in rows if not r["pile"]]
    runs = {(r["seed"], r["agent"]) for r in rows}
    print(f"{len(rows)} tracks with >= 4 detections over {len(runs)} BlueROV2-runs")
    per = collections.Counter((r["seed"], r["agent"], r["part"]) for r in pile)
    print(
        f"pile tracks {len(pile)} ({len(pile) / len(runs):.1f} per BlueROV2-run); parts with"
        f" more than one track {sum(1 for v in per.values() if v > 1)}; impure by > 10 % "
        f"{sum(r['impurity'] > 0.1 for r in pile)}; centre error median "
        f"{np.median([r['centre_error_m'] for r in pile]):.2f} m"
    )
    for name, rs in (("pile", pile), ("clutter", clut)):
        print(
            f"{name:7s} tracks: detections median {np.median([r['n'] for r in rs]):.0f},"
            f" spread median {np.median([r['spread_m'] for r in rs]):.2f} m, elongation median "
            f"{np.median([r['elongation'] for r in rs]):.1f}"
        )
    near = collections.Counter(r["near"] for r in clut)
    print(
        f"clutter tracks {len(clut)} ({len(clut) / len(runs):.1f} per BlueROV2-run), near: "
        + ", ".join(f"{k} {100 * v / len(clut):.0f} %" for k, v in near.most_common())
    )


def lambda_free_build(job: tuple[int, str, str | None, str]) -> None:
    seed, tracking, sonar, runs = job
    build_tier2_sim(
        Path(runs) / f"harbor_fleet_seed{seed}",
        AvatarParams(),
        FrontEndParams(tracking=tracking),
        sonar=sonar,
    )


if __name__ == "__main__":
    main()
