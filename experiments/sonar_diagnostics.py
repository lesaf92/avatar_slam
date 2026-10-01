#!/usr/bin/env python3
"""Why the team does not merge on DAVE sonar images: the diagnostics of docs/LOG.md L34.

Every study runs the decentralized estimator on the sonar-image detections of the development
seeds (``tier2_study.py``, tier ``T2s``: ground-truth intra-agent ids) or on the proxy's, changes
one thing, and reports how many teams merge, gate G1 (frame error of every robot < 1 m) and the
share of wrong accepted alignments (> 2 m or 5 degrees):

* ``reasons``:   why the back end refuses alignments, for the proxy and the sonar;
* ``ambiguity``: the ambiguity ratio of the alignment (0.8 by default) relaxed on sonar data;
* ``noise``:     the proxy's detections with independent noise added to the BlueROV2s';
* ``snap``:      sonar detections moved to the true centre (+ 0.1 m) and / or without the
                 unmatched (spurious) ones;
* ``rules``:     the sonar detector without its rejection rules and / or with a lower threshold.

Ground truth is used only to build the controls, never by the estimator.

    python experiments/sonar_diagnostics.py ambiguity --runs results/tier2 --jobs 24
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import collections
import dataclasses
import json
import shutil
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import avatar.agent as agent_module
import avatar.frontend.association as association
from avatar.agent import AvatarParams
from avatar.geometry import inverse_transform_points, wrap_angle
from avatar.runner import run_decentralized
from avatar.tier2.dataset import build_tier2_sim
from avatar.tier2.frontend import FrontEndParams
from avatar.tier2.sonar_image import SonarImageParams

WRONG_XY_M, WRONG_YAW_RAD, G1_XY_M = 2.0, np.deg2rad(5.0), 1.0

_NO_RULES = dict(arc_db=1e9, max_footprint_m=1e9, contrast_db=-1e9, chain_min=10**6)
STUDIES: dict[str, list[str]] = {
    "reasons": ["proxy", "sonar"],
    "ambiguity": ["0.8", "0.9", "0.95"],
    "noise": ["0.1", "0.2"],
    "snap": ["snap", "drop_spurious", "snap_drop"],
    "rules": ["default", "norules", "norules_thr12", "thr12"],
}


def _private_copy(run: Path) -> Path:
    """A scratch directory that links the recordings, so that a variant has its own cache."""
    tmp = Path(tempfile.mkdtemp(prefix="sonar_diag_"))
    for name in ("meta.json", "raw.npz", "sonar.npz"):
        if (run / name).exists():
            (tmp / name).symlink_to((run / name).resolve())
    return tmp


def _outcome(scenario, sim, metrics) -> dict:
    al = [
        (e["xy_m"], e["yaw_rad"]) for d in metrics["alignment_errors"].values() for e in d.values()
    ]
    wrong = sum(1 for xy, yaw in al if xy > WRONG_XY_M or yaw > WRONG_YAW_RAD)
    ferr = [v["xy_m"] for v in metrics["frame_error"].values()]
    n_slam = sum(1 for a in scenario.agents if a.role == "slam")
    return {
        "merged": len(metrics["connected"]) == n_slam,
        "g1": bool(len(ferr) == n_slam - 1 and max(ferr) < G1_XY_M),
        "wrong": wrong,
        "alignments": len(al),
    }


def _count_reasons(mine, remote, params, log: list, orig):
    """``align`` that also records why an attempt was refused (a copy of its decision points)."""
    ia, ib, cross = association.candidate_pairs(mine, remote, params)
    res = orig(mine, remote, params)
    reason = "accepted"
    if res is None:
        if len(ia) < params.min_inliers:
            reason = "few candidates"
        else:
            gxy, gz = association._gates(mine, remote, ia, ib, cross, params)
            cons = association._consistency(mine, remote, ia, ib, cross, gxy, gz)
            hyps = []
            for clique in association._greedy_cliques(cons, params.clique_seeds):
                if len(clique) < 2:
                    continue
                T = association._refine(mine, remote, ia, ib, cross, clique, params)
                nxy, nz = association._normalized_residuals(T, mine, remote, ia, ib, cross, gxy, gz)
                sel, score = association._select_inliers(nxy, nz, ia, ib, cross)
                if len(sel) >= 2:
                    hyps.append((score, len(sel), T))
            if not hyps:
                reason = "no hypothesis"
            else:
                hyps.sort(key=lambda h: h[0], reverse=True)
                best_score, best_n, best_T = hyps[0]
                if best_n < params.min_inliers:
                    reason = "too few inliers"
                else:
                    reason = "too few after refinement"
                    for score, _, T in hyps[1:]:
                        far = np.hypot(T[0] - best_T[0], T[1] - best_T[1]) > params.distinct_xy_m
                        far |= abs(wrap_angle(T[3] - best_T[3])) > params.distinct_yaw_rad
                        if far:
                            if score >= params.ambiguity_ratio * best_score:
                                reason = "ambiguous"
                            break
    log.append((reason, 0 if res is None else res.n_inliers))
    return res


def run_variant(job: tuple[str, int, str, str]) -> dict:
    study, seed, variant, runs = job
    run = Path(runs) / f"harbor_fleet_seed{seed}"
    params = AvatarParams()
    use_sonar = not (study == "noise" or (study == "reasons" and variant == "proxy"))
    fe = FrontEndParams(tracking="oracle")
    if study == "rules":
        base = SonarImageParams()
        cfg = {
            "default": base,
            "norules": dataclasses.replace(base, **_NO_RULES),
            "norules_thr12": dataclasses.replace(base, **_NO_RULES, threshold_db=12.0),
            "thr12": dataclasses.replace(base, threshold_db=12.0),
        }[variant]
        fe = FrontEndParams(tracking="oracle", sonar_image=cfg)
    work = _private_copy(run)
    try:
        scenario, sim, _ = build_tier2_sim(work, params, fe, sonar="sonar" if use_sonar else None)
        names = {a.agent_id: a.name for a in scenario.agents}
        rng = np.random.default_rng(seed + 991)
        log: list = []
        if study in ("noise", "snap"):
            agents = dict(sim.agents)
            for aid, ad in sim.agents.items():
                if ad.config.role != "slam" or not names[aid].startswith("uuv"):
                    continue
                kfs = []
                for k, kf in enumerate(ad.keyframes):
                    dets = []
                    for d in kf.detections:
                        if study == "noise":
                            s = float(variant)
                            sg = d.sigmas.copy()
                            sg[:2] = np.hypot(sg[:2], s)
                            n = rng.normal(0.0, s, 2)
                            dets.append(
                                dataclasses.replace(
                                    d, p_body=d.p_body + np.array([n[0], n[1], 0.0]), sigmas=sg
                                )
                            )
                        elif d.true_part_index < 0:
                            if variant == "snap":
                                dets.append(d)  # spurious ones stay
                        elif variant in ("snap", "snap_drop"):
                            pb = inverse_transform_points(
                                ad.gt[k], scenario.world.parts[d.true_part_index].position
                            )
                            n = rng.normal(0.0, 0.1, 2)
                            pb = pb + np.array([n[0], n[1], 0.0])
                            pb[2] = d.p_body[2]
                            dets.append(dataclasses.replace(d, p_body=pb))
                        else:
                            dets.append(d)
                    kfs.append(dataclasses.replace(kf, detections=dets))
                agents[aid] = dataclasses.replace(ad, keyframes=kfs)
            sim = dataclasses.replace(sim, agents=agents)
        if study == "ambiguity":
            params = dataclasses.replace(
                params,
                association=dataclasses.replace(params.association, ambiguity_ratio=float(variant)),
            )
        if study == "reasons":
            orig = association.align
            agent_module.align = lambda a, b, p: _count_reasons(a, b, p, log, orig)
        try:
            metrics = run_decentralized(scenario, sim, params, seed).metrics
        finally:
            if study == "reasons":
                agent_module.align = association.align
        out = _outcome(scenario, sim, metrics)
        if study == "reasons":
            out["reasons"] = dict(collections.Counter(r for r, _ in log))
            out["inliers"] = [n for r, n in log if r == "accepted"]
        return {"study": study, "variant": variant, "seed": seed, **out}
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("study", choices=list(STUDIES))
    ap.add_argument(
        "--runs", default=str(Path(__file__).resolve().parent.parent / "results" / "tier2")
    )
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", default=None, help="JSON lines of every run")
    args = ap.parse_args()
    jobs = [(args.study, s, v, args.runs) for v in STUDIES[args.study] for s in args.seeds]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        rows = list(ex.map(run_variant, jobs))
    if args.out:
        Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    for v in STUDIES[args.study]:
        rs = [r for r in rows if r["variant"] == v]
        n = len(rs)
        head = (
            f"{args.study} {v:>14s}: merged {sum(r['merged'] for r in rs)}/{n}, "
            f"G1 {sum(r['g1'] for r in rs)}/{n}, wrong {sum(r['wrong'] for r in rs)}"
            f"/{sum(r['alignments'] for r in rs)}"
        )
        if args.study == "reasons":
            tot: collections.Counter = collections.Counter()
            for r in rs:
                tot.update(r["reasons"])
            calls = sum(tot.values())
            inl = [i for r in rs for i in r["inliers"]]
            head += f"; {calls} attempts: " + ", ".join(
                f"{k} {100 * c / calls:.0f} %" for k, c in tot.most_common()
            )
            head += f"; median inliers {np.median(inl):.0f}"
        print(head)


if __name__ == "__main__":
    main()
