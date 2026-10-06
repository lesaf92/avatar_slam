#!/usr/bin/env python3
"""What the accepted inter-agent alignments are made of (task T-F3-06, docs/LOG.md L41).

Runs the decentralized estimator on Tier-2 recordings and inspects the agents afterwards:

* ``anatomy``: every agent's final alignment to each neighbour, right or wrong (> 2 m or 5
  degrees), and its inlier pairs by ground truth: ``correct`` (same object), ``clutter``,
  ``wrong_object``; ``split`` if the object has more than one landmark at that agent. Also the
  landmarks received from the neighbour and the support of the true transform (own landmarks
  within 1 m, horizontally, of a transformed remote one) against that of the estimate.
* ``history``: every alignment attempt (``None`` = rejected) and whether it was adopted.
* ``seeds``: the final maps of every cross pair re-aligned with several ``clique_seeds`` values:
  right, wrong or rejected, and the time ``align`` takes.

Ground truth is read only to label; the estimator never sees it.

    python experiments/sonar_alignment_anatomy.py anatomy --tracking ekf --sonar sonar --jobs 28
"""

from __future__ import annotations

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import collections
import dataclasses
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

import avatar.agent as agent_module
import avatar.runner as runner
from avatar.agent import AvatarParams
from avatar.eval.metrics import frame_error
from avatar.frontend.association import align, candidate_pairs
from avatar.geometry import compose, inverse, transform_points
from avatar.tier2.dataset import build_tier2_sim
from avatar.tier2.frontend import FrontEndParams

RUNS = Path(__file__).resolve().parent.parent / "results" / "tier2"


def _wrong(err: tuple[float, float]) -> bool:
    return bool(err[0] > 2.0 or err[1] > np.deg2rad(5.0))


def _cross(a: str, b: str) -> bool:
    return a.startswith("uuv") != b.startswith("uuv")


def _run(seed: int, args: argparse.Namespace, history: list | None = None):
    """Run one seed; returns the scenario, the data and the agents as they ended."""
    base = AvatarParams()
    params = dataclasses.replace(
        base, association=dataclasses.replace(base.association, clique_seeds=args.clique_seeds)
    )
    sc, sim, _ = build_tier2_sim(
        Path(args.runs) / f"harbor_fleet_seed{seed}",
        params,
        FrontEndParams(tracking=args.tracking),
        sonar=None if args.sonar == "proxy" else args.sonar,
    )
    agents: dict = {}
    make, update, align_fn = (
        runner._make_agents,
        agent_module.AvatarAgent.update_alignments,
        agent_module.align,
    )
    calls: list = []

    def make_and_keep(*a, **k):
        agents.update(make(*a, **k))
        return agents

    def logged_align(*a, **k):
        res = align_fn(*a, **k)
        calls.append(res)
        return res

    def logged_update(self):  # one align() per dirty neighbour, in sorted order
        senders, before = sorted(self._dirty), dict(self.alignments)
        calls.clear()
        update(self)
        for s, res in zip(senders, list(calls), strict=False):
            history.append((self.k, self.id, s, res, self.alignments.get(s) is not before.get(s)))

    runner._make_agents = make_and_keep
    if history is not None:
        agent_module.align, agent_module.AvatarAgent.update_alignments = logged_align, logged_update
    try:
        runner.run_decentralized(sc, sim, params, seed)
    finally:
        runner._make_agents = make
        agent_module.align, agent_module.AvatarAgent.update_alignments = align_fn, update
    return sc, sim, agents


def _truth(sim, agents) -> dict:
    """Per agent: landmark id -> (true object or -1, purity), and object -> landmark ids."""
    obj = [p.object_id for p in sim.world.parts]
    out = {}
    for i, ag in agents.items():
        cnt: dict = collections.defaultdict(collections.Counter)
        for kf in sim.agents[i].keyframes:
            for d in kf.detections:
                cnt[d.part_index][
                    d.part_index if d.true_part_index is None else d.true_part_index
                ] += 1
        lids, objs = {}, collections.defaultdict(list)
        for pidx, lid in ag._lid_of_part.items():
            if cnt.get(pidx):
                top, n = cnt[pidx].most_common(1)[0]
                o = obj[top] if 0 <= top < len(obj) else -1
                lids[lid] = (o, n / sum(cnt[pidx].values()))
                if o >= 0:
                    objs[o].append(lid)
        out[i] = (lids, objs)
    return out


def _support(T, mine, remote) -> int:
    """Own landmarks within 1 m (horizontal) of a transformed remote landmark, one-to-one."""
    if not len(mine) or not len(remote):
        return 0
    q = transform_points(T, remote.positions)
    d = np.hypot(
        mine.positions[:, None, 0] - q[None, :, 0], mine.positions[:, None, 1] - q[None, :, 1]
    )
    used, n = set(), 0
    for a in np.argsort(d.min(axis=1)):
        free = [b for b in np.argsort(d[a]) if b not in used]
        if free and d[a, free[0]] < 1.0:
            used.add(int(free[0]))
            n += 1
    return n


def anatomy(job: tuple[int, argparse.Namespace]) -> list[dict]:
    seed, args = job
    sc, sim, agents = _run(seed, args)
    names = {a.agent_id: a.name for a in sc.agents}
    truth = _truth(sim, agents)
    gt = {i: a.T_world_from_local for i, a in sim.agents.items()}
    rows = []
    for i, ag in agents.items():
        mine, _ = ag._my_landmarks()
        mine_objs = {o for o, _ in truth[i][0].values() if o >= 0}
        for j, al in ag.alignments.items():
            if j not in truth:
                continue
            T_true = compose(inverse(gt[i]), gt[j])
            c: collections.Counter = collections.Counter()
            for my, rem, _ in ag.alignment_ids.get(j) or []:
                mo = truth[i][0].get(my, (None, 0.0))[0]
                ro = truth[j][0].get(rem, (None, 0.0))[0]
                kind = (
                    "unknown"
                    if mo is None or ro is None
                    else "clutter"
                    if -1 in (mo, ro)
                    else "wrong_object"
                    if mo != ro
                    else "correct"
                )
                c[kind] += 1
                c["split"] += bool(
                    (mo is not None and mo >= 0 and len(truth[i][1][mo]) > 1)
                    or (ro is not None and ro >= 0 and len(truth[j][1][ro]) > 1)
                )
            seen: set = set()
            for rlid in ag.inbox.get(j, {}):
                ro = truth[j][0].get(rlid, (None, 0.0))[0]
                key = (
                    "rx_unknown"
                    if ro is None
                    else "rx_clutter"
                    if ro < 0
                    else "rx_dup"
                    if ro in seen
                    else "rx_pile"
                )
                c[key] += 1
                if key == "rx_pile":
                    seen.add(ro)
                    c["rx_shared"] += ro in mine_objs
            remote, _ = ag._remote_landmarks(j)
            rows.append(
                dict(
                    seed=seed,
                    i=names[i],
                    j=names[j],
                    n=al.n_inliers,
                    wrong=_wrong(frame_error(al.T_mine_from_remote, T_true)),
                    used=j not in (ag.vetoed | ag.unconfirmed),
                    support_true=_support(T_true, mine, remote),
                    support_est=_support(al.T_mine_from_remote, mine, remote),
                    **c,
                )
            )
    return rows


def history(job: tuple[int, argparse.Namespace]) -> list[dict]:
    seed, args = job
    log: list = []
    sc, sim, _ = _run(seed, args, history=log)
    names = {a.agent_id: a.name for a in sc.agents}
    gt = {i: a.T_world_from_local for i, a in sim.agents.items()}
    return [
        dict(
            seed=seed,
            k=k,
            i=names[i],
            j=names[j],
            adopted=adopted,
            attempt=None if res is None else res.n_inliers,
            attempt_wrong=None
            if res is None
            else _wrong(frame_error(res.T_mine_from_remote, compose(inverse(gt[i]), gt[j]))),
        )
        for k, i, j, res, adopted in log
        if j in gt
    ]


def seeds(job: tuple[int, argparse.Namespace]) -> list[dict]:
    seed, args = job
    sc, sim, agents = _run(seed, args)
    names = {a.agent_id: a.name for a in sc.agents}
    gt = {i: a.T_world_from_local for i, a in sim.agents.items()}
    base = AvatarParams().association
    rows = []
    for i, ag in agents.items():
        mine, _ = ag._my_landmarks()
        for j in ag.inbox:
            if j not in agents or not _cross(names[i], names[j]):
                continue
            remote, _ = ag._remote_landmarks(j)
            n_cand = len(candidate_pairs(mine, remote, base)[0])
            row: dict = dict(seed=seed, i=names[i], j=names[j], n_candidates=n_cand)
            for s in args.compare_seeds:
                t0 = time.perf_counter()
                res = align(mine, remote, dataclasses.replace(base, clique_seeds=s))
                row[f"t{s}"] = time.perf_counter() - t0
                row[f"s{s}"] = (
                    "none"
                    if res is None
                    else (
                        "WRONG"
                        if _wrong(
                            frame_error(res.T_mine_from_remote, compose(inverse(gt[i]), gt[j]))
                        )
                        else "right"
                    )
                )
            rows.append(row)
    return rows


def report(mode: str, rows: list[dict], args: argparse.Namespace) -> None:
    if mode == "anatomy":
        groups = collections.defaultdict(list)
        for r in rows:
            k = (
                "uuv-uuv"
                if r["i"].startswith("uuv") and r["j"].startswith("uuv")
                else ("cross" if _cross(r["i"], r["j"]) else "above")
            )
            groups[(k, r["wrong"])].append(r)
        for (k, w), rs in sorted(groups.items()):
            tot = max(sum(r["n"] for r in rs), 1)
            share = " ".join(
                f"{c} {sum(r.get(c, 0) for r in rs) / tot:.0%}"
                for c in ("correct", "clutter", "wrong_object", "split")
            )
            med = {
                c: float(np.median([r.get(c, 0) for r in rs]))
                for c in ("n", "support_true", "rx_pile", "rx_shared", "rx_clutter")
            }
            print(
                f"{k:8s} {'WRONG' if w else 'right'} {len(rs):3d}: median inliers {med['n']:.0f}, "
                f"true-transform support {med['support_true']:.0f}, received piles "
                f"{med['rx_pile']:.0f} ({med['rx_shared']:.0f} shared) clutter "
                f"{med['rx_clutter']:.0f}; inlier pairs: {share}"
            )
    elif mode == "history":
        by = collections.defaultdict(list)
        for r in rows:
            by[(r["seed"], r["i"], r["j"])].append(r)
        c: collections.Counter = collections.Counter()
        ks = []
        for key, rs in by.items():
            ad = [r for r in rs if r["adopted"]]
            if not _cross(key[1], key[2]) or not ad:
                continue
            c["final wrong" if ad[-1]["attempt_wrong"] else "final right"] += 1
            if ad[-1]["attempt_wrong"]:
                ks.append(ad[-1]["k"])
                c["  ... a right alignment adopted before it"] += any(
                    not r["attempt_wrong"] for r in ad[:-1]
                )
        print(dict(c), "| keyframe the final wrong ones were adopted:", sorted(ks))
    else:
        n_cand = np.median([r["n_candidates"] for r in rows])
        print(f"candidate pairs per cross pair: median {n_cand:.0f}")
        for s in args.compare_seeds:
            c = collections.Counter(r[f"s{s}"] for r in rows)
            t = [r[f"t{s}"] for r in rows]
            print(
                f"clique_seeds {s:4d}: right {c['right']:3d} wrong {c['WRONG']:3d} rejected "
                f"{c['none']:3d}; align() median {np.median(t) * 1e3:.0f} ms"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mode", choices=["anatomy", "history", "seeds"])
    ap.add_argument("--runs", default=str(RUNS))
    ap.add_argument("--tracking", default="ekf")
    ap.add_argument("--sonar", default="sonar", help="sonar recording, or 'proxy'")
    ap.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    ap.add_argument("--clique-seeds", type=int, default=AvatarParams().association.clique_seeds)
    ap.add_argument("--compare-seeds", type=int, nargs="+", default=[40, 100, 200, 400, 1000])
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--out", default=None, help="JSON lines")
    args = ap.parse_args()
    fn = {"anatomy": anatomy, "history": history, "seeds": seeds}[args.mode]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        rows = [r for rs in ex.map(fn, [(s, args) for s in args.seeds]) for r in rs]
    if args.out:
        Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    report(args.mode, rows, args)


if __name__ == "__main__":
    main()
