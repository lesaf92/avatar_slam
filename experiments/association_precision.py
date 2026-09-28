#!/usr/bin/env python3
"""Association precision of one-shot all-to-all alignment (diagnostic for task X1/X2).

Every agent maps for ``--duration`` seconds alone, then receives every other
agent's full digest (unlimited links, *including* pairs that could not talk
physically, e.g. UAV↔AUV) and aligns. We report, per seed, how many
alignments were accepted, how many pairs they contain, and the fraction of
pairs that link the same ground-truth structure.

    python experiments/association_precision.py --seeds 0 1 2 3 4 --duration 90
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from avatar.agent import AvatarAgent, AvatarParams
from avatar.runner import make_sim
from avatar.types import LinkType


def evaluate(seed: int, duration_s: float, params: AvatarParams) -> dict:
    scenario, data = make_sim("harbor", seed, duration_s, params)
    obj_ids = np.array([p.object_id for p in data.world.parts])
    agents: dict[int, AvatarAgent] = {}
    for cfg in scenario.agents:
        ag = AvatarAgent(
            cfg,
            params,
            obj_ids,
            float(data.agents[cfg.agent_id].gt[0, 2]),
            np.random.default_rng(seed * 100 + cfg.agent_id),
        )
        for kf in data.agents[cfg.agent_id].keyframes:
            ag.on_keyframe(kf)
        ag.solve_local()
        agents[cfg.agent_id] = ag
    # Digests are broadcasts: build each sender's packets once, deliver to everyone.
    packets = {
        i: ag.build_digests(0.0, LinkType.RF, 10**7, 1400) for i, ag in agents.items()
    }
    for rx in agents.values():
        for tx, pkts in packets.items():
            if tx != rx.id:
                for pkt in pkts:
                    rx.on_packet(pkt)
        rx.update_alignments()
    inv = {
        a: {lid: p for p, lid in ag._lid_of_part.items()} for a, ag in agents.items()
    }
    parts = data.world.parts
    out = {"seed": seed, "links": {}}
    n_all = n_ok = 0
    for rx, ag in agents.items():
        for tx, pairs in ag.alignment_ids.items():
            ok = sum(
                parts[inv[rx][a]].object_id == parts[inv[tx][b]].object_id
                for a, b, _ in pairs
            )
            n_all += len(pairs)
            n_ok += ok
            out["links"][f"{rx}<-{tx}"] = {"pairs": len(pairs), "correct": ok}
    out["accepted_alignments"] = sum(len(ag.alignment_ids) for ag in agents.values())
    out["possible_alignments"] = len(agents) * (len(agents) - 1)
    out["pair_precision"] = n_ok / n_all if n_all else float("nan")
    out["wrong_alignments"] = sum(
        1 for v in out["links"].values() if v["correct"] < v["pairs"] / 2
    )
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--duration", type=float, default=90.0)
    ap.add_argument("--json", action="store_true", help="print full JSON per seed")
    args = ap.parse_args()
    params = AvatarParams()
    rows = [evaluate(s, args.duration, params) for s in args.seeds]
    for r in rows:
        if args.json:
            print(json.dumps(r))
        print(
            f"seed {r['seed']}: accepted {r['accepted_alignments']}/{r['possible_alignments']}"
            f"  wrong {r['wrong_alignments']}  pair precision {r['pair_precision']:.3f}"
        )
    prec = [r["pair_precision"] for r in rows]
    print(
        f"mean pair precision {np.nanmean(prec):.3f}; "
        f"total wrong alignments {sum(r['wrong_alignments'] for r in rows)}"
    )


if __name__ == "__main__":
    main()
