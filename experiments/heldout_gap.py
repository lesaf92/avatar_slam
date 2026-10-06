#!/usr/bin/env python3
"""Do fresh seeds fail gate G1 more often than seeds 0-39? (task T-E3-02, docs/LOG.md L38-L39)

Reads the rows of ``alignment_acceptance_study.py`` (all runs at one commit, threshold 0) and
compares seed blocks of 20 for every perception: G1 passes, and the largest frame error per run
(a continuous measure, so a shift is visible before it crosses the 1 m gate). Seeds 0-39 against
40-79: Fisher's exact test on G1, Mann-Whitney U on the largest frame error.

    python experiments/heldout_gap.py results/heldout_gap_*.jsonl
"""

from __future__ import annotations

import collections
import json
import sys

import numpy as np
from scipy.stats import fisher_exact, mannwhitneyu


def max_err(r: dict) -> float:
    """Largest frame error of a run [m]; a robot outside the team frame counts as infinite."""
    return max(r["frame_err_m"].values()) if r["merged"] else float("inf")


def main(paths: list[str]) -> None:
    rows = [json.loads(line) for p in paths for line in open(p) if line.strip()]
    by_cfg = collections.defaultdict(list)
    for r in rows:
        by_cfg[r["config"]].append(r)
    for cfg, rs in by_cfg.items():
        print(f"== {cfg}")
        for lo in range(0, 80, 20):
            b = [r for r in rs if lo <= r["seed"] < lo + 20]
            if not b:
                continue
            e = np.array([max_err(r) for r in b])
            fails = [f"{r['seed']}:{max_err(r):.2f}" for r in b if not r["g1"]]
            print(
                f"  seeds {lo:2d}-{lo + 19}: G1 {sum(r['g1'] for r in b):2d}/{len(b)}"
                f"  max frame error median {np.median(e):.2f} m, p90 {np.quantile(e, 0.9):.2f} m"
                f"  fails {' '.join(fails)}"
            )
        old = [r for r in rs if r["seed"] < 40]
        new = [r for r in rs if r["seed"] >= 40]
        g = [[sum(r["g1"] for r in x), sum(not r["g1"] for r in x)] for x in (old, new)]
        u = mannwhitneyu([max_err(r) for r in old], [max_err(r) for r in new])
        print(
            f"  0-39 vs 40-79: G1 {g[0][0]}/{len(old)} vs {g[1][0]}/{len(new)},"
            f" Fisher p = {fisher_exact(g).pvalue:.3f}; max frame error Mann-Whitney"
            f" p = {u.pvalue:.3f}"
        )


if __name__ == "__main__":
    main(sys.argv[1:])
