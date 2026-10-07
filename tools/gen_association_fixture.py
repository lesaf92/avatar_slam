#!/usr/bin/env python3
"""Regenerate ``testdata/association/sonar_seed18_uuv0_uav0.npz`` (LOG L41).

The final landmark sets of BlueROV2 0 and of what it received from the Tarot, in Tier-2 seed 18
with DAVE's sonar and the EKF tracker, run with ``clique_seeds = 40``: a dense candidate graph in
which 40 clique seeds grow only a wrong hypothesis and 200 find the true one. Also stores the true
``T_mine_from_remote``. The test (``test_association.py``) needs no recording.

    python tools/gen_association_fixture.py results/tier2/harbor_fleet_seed18
"""

from __future__ import annotations

import argparse
import dataclasses
from pathlib import Path

import numpy as np

import avatar.runner as runner
from avatar.agent import AvatarParams
from avatar.geometry import compose, inverse
from avatar.tier2.dataset import build_tier2_sim, load_meta
from avatar.tier2.frontend import FrontEndParams

FIELDS = ("ids", "positions", "sigma_xy", "sigma_z", "extents", "class_ids", "flags", "descriptors")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--out", default="testdata/association")
    args = ap.parse_args()
    base = AvatarParams()
    params = dataclasses.replace(
        base, association=dataclasses.replace(base.association, clique_seeds=40)
    )
    sc, sim, _ = build_tier2_sim(
        args.run_dir, params, FrontEndParams(tracking="ekf"), sonar="sonar"
    )
    agents: dict = {}
    make = runner._make_agents
    runner._make_agents = lambda *a, **k: agents.update(make(*a, **k)) or agents
    try:
        runner.run_decentralized(sc, sim, params, int(load_meta(args.run_dir)["seed"]))
    finally:
        runner._make_agents = make
    ids = {a.name: a.agent_id for a in sc.agents}
    me, other = ids["uuv_0"], ids["uav_0"]
    mine, _ = agents[me]._my_landmarks()
    remote, _ = agents[me]._remote_landmarks(other)
    T = compose(inverse(sim.agents[me].T_world_from_local), sim.agents[other].T_world_from_local)
    arrays = {f"mine_{f}": getattr(mine, f) for f in FIELDS}
    arrays |= {f"remote_{f}": getattr(remote, f) for f in FIELDS}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "sonar_seed18_uuv0_uav0.npz", T_true=T, **arrays)
    print(f"wrote {out / 'sonar_seed18_uuv0_uav0.npz'}: {len(mine)} own, {len(remote)} remote")


if __name__ == "__main__":
    main()
