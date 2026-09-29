"""Assemble a Tier-2 :class:`~avatar.sim.measurements.SimData` from recorded Gazebo data.

The scenario and the Tier-1 measurements are regenerated from the recorded
``meta.json`` (scenario, arguments, seed, duration). Odometry, absolute
height, ground truth and every random stream of Tier 1 are therefore
identical; only the detections are replaced by those of the Tier-2 front-end
(:mod:`avatar.tier2.frontend`). A Tier-1 and a Tier-2 run with the same seed
thus form a **paired** comparison in which only perception differs (the
ADR-0003 parity check).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.sim.measurements import SimData
from avatar.sim.scenarios import Scenario
from avatar.tier2 import frontend as frontend_module
from avatar.tier2.frontend import FrontEndParams, detections_for_agent
from avatar.tier2.sdf import GZ_SENSORS

TRACK_ID_STRIDE = 1_000_000


def load_meta(run_dir: str | Path) -> dict:
    """Recording metadata (``meta.json``) of a Tier-2 run directory."""
    return json.loads((Path(run_dir) / "meta.json").read_text())


def build_tier2_sim(
    run_dir: str | Path,
    params: AvatarParams | None = None,
    fe_params: FrontEndParams | None = None,
    cache: bool = True,
) -> tuple[Scenario, SimData, dict]:
    """Scenario, Tier-2 SimData and front-end statistics for one recorded run.

    Front-end output is cached in ``<run_dir>/detections_<tracking>.pkl`` (keyed
    by the front-end parameters and source) because segmentation takes a while.
    """
    run = Path(run_dir)
    meta = load_meta(run)
    params = params or AvatarParams()
    fe_params = fe_params or FrontEndParams()
    scenario, sim = make_sim(
        meta["scenario"], int(meta["seed"]), float(meta["duration_s"]), params,
        **meta["scenario_args"],
    )  # fmt: skip
    # Cache key: parameters and the front-end source (a code change invalidates it).
    src = Path(frontend_module.__file__).read_bytes()
    key = repr(fe_params) + hashlib.sha1(src).hexdigest()
    cache_file = run / f"detections_{fe_params.tracking}.pkl"
    cached = None
    if cache and cache_file.exists():
        blob = pickle.loads(cache_file.read_bytes())
        if blob.get("key") == key and blob.get("n_parts") == len(sim.world.parts):
            cached = blob
    if cached is None:
        raw = np.load(run / "raw.npz")
        fe_rng = np.random.default_rng(int(meta["seed"]) + 40_000)
        per_agent, stats = {}, {}
        n_parts = len(sim.world.parts)
        for aid, ad in sim.agents.items():
            name = ad.config.name
            sensors = meta["rigs"].get(name, [])
            if ad.config.role != "slam" or not sensors:
                continue
            specs = {s: GZ_SENSORS[s] for s in sensors}
            data = {s: raw[f"{name}/{s}"] for s in sensors}
            offset = n_parts + TRACK_ID_STRIDE * (aid + 1)
            dets, st = detections_for_agent(
                ad, data, specs, sim.world, sim.instance_descriptors, offset, fe_params, fe_rng
            )
            per_agent[aid], stats[name] = dets, st
        cached = {"key": key, "n_parts": n_parts, "dets": per_agent, "stats": stats}
        if cache:
            cache_file.write_bytes(pickle.dumps(cached))
    agents = dict(sim.agents)
    for aid, ad in sim.agents.items():
        if ad.config.role != "slam":
            continue
        dets = cached["dets"].get(aid)
        if dets is None:  # sensors not emulated in Tier 2: no detections
            dets = [[] for _ in ad.keyframes]
        kfs = [
            dataclasses.replace(kf, detections=d) for kf, d in zip(ad.keyframes, dets, strict=True)
        ]
        agents[aid] = dataclasses.replace(ad, keyframes=kfs)
    sim2 = dataclasses.replace(sim, agents=agents)
    return scenario, sim2, cached["stats"]
