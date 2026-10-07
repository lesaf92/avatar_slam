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
import scipy

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.sim.measurements import SimData
from avatar.sim.scenarios import Scenario
from avatar.tier2.frontend import FrontEndParams, detections_for_agent
from avatar.tier2.sdf import DAVE_SONARS, GZ_SENSORS, SONAR_DB_MAX, SONAR_DB_MIN
from avatar.tier2.sonar_image import SonarFrames

TRACK_ID_STRIDE = 1_000_000
_PKG = Path(__file__).resolve().parents[1]  # the `avatar` package


def frontend_sources() -> list[Path]:
    """Source files the Tier-2 detections depend on (they key the detection cache).

    The front-end and its tracker, and the scenario and sensor models it reads. Until
    2026-10-06 only ``frontend.py`` and ``sonar_image.py`` keyed it, so a change to the EKF
    tracker kept stale caches (no stored cache was affected: LOG L39).
    """
    files = [*(_PKG / "tier2").glob("*.py"), *(_PKG / "sim").glob("*.py")]
    files += [
        _PKG / f for f in ("frontend/ekf_tracker.py", "geometry.py", "semantics.py", "types.py")
    ]
    return sorted(files)


def load_meta(run_dir: str | Path) -> dict:
    """Recording metadata (``meta.json``) of a Tier-2 run directory."""
    return json.loads((Path(run_dir) / "meta.json").read_text())


def build_tier2_sim(
    run_dir: str | Path,
    params: AvatarParams | None = None,
    fe_params: FrontEndParams | None = None,
    cache: bool = True,
    sonar: str | None = None,
) -> tuple[Scenario, SimData, dict]:
    """Scenario, Tier-2 SimData and front-end statistics for one recorded run.

    Front-end output is cached in ``<run_dir>/detections_<tracking>.pkl`` (keyed
    by the front-end parameters and source) because segmentation takes a while.

    ``sonar`` names a sonar recording of the run (``<run_dir>/<sonar>.npz``, made by
    ``experiments/gazebo/record_sonar.py``): the sensors it holds are read from it as images
    of DAVE's multibeam sonar instead of the ray-cast proxy (ADR-0008).
    """
    run = Path(run_dir)
    meta = load_meta(run)
    params = params or AvatarParams()
    fe_params = fe_params or FrontEndParams()
    scenario, sim = make_sim(
        meta["scenario"], int(meta["seed"]), float(meta["duration_s"]), params,
        **meta["scenario_args"],
    )  # fmt: skip
    # Cache key: parameters, the source the detections depend on (a code change invalidates it),
    # the NumPy/SciPy versions (random-number streams and solvers differ, LOG L33) and the
    # recordings themselves (a re-recorded run must not reuse old detections, L39).
    src = b"".join(p.read_bytes() for p in frontend_sources())
    key = (
        repr(fe_params)
        + hashlib.sha1(src).hexdigest()
        + f"|np{np.__version__}|sp{scipy.__version__}"
    )
    sonar_file = run / f"{sonar}.npz" if sonar else None
    for name, f in (("raw", run / "raw.npz"), (sonar, sonar_file)):
        if f is not None:
            st = f.stat()
            key += f"|{name}:{st.st_size}:{st.st_mtime_ns}"
    # Non-default front-end parameters get their own file, so that variants of one run can be
    # built side by side (in parallel) without overwriting each other's cache.
    variant = ""
    if fe_params != FrontEndParams(tracking=fe_params.tracking):
        variant = "_" + hashlib.sha1(repr(fe_params).encode()).hexdigest()[:8]
    cache_file = run / f"detections_{fe_params.tracking}{'_' + sonar if sonar else ''}{variant}.pkl"
    cached = None
    if cache and cache_file.exists():
        blob = pickle.loads(cache_file.read_bytes())
        if blob.get("key") == key and blob.get("n_parts") == len(sim.world.parts):
            cached = blob
    if cached is None:
        raw = np.load(run / "raw.npz")
        sz = np.load(sonar_file) if sonar_file is not None else None
        fe_rng = np.random.default_rng(int(meta["seed"]) + 40_000)
        per_agent, stats = {}, {}
        n_parts = len(sim.world.parts)
        for aid, ad in sim.agents.items():
            name = ad.config.name
            sensors = meta["rigs"].get(name, [])
            if ad.config.role != "slam" or not sensors:
                continue
            specs, data = {}, {}
            for s in sensors:
                if sz is not None and f"{name}/{s}" in sz.files and s in DAVE_SONARS:
                    specs[s] = DAVE_SONARS[s].image_spec()
                    data[s] = SonarFrames(
                        sz[f"{name}/{s}"],
                        sz[f"{name}/{s}/range_m"],
                        sz[f"{name}/{s}/azimuth_rad"],
                        SONAR_DB_MIN,
                        SONAR_DB_MAX,
                    )
                else:
                    specs[s] = GZ_SENSORS[s]
                    data[s] = raw[f"{name}/{s}"]
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
