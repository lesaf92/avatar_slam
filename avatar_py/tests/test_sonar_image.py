"""Sonar-image front-end and the acoustic world (ADR-0008, docs/LOG.md L34): no Gazebo needed."""

from __future__ import annotations

import re

import numpy as np

from avatar.agent import AvatarParams
from avatar.runner import make_sim
from avatar.tier2.frontend import FrontEndParams, segment
from avatar.tier2.sdf import (
    DAVE_SONARS,
    SONAR_DB_MAX,
    SONAR_DB_MIN,
    sonar_image_specs,
    sonar_rigs,
    sonar_world_sdf,
)
from avatar.tier2.sonar_image import SonarFrames, SonarImage, SonarImageParams, detect_blobs

ROWS, BEAMS, DR = 940, 129, 0.0319


def _image(seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Noise floor with speckle-like fluctuation, over range and azimuth."""
    rng = np.random.default_rng(seed)
    r = (np.arange(ROWS) + 0.5) * DR
    az = np.linspace(-np.pi / 4, np.pi / 4, BEAMS)
    db = 10.0 + 4.0 * rng.standard_normal((ROWS, BEAMS))
    return db, r, az


def _add_blob(db, r, az, r0, a0_deg, amp=40.0, sr=0.15, sa_deg=0.7):
    rr, aa = np.meshgrid(r, az, indexing="ij")
    db += amp * np.exp(
        -0.5 * ((rr - r0) / sr) ** 2 - 0.5 * ((aa - np.radians(a0_deg)) / np.radians(sa_deg)) ** 2
    )


def _detect(db, r, az, **kw):
    img = SonarImage(db.astype(np.float32), r, az)
    return detect_blobs(img, SonarImageParams(**kw))


def test_compact_echo_is_detected_behind_its_face() -> None:
    db, r, az = _image()
    _add_blob(db, r, az, 12.0, 10.0)
    (b,) = _detect(db, r, az)
    # the echo peaks at the face; the centre is one nominal radius behind it
    assert abs(b.range_m - (12.0 + SonarImageParams().radius_prior_m)) < 0.5
    assert abs(np.degrees(np.arctan2(b.y, b.x)) - 10.0) < 1.0


def test_arc_across_the_fan_is_not_an_object() -> None:
    db, r, az = _image()
    row = int(20.0 / DR)
    db[row - 1 : row + 2, :] += 30.0  # constant range across all beams (a seabed edge)
    _add_blob(db, r, az, 12.0, -5.0)
    blobs = _detect(db, r, az)
    assert len(blobs) == 1
    assert abs(blobs[0].range_m - 12.4) < 0.6


def test_chain_of_comparable_peaks_is_a_wall() -> None:
    """Glints along one face: the chain rule removes most of them (speckle lets the strongest
    stand out by 6 dB now and then, which the exemption for a dominant peak keeps)."""
    with_rule, without = [], []
    for seed in range(10):
        db, r, az = _image(seed)
        for a in np.arange(-22.8, 22.9, 5.7):  # nine glints, about 1.5 m apart at 15 m
            _add_blob(db, r, az, 15.0, float(a))
        with_rule.append(len(_detect(db, r, az)))
        without.append(len(_detect(db, r, az, chain_min=1000)))
    assert np.mean(without) > 5.0
    assert np.mean(with_rule) < 0.4 * np.mean(without)


def test_two_separate_piles_are_two_objects() -> None:
    db, r, az = _image()
    _add_blob(db, r, az, 10.0, -20.0)
    _add_blob(db, r, az, 18.0, 15.0, amp=35.0)
    assert len(_detect(db, r, az)) == 2


def test_echo_at_the_fan_border_is_partial() -> None:
    db, r, az = _image()
    _add_blob(db, r, az, 12.0, 45.0)
    assert _detect(db, r, az) == []


def test_frames_decode_to_the_stored_level() -> None:
    db, r, az = _image()
    lo, hi = SONAR_DB_MIN, SONAR_DB_MAX
    codes = np.clip(np.rint((db - lo) / (hi - lo) * 255), 0, 255).astype(np.uint8)
    frames = SonarFrames(codes[None], r, az, lo, hi)
    assert len(frames) == 1
    step = (hi - lo) / 255
    assert np.max(np.abs(frames[0].db - np.clip(db, lo, hi))) <= step


def test_segment_turns_a_sonar_image_into_clusters() -> None:
    db, r, az = _image()
    _add_blob(db, r, az, 9.0, 0.0)
    spec = sonar_image_specs()["gemini_720s"]
    (c,) = segment(
        spec,
        SonarImage(db.astype(np.float32), r, az),
        -4.0,
        -12.0,
        FrontEndParams(),
        np.random.default_rng(0),
    )
    assert c.one_sided and abs(c.p_body[2]) < 1e-9 and c.range_m > 8.5


def test_dave_sonar_image_size() -> None:
    d = DAVE_SONARS["gemini_720s"]
    spec = d.image_spec()
    assert spec.kind == "sonar_image"
    assert spec.h_samples == d.beams + 1  # DAVE gives one column more than beams
    assert spec.v_samples == d.raw_range_bins // 4 == 940


def test_acoustic_world_holds_only_what_is_below_the_waterline() -> None:
    scenario, sim = make_sim("harbor_fleet", 0, 30.0, AvatarParams())
    rigs = sonar_rigs(scenario)
    assert set(rigs) == {"uuv_0", "uuv_1"}
    names = {a.agent_id: a.name for a in scenario.agents}
    poses = {names[i]: ad.gt[0] for i, ad in sim.agents.items() if names[i] in rigs}
    sdf = sonar_world_sdf(scenario, poses, "w")
    assert sdf.count('gz:type="multibeam_sonar"') == 2 and "multibeam_sonar_system" in sdf
    assert sdf.count("<gz:multibeam_sonar>") == 2
    # every static model reaches at most z = 0 (pose z + half its height)
    for m in re.finditer(r'<model name="(obj_|land|seabed)[^>]*>(.*?)</model>', sdf, re.S):
        body = m.group(2)
        z = float(re.search(r"<pose>\S+ \S+ (\S+) ", body).group(1))
        h = re.search(r"<length>(\S+)</length>", body) or re.search(
            r"<size>\S+ \S+ (\S+)</size>", body
        )
        assert z + 0.5 * float(h.group(1)) <= 1e-3, m.group(1)
    below = sum(1 for s in scenario.world.structures if s.z_min < 0.0)
    assert sdf.count('<model name="obj_') == below
    # the vertical rays are written at half the aperture (DAVE illuminates twice the SDF angle)
    v = re.search(r"<vertical><rays>\d+</rays><min_angle>(\S+)</min_angle>", sdf)
    assert abs(float(v.group(1)) + DAVE_SONARS["gemini_720s"].v_fov_rad / 4) < 1e-3
