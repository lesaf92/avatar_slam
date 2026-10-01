"""Front-end for sonar images: an echo-level image over range and azimuth -> landmark parts.

An imaging sonar returns, per ping, the echo level [dB] over range and azimuth; elevation is
integrated away. The image of DAVE's multibeam sonar (ADR-0008) has speckle, a noise floor that
falls with range, and, around every strong echo, an arc of constant range across the fan (azimuth
side lobes). None of these exist in the ray-cast proxy of ADR-0007. The detector is therefore
image-based (task T-F2-05):

1. **Noise floor.** The median over the beams of each range row. An arc that spans most of the fan
   raises the median of its row and disappears; compact echoes stay.
2. **Detections.** Cells more than ``threshold_db`` above the floor that are the maximum of their
   neighbourhood (``nms_range_m`` by ``nms_beams``), strongest first, each removing its neighbours.
3. **Rejection.** A peak is dropped if (a) it lies at the border of the fan (a partial view),
   (b) the strongest echo at its range, away from its own beams, is more than ``arc_db`` stronger
   (it is a side lobe: the arc of constant range around a strong echo), or (c) the region within
   ``ext_db`` of it is longer than ``max_footprint_m`` (an extended object: a hull, the quay,
   which has no viewpoint-invariant centre).
4. **Measurement.** The *face* of the object, the point of its surface nearest to the sonar, is
   the leading edge of the echo along the range (the level rises through ``peak - edge_db``);
   the azimuth is the power-weighted centroid of the beams within ``face_db`` of the peak. The
   echo of a cylinder trails a few tenths of a metre behind the face, so a centroid in range is
   biased. The centre of a round object lies one radius behind the face: the mean radius of the
   round structures (``radius_prior_m``) is added and the error is left to the covariance.

Nothing here reads the ground truth. Elevation is not observed (as with the proxy).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import label, maximum_filter

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SonarImage:
    """One sonar frame: echo level [dB] by range row and azimuth column."""

    db: NDArray[np.float32]  # (rows, beams)
    range_m: FloatArray  # (rows,) bin-centre ranges, ascending
    azimuth_rad: FloatArray  # (beams,) ascending; 0 = boresight, positive = left


class SonarFrames:
    """Stored frames of one sonar (uint8 echo-level codes); ``frames[k]`` is an image."""

    def __init__(
        self,
        codes: NDArray[np.uint8],
        range_m: FloatArray,
        azimuth_rad: FloatArray,
        db_min: float,
        db_max: float,
    ) -> None:
        self.codes = codes
        self.range_m = np.asarray(range_m, dtype=np.float64)
        self.azimuth_rad = np.asarray(azimuth_rad, dtype=np.float64)
        self._scale = np.float32((db_max - db_min) / 255.0)
        self._db_min = np.float32(db_min)

    def __len__(self) -> int:
        return len(self.codes)

    def __getitem__(self, k: int) -> SonarImage:
        db = self.codes[k].astype(np.float32) * self._scale + self._db_min
        return SonarImage(db, self.range_m, self.azimuth_rad)


@dataclass(frozen=True)
class SonarImageParams:
    """Detector parameters (see the module docstring)."""

    threshold_db: float = 20.0
    nms_range_m: float = 0.6
    nms_beams: int = 4
    face_db: float = 6.0
    window_beams: int = 3
    min_range_m: float = 0.8
    edge_beams: int = 2
    arc_db: float = 12.0
    arc_band_m: float = 0.5
    ext_db: float = 6.0
    ext_window_m: float = 3.0
    ext_window_beams: int = 12
    contrast_db: float = 15.0
    contrast_pct: float = 90.0
    chain_radius_m: float = 3.0
    chain_db: float = 10.0
    chain_min: int = 2
    chain_dominance_db: float = 6.0
    max_footprint_m: float = 2.5
    edge_db: float = 3.0
    max_lead_m: float = 1.5
    radius_prior_m: float = 0.38
    centre_sigma_m: float = 0.2  # error of the centre estimate, added to the sensor model's sigma


@dataclass(frozen=True)
class SonarBlob:
    """One detection: estimated object centre in the sensor frame [m] and its extents."""

    x: float
    y: float
    range_m: float  # to the estimated centre
    cross_range_m: float  # extent across the beams at the -face_db level
    along_range_m: float  # extent along the range at the -face_db level
    peak_db: float  # above the noise floor of its range row


def detect_blobs(
    img: SonarImage, p: SonarImageParams, rejects: list | None = None
) -> list[SonarBlob]:
    """Compact echoes of one sonar image, as objects (centre estimate) in the sensor frame.

    ``rejects``, if given, receives ``(row, beam, level_db, rule)`` of every rejected peak.
    """
    db = img.db
    rows, beams = db.shape
    if rows < 8 or beams < 8:
        return []
    dr = float(np.median(np.diff(img.range_m)))
    dtheta = float(np.median(np.diff(img.azimuth_rad)))
    res = db - np.median(db, axis=1, keepdims=True)
    nr = max(round(p.nms_range_m / dr), 1)
    band = max(round(p.arc_band_m / dr), 1)
    ext_rows = max(round(p.ext_window_m / dr), 1)
    peak = res == maximum_filter(res, size=(2 * nr + 1, 2 * p.nms_beams + 1), mode="nearest")
    peak &= res > p.threshold_db
    peak &= (img.range_m >= p.min_range_m)[:, None]
    cand = np.argwhere(peak)
    if len(cand) == 0:
        return []
    order = np.argsort(-res[cand[:, 0], cand[:, 1]], kind="stable")
    taken = np.zeros((rows, beams), dtype=bool)
    peaks: list[tuple[int, int]] = []
    for ri, bi in cand[order]:
        if taken[ri, bi]:
            continue
        taken[max(ri - nr, 0) : ri + nr + 1, max(bi - p.nms_beams, 0) : bi + p.nms_beams + 1] = True
        peaks.append((int(ri), int(bi)))
    # (e) chain: an extended object (hull, wall) gives several comparable peaks within a few metres,
    # none dominating the others
    lv = np.array([res[ri, bi] for ri, bi in peaks])
    xy = np.array(
        [
            [
                img.range_m[ri] * np.cos(img.azimuth_rad[bi]),
                img.range_m[ri] * np.sin(img.azimuth_rad[bi]),
            ]
            for ri, bi in peaks
        ]
    )
    dist = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1])
    within = dist < p.chain_radius_m
    np.fill_diagonal(within, False)
    near = within & (lv[None, :] >= lv[:, None] - p.chain_db)
    # a peak that stands well above every neighbour is an object with its side lobes, not a chain
    strongest = np.where(within, lv[None, :], -np.inf).max(axis=1)
    in_chain = (near.sum(axis=1) >= p.chain_min) & (lv - strongest < p.chain_dominance_db)
    out: list[SonarBlob] = []
    for n_peak, (ri, bi) in enumerate(peaks):
        level = float(res[ri, bi])
        if in_chain[n_peak]:
            if rejects is not None:
                rejects.append((ri, bi, level, "chain"))
            continue
        if bi < p.edge_beams or bi >= beams - p.edge_beams or ri >= rows - 2:
            if rejects is not None:
                rejects.append((ri, bi, level, "border"))
            continue  # cut by the fan border: a partial view
        # (b) side lobe: a much stronger echo at this range, outside the peak's own beams
        rb = slice(max(ri - band, 0), ri + band + 1)
        own = slice(max(bi - p.nms_beams - 3, 0), bi + p.nms_beams + 4)
        others = np.concatenate([res[rb, : own.start].ravel(), res[rb, own.stop :].ravel()])
        if others.size and float(others.max()) - level > p.arc_db:
            if rejects is not None:
                rejects.append((ri, bi, level, "side lobe"))
            continue
        # (c) extended object: the region within ext_db of the peak is longer than any pile
        r0, r1 = max(ri - ext_rows, 0), min(ri + ext_rows + 1, rows)
        b0, b1 = max(bi - p.ext_window_beams, 0), min(bi + p.ext_window_beams + 1, beams)
        lab, _ = label(res[r0:r1, b0:b1] >= level - p.ext_db, structure=np.ones((3, 3)))
        reg = np.argwhere(lab == lab[ri - r0, bi - b0])
        d_along = (np.ptp(reg[:, 0]) + 1) * dr
        d_cross = (np.ptp(reg[:, 1]) + 1) * dtheta * float(img.range_m[ri])
        if float(np.hypot(d_along, abs(d_cross))) > p.max_footprint_m:
            if rejects is not None:
                rejects.append((ri, bi, level, "extent"))
            continue
        # (d) local contrast: a wall, an arc or a ridge is surrounded by cells of its own level
        ring = res[r0:r1, b0:b1].copy()
        ring[
            max(ri - r0 - nr, 0) : ri - r0 + nr + 1,
            max(bi - b0 - p.nms_beams, 0) : bi - b0 + p.nms_beams + 1,
        ] = np.nan
        if level - float(np.nanpercentile(ring, p.contrast_pct)) < p.contrast_db:
            if rejects is not None:
                rejects.append((ri, bi, level, "contrast"))
            continue
        # measurement: azimuth by centroid, face range by the leading edge of the echo
        c0, c1 = max(bi - p.window_beams, 0), min(bi + p.window_beams + 1, beams)
        cols = res[max(ri - nr, 0) : ri + nr + 1, c0:c1]
        w = np.where(cols >= level - p.face_db, 10.0 ** (cols / 10.0), 0.0)
        a = float((w.sum(axis=0) * img.azimuth_rad[c0:c1]).sum() / w.sum())
        prof = res[:, max(bi - 1, 0) : bi + 2].max(axis=1)
        lo = max(ri - round(p.max_lead_m / dr), 0)
        below = np.nonzero(prof[lo : ri + 1] < level - p.edge_db)[0]
        j = lo + int(below[-1]) if len(below) else lo
        if j < ri:
            f = (level - p.edge_db - prof[j]) / max(prof[j + 1] - prof[j], 1e-6)
            r_face = float(
                img.range_m[j] + np.clip(f, 0.0, 1.0) * (img.range_m[j + 1] - img.range_m[j])
            )
        else:
            r_face = float(img.range_m[ri])
        r_c = r_face + p.radius_prior_m
        out.append(
            SonarBlob(
                r_c * np.cos(a),
                r_c * np.sin(a),
                r_c,
                abs(d_cross),
                d_along,
                level,
            )
        )
    return out


__all__ = [
    "SonarBlob",
    "SonarFrames",
    "SonarImage",
    "SonarImageParams",
    "detect_blobs",
]
