"""Geometric front-end for Tier-2 range data: returns → landmark-part detections.

Per sensor and keyframe (tasks T-F2-02, T-F2-03):

1. **Noise.** Range noise is added here (the recorded data is noise-free), from
   the device models in ``docs/hardware.md``: VLP-16 σ = 3 cm; D435i
   σ = 3 mm + 0.5 %·d² (depth ∝ d²); Gemini proxy σ = 2 cm.
2. **Medium gating.** With the agent's height ``z_a`` (pressure / barometer /
   odometry) and the gravity-aligned body frame, each return's height is
   ``z_a + p_z``. Optical returns below ``+optical_min_z_m`` never exist in
   reality (the water surface stops light), acoustic returns above
   ``-acoustic_max_z_m`` are surface multipath: both are removed.
3. **Ground removal.** Optical: returns within ``ground_band_m`` of the dominant
   horizontal plane below the sensor (the quay, land). Acoustic: returns
   within ``seabed_band_m`` of the seabed, whose height a DVL measures
   (altitude); Tier 2 uses the flat scenario seabed.
4. **Euclidean clustering** (KD-tree radius graph, connected components). The
   sonar is clustered in its horizontal (range, bearing) plane because an
   imaging sonar does not resolve elevation.
5. **Cluster → part.** Horizontal position by an algebraic circle fit for small
   round clusters, otherwise the centre of the principal-axes bounding box.
   A one-sided view of a large object biases this centre; its σ therefore
   grows with the cluster's smaller footprint dimension. Height: optical,
   middle of the observed span clipped at the waterline; acoustic, the fan
   centre (elevation unknown) with the imaging-sonar elevation σ.
   Walls, the quay face and seabed patches are rejected by size and
   elongation.

The **tracker** (task T-F3-01) associates each detection with the agent's own
tracks by gated nearest neighbour in the agent's dead-reckoning frame (no
ground truth). The gate grows with the distance travelled since the track was
last seen. Its errors (duplicates after drift, switches between close parts)
are real front-end errors, which the robust back-end (GNC) must absorb.

Semantic labels and descriptors are **not** rendered: a detection that lies
near a ground-truth part takes that part's class and descriptor through the
Tier-1 label model of the sensor (``avatar.sim.sensors``); other clusters are
unlabelled. Replacing this with an image-based detector is task T-F2-01.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from avatar.frontend.ekf_tracker import (
    BAD,
    CONFIRMED,
    EkfTracker,
    EkfTrackerParams,
    PlatformPrior,
)
from avatar.geometry import compose, transform_points
from avatar.semantics import CLASS_NAMES, class_id, normalize
from avatar.sim.agents import heading_bias_modelled
from avatar.sim.sensors import SENSOR_LIBRARY, Detection, DetectionSensor
from avatar.tier2.rays import ray_angles, sensor_points
from avatar.tier2.sdf import RaySensorSpec
from avatar.tier2.sonar_image import SonarImage, SonarImageParams, detect_blobs
from avatar.types import LandmarkFlags, Medium

FloatArray = NDArray[np.float64]
TRACK_SPAN = 100_000  # id sub-ranges per agent: parts, objects, clutter


@dataclass(frozen=True)
class FrontEndParams:
    """Tuning of the Tier-2 geometric front-end (SI units)."""

    lidar_range_sigma_m: float = 0.03
    depth_sigma_base_m: float = 0.003
    depth_sigma_per_m2: float = 0.005
    sonar_range_sigma_m: float = 0.02
    optical_min_z_m: float = 0.15
    acoustic_max_z_m: float = 0.3
    ground_band_m: float = 0.25
    seabed_band_m: float = 0.5
    voxel_m: float = 0.1  # downsampling before clustering (dense depth images)
    cluster_radius_m: float = 0.5
    # Vertical distances are scaled by this factor before clustering, so that the
    # rings a sparse LiDAR leaves on one tall pole (2° apart) form one cluster.
    cluster_z_scale: float = 0.2
    sonar_cluster_radius_m: float = 0.7
    min_points: dict[str, int] = field(
        default_factory=lambda: {"lidar": 6, "depth": 40, "sonar": 4}
    )
    max_footprint_m: float = 18.0
    max_elongation: float = 8.0  # length / width above which a long cluster is a wall
    min_wall_length_m: float = 6.0
    circle_max_diameter_m: float = 3.5
    # One-sided view of a box-like object: its centre may lie anywhere behind the
    # visible face, so σ_h += frac · larger footprint dimension.
    one_sided_sigma_frac: float = 0.3
    truncated_sigma_z_m: float = 3.0  # σ_z when the object leaves the vertical fan
    # Report only compact, round clusters (circle fit accepted). Extended objects
    # (hulls, containers) seen from one side have no viewpoint-invariant centre:
    # their point-landmark centres were biased by 4-6 m (docs/LOG.md L28).
    keep_extended: bool = False
    circle_max_rms_m: dict[str, float] = field(
        default_factory=lambda: {"lidar": 0.08, "depth": 0.08, "sonar": 0.25}
    )
    max_round_elongation: float = 3.0  # box faces / fragments are elongated
    # Intra-agent association (docs/LOG.md L28):
    #   "oracle": ground-truth identity of the part a detection measures (as in
    #             Tier 1); detections of no part are one-off clutter;
    #   "nn": gated nearest neighbour in the dead-reckoning frame, gate growing
    #         with the distance since the track was last seen, ambiguity test;
    #   "registration": keyframe-to-map registration, then tight gating;
    #   "ekf": a filter over the pose and the odometry biases with per-landmark
    #          covariances and Mahalanobis gating (avatar.frontend.ekf_tracker, T-F3-02).
    tracking: str = "oracle"
    # Tracker (dead-reckoning frame)
    track_gate_m: float = 1.0
    track_gate_sigmas: float = 3.0
    # Registration search radius [m]: gate + growth per metre travelled since the
    # last accepted registration (dead-reckoning drift), capped.
    track_drift_per_m: float = 0.02
    track_search_max_m: float = 8.0
    reg_inlier_m: float = 0.6  # inlier radius of a registration hypothesis
    reg_min_inliers: int = 2
    reg_distinct_m: float = 1.0  # hypotheses closer than this count as the same
    # Re-identify only tracks seen in the last ``track_max_age_kf`` keyframes
    # (None: all). Kept for ablations; registration makes old tracks safe.
    track_max_age_kf: int | None = None
    # Ambiguity test per detection: a second track within ``ratio`` × the best
    # distance (inside the gate) starts a new track instead of a risky match.
    track_ambiguity_ratio: float | None = 2.0
    # What an ambiguous detection does: "drop" (no update, no new track) or "new"
    # (start a track). "new" runs away: each duplicate makes the next detection
    # ambiguous too (2 161 tracks for 48 parts on the UGV, docs/LOG.md L28).
    track_on_ambiguity: str = "drop"
    ekf: EkfTrackerParams = field(default_factory=EkfTrackerParams)
    # Tracker parameters of an agent whose sensors include a sonar image (None: ``ekf``).
    # The birth test that rejects clutter on the proxy passes the glints of the quay and the
    # hulls on sonar images, whose centre error is much larger (docs/LOG.md L36, T-F3-06).
    ekf_sonar: EkfTrackerParams | None = None
    # Sonar images (spec kind "sonar_image", DAVE's multibeam sonar, ADR-0008)
    sonar_image: SonarImageParams = field(default_factory=SonarImageParams)
    # Ping360 (T-S1-11): seconds per 360° turn of the mechanical scanner; each keyframe keeps
    # the sector swept since the previous one. 21 s interpolates the product page's 3.4-4.3 s
    # at a 1 m range setting and 33 s at 50 m to the 30 m used here (UNVERIFIED). 0: the whole
    # turn at every keyframe (an upper bound).
    ping360_sweep_s: float = 21.0
    # A Ping360 *image* (DAVE, T-S1-12) keeps this much more on each side of the swept sector,
    # so that a pile cut by one sector's border lies whole in the next one (the detector drops
    # echoes at the border of an image); 5° is 1 m at 11 m.
    ping360_margin_rad: float = float(np.deg2rad(5.0))
    # A Ping360 image is detected on the whole turn (the previous turn's pings are at hand on
    # the vehicle), rolled so that the swept sector lies in the middle, and only the detections
    # in the sector are kept: the noise floor, the side-lobe arcs and the extent of a hull or
    # the quay face are judged on the whole turn, not on a 20-30° cut (T-F3-08).
    ping360_turn_context: bool = False
    # Sensors whose data are ignored (paired controls: the same recording without them).
    ignore_sensors: tuple[str, ...] = ()
    # Semantic oracle: max distance [m] from a detection to a GT part's surface
    label_match_m: float = 1.5


@dataclass
class Cluster:
    """One segmented object in the body frame."""

    p_body: FloatArray  # (3,) centre [m]
    footprint: FloatArray  # (2,) sorted descending [m]
    height_m: float
    range_m: float
    one_sided: bool
    z_truncated: bool = False


def _voxel_downsample_idx(p: FloatArray, voxel: float) -> NDArray[np.int64]:
    """Indices of one point per occupied voxel (first in input order)."""
    if len(p) == 0:
        return np.zeros(0, dtype=np.int64)
    keys = np.floor(p / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return np.sort(idx)


def _components(p2or3: FloatArray, radius: float) -> NDArray[np.int64]:
    tree = cKDTree(p2or3)
    pairs = tree.query_pairs(radius, output_type="ndarray")
    n = len(p2or3)
    if len(pairs) == 0:
        return np.arange(n)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)[1]


def _circle_fit(xy: FloatArray) -> tuple[FloatArray, float] | None:
    """Algebraic (Kåsa) circle fit; ``None`` if ill-conditioned."""
    if len(xy) < 5:
        return None
    A = np.column_stack([2 * xy[:, 0], 2 * xy[:, 1], np.ones(len(xy))])
    b = (xy**2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:2]
    r2 = sol[2] + c @ c
    if not np.isfinite(r2) or r2 <= 0:
        return None
    return c, float(np.sqrt(r2))


def _footprint(xy: FloatArray) -> tuple[FloatArray, FloatArray]:
    """Centre of the principal-axes bounding box and its sorted dimensions."""
    mu = xy.mean(axis=0)
    if len(xy) < 3:
        return mu, np.array([0.1, 0.1])
    cov = np.cov((xy - mu).T)
    _, vecs = np.linalg.eigh(cov)
    proj = (xy - mu) @ vecs
    lo, hi = proj.min(axis=0), proj.max(axis=0)
    centre = mu + vecs @ (0.5 * (lo + hi))
    dims = np.maximum(hi - lo, 0.05)
    return centre, np.sort(dims)[::-1]


def _range_noise(spec: RaySensorSpec, range_m: float, params: FrontEndParams) -> float:
    """1-σ range noise [m] of a return at ``range_m`` (plus sonar beam quantization)."""
    if spec.kind == "lidar":
        return params.lidar_range_sigma_m
    if spec.kind == "depth":
        return params.depth_sigma_base_m + params.depth_sigma_per_m2 * range_m**2
    beam = spec.h_fov_rad / max(spec.h_samples - 1, 1)
    return params.sonar_range_sigma_m + range_m * beam / np.sqrt(12.0)


def _dominant_plane_z(z: FloatArray, below: float, band: float) -> float | None:
    """Height of the dominant horizontal plane among returns lower than ``below``."""
    zz = z[z < below]
    if len(zz) < 20:
        return None
    bins = np.arange(zz.min() - band, zz.max() + band + 1e-9, band)
    if len(bins) < 2:
        return float(np.median(zz))
    hist, edges = np.histogram(zz, bins=bins)
    i = int(np.argmax(hist))
    if hist[i] < 0.15 * len(zz):
        return None
    return float(0.5 * (edges[i] + edges[i + 1]))


def segment(
    spec: RaySensorSpec,
    data: NDArray | SonarImage,
    agent_z_m: float,
    seabed_z_m: float,
    params: FrontEndParams,
    rng: np.random.Generator,
) -> list[Cluster]:
    """Clusters (body frame) in one scan / depth image, after noise and gating.

    A sonar image (``spec.kind == "sonar_image"``) is detected as such
    (:mod:`avatar.tier2.sonar_image`); it carries its own noise and its acoustic world holds
    only what is below the waterline, so no noise is added and no medium gating is needed.
    """
    if spec.kind == "sonar_image":
        assert isinstance(data, SonarImage)
        # The image cannot measure a diameter: the footprint is 0, "not measured" (wire format
        # v0 §3), so the association neither checks nor ranks by size (docs/LOG.md L35). The
        # error of the centre goes into the covariance (centre_sigma_m, detections_for_agent).
        return [
            Cluster(np.array([b.x, b.y, 0.0]), np.zeros(2), 0.05, b.range_m, False, False)
            for b in detect_blobs(data, params.sonar_image)
        ]
    d = np.asarray(data, dtype=np.float64).copy()
    finite = np.isfinite(d)
    if spec.kind == "lidar":
        d[finite] += rng.normal(0.0, params.lidar_range_sigma_m, finite.sum())
    elif spec.kind == "depth":
        sig = params.depth_sigma_base_m + params.depth_sigma_per_m2 * d[finite] ** 2
        d[finite] += rng.normal(0.0, 1.0, finite.sum()) * sig
    else:
        d[finite] += rng.normal(0.0, params.sonar_range_sigma_m, finite.sum())
    p, valid = sensor_points(spec, d)
    if len(p) == 0:
        return []
    row, col = np.nonzero(valid)  # grid index of every point (row = elevation / image row)
    z_w = agent_z_m + p[:, 2]
    if spec.kind == "sonar":
        keep = (z_w < -params.acoustic_max_z_m) & (z_w > seabed_z_m + params.seabed_band_m)
        p, row, col = p[keep], row[keep], col[keep]
        if len(p) == 0:
            return []
        r = np.linalg.norm(p, axis=1)
        b = np.arctan2(p[:, 1], p[:, 0])
        xy = np.column_stack([r * np.cos(b), r * np.sin(b)])  # elevation dropped
        labels = _components(xy, params.sonar_cluster_radius_m)
        pts2 = xy
    else:
        keep = z_w > params.optical_min_z_m
        p, z_w, row, col = p[keep], z_w[keep], row[keep], col[keep]
        gz = _dominant_plane_z(z_w, agent_z_m - 0.3, params.ground_band_m)
        if gz is not None:
            keep = np.abs(z_w - gz) > params.ground_band_m
            p, z_w, row, col = p[keep], z_w[keep], row[keep], col[keep]
        if spec.kind == "depth":
            idx = _voxel_downsample_idx(p, params.voxel_m)
            p, row, col = p[idx], row[idx], col[idx]
        if len(p) == 0:
            return []
        scaled = p * np.array([1.0, 1.0, params.cluster_z_scale])
        labels = _components(scaled, params.cluster_radius_m)
        pts2 = p[:, :2]
    min_pts = params.min_points[spec.kind]
    out: list[Cluster] = []
    for lab in np.unique(labels):
        m = labels == lab
        if m.sum() < min_pts:
            continue
        # Objects cut by the horizontal image / fan border are partial: skip them
        # (a 360° LiDAR has no such border).
        if spec.h_fov_rad < 2 * np.pi - 1e-6 and (
            col[m].min() == 0 or col[m].max() == spec.h_samples - 1
        ):
            continue
        # Top or bottom leaving the vertical fan / image: the height is truncated.
        # (Depth image rows grow downwards, scan rows upwards; both ends count.)
        z_trunc = bool(row[m].min() == 0 or row[m].max() == spec.v_samples - 1)
        xy = pts2[m]
        centre, dims = _footprint(xy)
        if dims[0] > params.max_footprint_m:
            continue
        if dims[0] > params.min_wall_length_m and dims[0] / dims[1] > params.max_elongation:
            continue
        one_sided = True
        if (
            dims[0] <= params.circle_max_diameter_m
            and dims[0] / dims[1] <= params.max_round_elongation
        ):
            fit = _circle_fit(xy)
            if fit is not None and fit[1] <= 0.5 * params.circle_max_diameter_m:
                rms = float(np.sqrt(np.mean((np.linalg.norm(xy - fit[0], axis=1) - fit[1]) ** 2)))
                sn = _range_noise(spec, float(np.linalg.norm(centre)), params)
                if (
                    rms <= params.circle_max_rms_m[spec.kind] + 2.0 * sn
                    and dims[0] <= 2.2 * fit[1] + 4.0 * sn
                ):
                    centre, radius = fit
                    dims = np.array([2 * radius, 2 * radius])
                    one_sided = False
        if one_sided and not params.keep_extended:
            continue
        if spec.kind == "sonar":
            z_c, height = 0.0, 0.05  # elevation (and vertical extent) unobserved
        else:
            zs = agent_z_m + p[m, 2]
            lo, hi = max(float(zs.min()), 0.0), float(zs.max())
            z_c, height = 0.5 * (lo + hi) - agent_z_m, max(hi - lo, 0.05)
        pb = np.array([centre[0], centre[1], z_c])
        out.append(Cluster(pb, dims, height, float(np.linalg.norm(pb[:2])), one_sided, z_trunc))
    return out


@dataclass
class _Track:
    tid: int
    medium: Medium
    p: FloatArray  # dead-reckoning frame
    n: int
    last_k: int
    s_last: float  # odometer at last sighting [m]
    object_key: int


class Tracker:
    """Per-agent landmark-part tracker in the dead-reckoning frame (no ground truth)."""

    def __init__(self, id_offset: int, params: FrontEndParams) -> None:
        self.params = params
        self.offset = id_offset
        self.tracks: list[_Track] = []
        self.odometer_m = 0.0
        self.correction_xy = np.zeros(2)  # map frame = dead reckoning + correction
        self.s_registered = 0.0  # odometer at the last accepted registration
        self.n_registrations = 0
        self.n_ambiguous = 0  # registrations refused for a tie
        self.n_dropped = 0  # detections dropped for an ambiguous association

    def register(self, media: list[Medium], p_map: FloatArray, sigmas: FloatArray) -> FloatArray:
        """Keyframe-to-map registration: horizontal offset that best explains the
        current detections by existing tracks (consensus over pair hypotheses).

        Returns the offset [m] (zeros if none is accepted) and updates the
        correction. A hypothesis is accepted only if it has at least
        ``reg_min_inliers`` inliers and strictly more than any distinct one
        (perceptual aliasing along a pile row leaves ties, which are refused).
        """
        prm = self.params
        n = len(media)
        if n == 0 or not self.tracks:
            return np.zeros(2)
        radius = min(
            prm.track_gate_m + prm.track_drift_per_m * (self.odometer_m - self.s_registered),
            prm.track_search_max_m,
        )
        tp = np.array([t.p[:2] for t in self.tracks])
        tm = np.array([int(t.medium) for t in self.tracks])
        hyps = []
        for i in range(n):
            d = tp - p_map[i, :2]
            near = (tm == int(media[i])) & (np.hypot(d[:, 0], d[:, 1]) < radius)
            hyps.extend(d[near])
        if not hyps:
            return np.zeros(2)
        hyps_a = np.array(hyps)

        def score(o: FloatArray) -> tuple[int, FloatArray]:
            offs = []
            used: set[int] = set()
            for i in range(n):
                q = p_map[i, :2] + o
                d = np.hypot(tp[:, 0] - q[0], tp[:, 1] - q[1])
                d[tm != int(media[i])] = np.inf
                if used:
                    d[list(used)] = np.inf
                j = int(np.argmin(d))
                if d[j] < max(prm.reg_inlier_m, 2.0 * float(sigmas[i])):
                    used.add(j)
                    offs.append(tp[j] - p_map[i, :2])
            return len(offs), (np.mean(offs, axis=0) if offs else o)

        scored = [(*score(o), o) for o in hyps_a]
        scored.sort(key=lambda x: -x[0])
        best_n, best_o, _ = scored[0]
        second = max(
            (c for c, _, o in scored if np.hypot(*(o - best_o)) > prm.reg_distinct_m), default=0
        )
        if best_n < prm.reg_min_inliers or best_n <= second:
            if best_n >= prm.reg_min_inliers:
                self.n_ambiguous += 1
            return np.zeros(2)
        self.correction_xy = self.correction_xy + best_o
        self.s_registered = self.odometer_m
        self.n_registrations += 1
        return best_o

    def associate(
        self, k: int, media: list[Medium], p_dr: FloatArray, sigmas: FloatArray
    ) -> list[tuple[int, int] | None]:
        """Track ids and object keys for all detections of one keyframe (``None``: dropped)."""
        if len(media) == 0:
            return []
        p_map = np.array(p_dr, dtype=float).copy()
        p_map[:, :2] += self.correction_xy
        p_map[:, :2] += self.register(media, p_map, sigmas)
        taken: set[int] = set()
        return [
            self.assign(k, media[i], p_map[i], float(sigmas[i]), taken) for i in range(len(media))
        ]

    def step(self, dist_m: float) -> None:
        """Advance the odometer by the distance travelled since the last keyframe."""
        self.odometer_m += dist_m

    def assign(
        self,
        k: int,
        medium: Medium,
        p_dr: FloatArray,
        sigma_h: float,
        taken: set[int],
        drift: bool = False,
    ) -> tuple[int, int] | None:
        """Track id (offset applied) and object key for one detection.

        ``drift``: grow the gate by ``track_drift_per_m`` per metre travelled since
        the track was last seen (``nn`` mode; registration mode corrects drift).
        Returns ``None`` if two tracks explain the detection about equally well
        and ``track_on_ambiguity`` is ``"drop"``.
        """
        best, best_d, second_d = None, np.inf, np.inf
        for t in self.tracks:
            if t.medium != medium or t.tid in taken:
                continue
            if (
                self.params.track_max_age_kf is not None
                and k - t.last_k > self.params.track_max_age_kf
            ):
                continue
            gate = max(self.params.track_gate_m, self.params.track_gate_sigmas * float(sigma_h))
            if drift:
                gate += self.params.track_drift_per_m * (self.odometer_m - t.s_last)
            dxy = float(np.hypot(*(t.p[:2] - p_dr[:2])))
            if dxy < gate:
                if dxy < best_d:
                    best, best_d, second_d = t, dxy, best_d
                elif dxy < second_d:
                    second_d = dxy
        ratio = self.params.track_ambiguity_ratio
        if best is not None and ratio is not None and second_d < ratio * max(best_d, 0.1):
            if self.params.track_on_ambiguity == "drop":
                self.n_dropped += 1
                return None
            best = None
        if best is None:
            key = len(self.tracks)
            other = Medium.BELOW if medium == Medium.ABOVE else Medium.ABOVE
            for t in self.tracks:  # intra-agent coaxial link: other medium, same axis
                if t.medium == other and np.hypot(*(t.p[:2] - p_dr[:2])) < 1.0:
                    key = t.object_key
                    break
            best = _Track(len(self.tracks), medium, p_dr.copy(), 0, k, self.odometer_m, key)
            self.tracks.append(best)
        best.p = (best.p * best.n + p_dr) / (best.n + 1)
        best.n += 1
        best.last_k = k
        best.s_last = self.odometer_m
        taken.add(best.tid)
        return self.offset + best.tid, self.offset + best.object_key


def label_detection(
    sensor: DetectionSensor,
    true_idx: int,
    part_classes: list[str],
    inst_desc: FloatArray,
    rng: np.random.Generator,
) -> tuple[int, FloatArray]:
    """Class id and descriptor through the Tier-1 label model of ``sensor``."""
    dim = inst_desc.shape[1] if inst_desc.ndim == 2 else 0
    if true_idx < 0:
        desc = (
            normalize(rng.normal(size=dim)) if sensor.semantic_noise > 0 and dim else np.zeros(dim)
        )
        return 0, desc
    true_cls = class_id(part_classes[true_idx])
    if sensor.label_correct_prob > 0 and rng.random() < sensor.label_correct_prob:
        cls = true_cls
    elif sensor.label_correct_prob > 0 and not sensor.label_confusion_to_unknown:
        cls = int(rng.integers(1, len(CLASS_NAMES)))
    else:
        cls = 0
    if sensor.semantic_noise > 0 and dim > 0:
        desc = normalize(inst_desc[true_idx] + sensor.semantic_noise * rng.normal(size=dim))
    else:
        desc = np.zeros(dim)
    return cls, desc


def sweep_window(times: FloatArray, k: int, sweep_s: float) -> tuple[float, float]:
    """``(start, width)`` [rad] of the azimuths swept up to keyframe ``k``, from ``-π + start``."""
    t0 = times[k - 1] if k > 0 else times[0] - (times[1] - times[0] if len(times) > 1 else 1.0)
    rate = 2.0 * np.pi / sweep_s
    return rate * (t0 - times[0]), rate * (times[k] - t0)


def swept_sector(
    spec: RaySensorSpec,
    scan: NDArray | SonarImage,
    times: FloatArray,
    k: int,
    sweep_s: float,
    margin_rad: float = 0.0,
) -> NDArray | SonarImage:
    """The part of a full-turn scan that a mechanical scanner swept up to keyframe ``k``.

    The head turns at ``2π / sweep_s`` [rad/s] from azimuth ``-π`` at ``times[0]``. A range
    scan (``(v, h)`` ranges) is returned with the columns outside the azimuths swept since
    keyframe ``k - 1`` set to ``inf`` (no return). A sonar image is cut to those columns, widened
    by ``margin_rad`` on each side, in sweep order (contiguous across the ±π seam; the azimuths
    are unwrapped, ascending). The vehicle's motion during the sweep is neglected: a keyframe
    lasts one second, the BlueROV2 moves 0.5 m in it.
    """
    start, width = sweep_window(times, k, sweep_s)
    if isinstance(scan, SonarImage):
        lo = start - margin_rad
        rel = np.mod(scan.azimuth_rad + np.pi - lo, 2.0 * np.pi)
        cols = np.flatnonzero(rel < width + 2.0 * margin_rad)
        cols = cols[np.argsort(rel[cols], kind="stable")]
        return SonarImage(scan.db[:, cols], scan.range_m, lo - np.pi + rel[cols])
    az, _ = ray_angles(spec)
    keep = np.mod(az + np.pi - start, 2.0 * np.pi) < width
    out = np.array(scan, dtype=np.float64, copy=True)
    out[:, ~keep] = np.inf
    return out


def nearest_part(world, medium: Medium, p_world: FloatArray, max_dist_m: float) -> int:
    """Index of the ground-truth part a detection measures (evaluation / labels only).

    The detection's horizontal position must lie within ``max_dist_m`` of the
    part's **centre**; otherwise it is ``-1``: a spurious detection, or a
    fragment of an extended object whose centre it does not measure. The
    centralized oracle drops such detections; Avatar agents keep them as
    front-end errors.
    """
    best, best_d = -1, max_dist_m
    for i, part in enumerate(world.parts):
        if part.medium != medium:
            continue
        d = float(np.hypot(p_world[0] - part.position[0], p_world[1] - part.position[1]))
        if d < best_d:
            best, best_d = i, d
    return best


def detections_for_agent(
    agent_data,
    sensor_data: dict[str, NDArray],
    specs: dict[str, RaySensorSpec],
    world,
    inst_desc: FloatArray,
    id_offset: int,
    params: FrontEndParams,
    rng: np.random.Generator,
) -> tuple[list[list[Detection]], dict]:
    """Detections per keyframe for one agent, and front-end statistics.

    ``agent_data`` is the Tier-1 :class:`~avatar.sim.measurements.AgentData`
    of the same seed (its odometry drives the tracker, its ground truth is
    used only to label detections for evaluation and the semantic oracle).
    """
    part_classes = [p.class_name for p in world.parts]
    tracker = Tracker(id_offset, params)
    noise = agent_data.config.odometry_noise
    sonar = any(spec.kind == "sonar_image" for spec in specs.values())
    ekf = EkfTracker(
        id_offset,
        params.ekf_sonar if sonar and params.ekf_sonar is not None else params.ekf,
        PlatformPrior(
            noise.yaw_bias_std_rad_per_m if heading_bias_modelled(agent_data.config) else 0.0,
            noise.scale_bias_std,
            noise.yaw_scale_bias_std,
        ),
    )
    dr = np.array([0.0, 0.0, 0.0, 0.0])
    n_kf = len(agent_data.keyframes)
    out: list[list[Detection]] = []
    stats = {"clusters": 0, "matched": 0, "spurious": 0, "wrong_track": 0}
    track_truth: dict[int, int] = {}
    n_clutter = 0
    for k in range(n_kf):
        kf = agent_data.keyframes[k]
        if k > 0 and kf.odom is not None:
            dr = compose(dr, kf.odom)
            tracker.step(float(np.linalg.norm(kf.odom[:3])))
            ekf.predict(kf.odom, kf.odom_sigmas)
        z_a = kf.abs_z if kf.abs_z is not None else float(agent_data.gt[0, 2] + dr[2])
        pose_dr = np.array([dr[0], dr[1], z_a, dr[3]])
        gt_pose = agent_data.gt[k]
        items = []  # (sensor, medium, cluster, sigmas, true part)
        for sname, spec in specs.items():
            if sname in params.ignore_sensors:
                continue
            sensor = SENSOR_LIBRARY[sname]
            medium = sensor.target_medium
            scan = sensor_data[sname][k]
            in_sector = None
            if sname == "ping360" and params.ping360_sweep_s > 0.0:
                sweep = params.ping360_sweep_s
                margin = params.ping360_margin_rad if isinstance(scan, SonarImage) else 0.0
                if isinstance(scan, SonarImage) and params.ping360_turn_context:
                    start, width = sweep_window(agent_data.times, k, sweep)
                    margin = np.pi - 0.5 * width  # the whole turn, the sector in the middle

                    def in_sector(c, start=start, width=width):
                        az = np.arctan2(c.p_body[1], c.p_body[0])
                        return np.mod(az + np.pi - start, 2.0 * np.pi) < width

                scan = swept_sector(spec, scan, agent_data.times, k, sweep, margin)
            for c in segment(spec, scan, z_a, world.seabed_z, params, rng):
                if c.range_m > sensor.max_range_m or c.range_m < sensor.min_range_m:
                    continue
                if in_sector is not None and not in_sector(c):
                    continue
                stats["clusters"] += 1
                sig = sensor.sigmas(c.range_m).copy()
                if spec.kind == "sonar_image":
                    sig[:2] = np.hypot(sig[:2], params.sonar_image.centre_sigma_m)
                if c.one_sided:
                    sig[:2] += params.one_sided_sigma_frac * float(c.footprint[0])
                if c.z_truncated and spec.kind != "sonar":
                    sig[2] = max(sig[2], params.truncated_sigma_z_m)
                p_w = transform_points(gt_pose, c.p_body)
                true_idx = nearest_part(world, medium, p_w, params.label_match_m)
                stats["matched" if true_idx >= 0 else "spurious"] += 1
                items.append((sensor, medium, c, sig, true_idx))
        p_dr = (
            np.array([transform_points(pose_dr, it[2].p_body) for it in items])
            if items
            else np.zeros((0, 3))
        )
        ids: list[tuple[int, int] | None]
        if params.tracking == "oracle":
            ids = []
            for it in items:
                if it[4] >= 0:
                    obj = world.parts[it[4]].object_id
                    ids.append((id_offset + it[4], id_offset + len(world.parts) + obj))
                else:
                    cid = id_offset + 2 * TRACK_SPAN + n_clutter
                    ids.append((cid, cid))  # one-off: its own object
                    n_clutter += 1
        elif params.tracking == "nn":
            taken: set[int] = set()
            ids = [
                tracker.assign(k, it[1], p_dr[i], float(it[3][0]), taken, drift=True)
                for i, it in enumerate(items)
            ]
        elif params.tracking in ("ekf", "ekf_truth"):
            ids = ekf.associate(
                [it[1] for it in items],
                np.array([it[2].p_body for it in items]).reshape(-1, 3),
                np.array([it[3] for it in items]).reshape(-1, 3),
                # "ekf_truth" (diagnostic upper bound): ground-truth identities
                truth=[it[4] for it in items] if params.tracking == "ekf_truth" else None,
            )
        elif params.tracking == "registration":
            ids = tracker.associate(
                k, [it[1] for it in items], p_dr, np.array([it[3][0] for it in items])
            )
        else:
            raise ValueError(f"unknown tracking mode {params.tracking!r}")
        dets: list[Detection] = []
        for (sensor, medium, c, sig, true_idx), tid_okey in zip(items, ids, strict=True):
            if tid_okey is None:  # ambiguous association: no detection is emitted
                continue
            tid, okey = tid_okey
            first = track_truth.setdefault(tid, true_idx)
            if first != true_idx:
                stats["wrong_track"] += 1
            cls, desc = label_detection(sensor, true_idx, part_classes, inst_desc, rng)
            ext = np.array([c.footprint[0], c.footprint[1], c.height_m])
            dets.append(
                Detection(
                    part_index=tid,
                    medium=medium,
                    modality=LandmarkFlags(sensor.modality),
                    p_body=c.p_body,
                    sigmas=sig,
                    extent=ext,
                    class_id=cls,
                    descriptor=desc,
                    true_part_index=true_idx,
                    object_key=okey,
                )
            )
        out.append(dets)
    if params.tracking in ("ekf", "ekf_truth"):
        # Detections are released once their landmark has proved static (a delay of
        # ``confirm_frames`` keyframes online; applied after the pass here).
        n_before = sum(len(d) for d in out)
        out = [[d for d in dets if ekf.released(d.part_index - id_offset)] for dets in out]
        stats["unreleased"] = n_before - sum(len(d) for d in out)
        status = ekf._status[: ekf.n_landmarks]
        stats["tracks"] = sum(ekf.released(j) for j in range(ekf.n_landmarks))
        stats["landmarks_confirmed"] = int(np.sum(status == CONFIRMED))
        stats["landmarks_bad"] = int(np.sum(status == BAD))
        stats["registrations"] = ekf.n_updates
        stats["dropped_ambiguous"] = ekf.n_dropped
    else:
        stats["tracks"] = len(tracker.tracks)
        stats["registrations"] = tracker.n_registrations
        stats["dropped_ambiguous"] = tracker.n_dropped
    stats["ambiguous_registrations"] = tracker.n_ambiguous
    return out, stats


__all__ = [
    "Cluster",
    "FrontEndParams",
    "Tracker",
    "detections_for_agent",
    "label_detection",
    "nearest_part",
    "segment",
    "swept_sector",
]
