"""Abstract exteroceptive sensor models at the landmark-part level.

Tier-1 simulation (ADR-0003) does not render images or point clouds. Each
sensor directly returns *detections* of landmark parts: a 3-D position in the
agent's gravity-aligned body frame (roll/pitch compensated, conventions §4),
a measured extent, a class label and an optional semantic descriptor. The
error models capture the properties that matter for cross-medium SLAM:

* **Medium gating.** Optical sensors (LiDAR, camera) work only when the agent
  is above water and see only ``ABOVE`` parts. Sonar works only in water and
  sees only ``BELOW`` parts.
* **Imaging-sonar elevation ambiguity.** Vertical σ grows with range
  (≈ range · tan(elevation aperture / 2)).
* **Turbidity.** Underwater cameras have a range of a few metres.
* **Semantics.** Only cameras produce class labels reliably and open-vocabulary
  descriptors. Sonar labels are coarse and often ``unknown``. LiDAR is geometric only.

* **Vertical field of view.** A part is detected only if its vertical span
  intersects the sensor's elevation fan (e.g. VLP-16 ±15°, D435i 58° around its
  mount pitch). Close, low objects fall out of a LiDAR's fan.

The library holds generic models (``lidar``, ``camera``, ``sonar``, …) used by
the v0 ``harbor`` scenario, and **reference-fleet models** named after the real
devices (``vlp16``, ``d435i``, ``d435i_down30``, ``gemini_720s``,
``bluerov2_camera``). Their parameters and sources are in ``docs/hardware.md``.

Occlusion is **not** modelled in v0 (task T-S1-03).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import rot_z
from avatar.semantics import CLASS_NAMES, class_id, normalize
from avatar.types import WATER_SURFACE_TOL_M, LandmarkFlags, Medium


@dataclass(frozen=True)
class DetectionSensor:
    """Landmark-part detector with range-dependent Gaussian noise."""

    name: str
    modality: LandmarkFlags  # CAMERA, LIDAR or SONAR
    target_medium: Medium
    max_range_m: float
    min_range_m: float = 0.5
    hfov_rad: float = 2 * np.pi  # horizontal field of view centred on heading
    vfov_rad: float = np.pi  # vertical field of view (π = no vertical limit)
    mount_pitch_rad: float = 0.0  # boresight elevation (negative = tilted down)
    sigma_h_base_m: float = 0.03
    sigma_h_per_m: float = 0.002
    sigma_h_per_m2: float = 0.0  # quadratic term (stereo depth error ∝ range²)
    sigma_z_base_m: float = 0.03
    sigma_z_per_m: float = 0.002
    sigma_z_per_m2: float = 0.0
    detection_prob: float = 0.95
    label_correct_prob: float = 0.0  # 0 → never labels (reports "unknown")
    label_confusion_to_unknown: bool = True  # wrong labels become "unknown" (vs random class)
    extent_rel_noise: float = 0.1
    semantic_noise: float = 0.0  # >0 enables descriptors (optical only)

    def can_operate(self, agent_z: float) -> bool:
        """Whether the sensor works at the agent's current height (medium gating)."""
        if self.modality == LandmarkFlags.SONAR:
            return agent_z <= WATER_SURFACE_TOL_M
        if self.target_medium == Medium.BELOW:  # underwater camera
            return agent_z < -WATER_SURFACE_TOL_M
        return agent_z >= -WATER_SURFACE_TOL_M  # optical sensors above water

    def sigmas(self, range_m: float) -> NDArray[np.float64]:
        """Nominal 1-σ ``(x, y, z)`` [m] of a detection at ``range_m``."""
        r = range_m
        sh = self.sigma_h_base_m + self.sigma_h_per_m * r + self.sigma_h_per_m2 * r * r
        sz = self.sigma_z_base_m + self.sigma_z_per_m * r + self.sigma_z_per_m2 * r * r
        return np.array([sh, sh, sz])


# Sensor library used by scenarios (names are referenced by AgentConfig.sensors).
SENSOR_LIBRARY: dict[str, DetectionSensor] = {
    "lidar": DetectionSensor(
        "lidar",
        LandmarkFlags.LIDAR,
        Medium.ABOVE,
        max_range_m=40.0,
        sigma_h_base_m=0.05,
        sigma_h_per_m=0.002,
        sigma_z_base_m=0.05,
        sigma_z_per_m=0.002,
        detection_prob=0.95,
    ),
    "lidar_aerial": DetectionSensor(
        "lidar_aerial",
        LandmarkFlags.LIDAR,
        Medium.ABOVE,
        max_range_m=50.0,
        sigma_h_base_m=0.05,
        sigma_h_per_m=0.003,
        sigma_z_base_m=0.05,
        sigma_z_per_m=0.003,
        detection_prob=0.9,
    ),
    "camera": DetectionSensor(
        "camera",
        LandmarkFlags.CAMERA,
        Medium.ABOVE,
        max_range_m=30.0,
        hfov_rad=np.deg2rad(100.0),
        sigma_h_base_m=0.1,
        sigma_h_per_m=0.02,
        sigma_z_base_m=0.1,
        sigma_z_per_m=0.02,
        detection_prob=0.85,
        label_correct_prob=0.9,
        label_confusion_to_unknown=False,
        semantic_noise=0.15,
    ),
    "sonar": DetectionSensor(  # forward-looking imaging sonar (e.g. 130° x 20° aperture)
        "sonar",
        LandmarkFlags.SONAR,
        Medium.BELOW,
        max_range_m=20.0,
        min_range_m=1.0,
        hfov_rad=np.deg2rad(130.0),
        sigma_h_base_m=0.1,
        sigma_h_per_m=0.005,
        sigma_z_base_m=0.2,
        sigma_z_per_m=0.09,
        detection_prob=0.6,
        label_correct_prob=0.5,
        extent_rel_noise=0.2,
    ),
    "camera_underwater": DetectionSensor(
        "camera_underwater",
        LandmarkFlags.CAMERA,
        Medium.BELOW,
        max_range_m=4.0,
        hfov_rad=np.deg2rad(90.0),
        sigma_h_base_m=0.05,
        sigma_h_per_m=0.02,
        sigma_z_base_m=0.05,
        sigma_z_per_m=0.02,
        detection_prob=0.7,
        label_correct_prob=0.8,
        label_confusion_to_unknown=False,
        semantic_noise=0.25,
    ),
    # ---- Reference fleet (docs/hardware.md). Object-level ranges are below the
    # ---- devices' raw ranges: detection/segmentation degrades long before the
    # ---- last return.
    "vlp16": DetectionSensor(  # Velodyne VLP-16: 100 m raw, ±3 cm, 360° × 30° (±15°)
        "vlp16",
        LandmarkFlags.LIDAR,
        Medium.ABOVE,
        max_range_m=40.0,
        min_range_m=1.0,
        vfov_rad=np.deg2rad(30.0),
        sigma_h_base_m=0.04,
        sigma_h_per_m=0.002,
        sigma_z_base_m=0.04,
        sigma_z_per_m=0.003,
        detection_prob=0.95,
    ),
    "d435i": DetectionSensor(  # RealSense D435i RGB-D, level mount: 87° × 58°, ideal 0.3-3 m
        "d435i",
        LandmarkFlags.CAMERA,
        Medium.ABOVE,
        max_range_m=6.0,
        min_range_m=0.3,
        hfov_rad=np.deg2rad(87.0),
        vfov_rad=np.deg2rad(58.0),
        sigma_h_base_m=0.03,
        sigma_h_per_m2=0.005,
        sigma_z_base_m=0.03,
        sigma_z_per_m2=0.005,
        detection_prob=0.85,
        label_correct_prob=0.9,
        label_confusion_to_unknown=False,
        semantic_noise=0.15,
    ),
    "d435i_down30": DetectionSensor(  # same camera on the UAV, pitched 30° down
        "d435i_down30",
        LandmarkFlags.CAMERA,
        Medium.ABOVE,
        max_range_m=6.0,
        min_range_m=0.3,
        hfov_rad=np.deg2rad(87.0),
        vfov_rad=np.deg2rad(58.0),
        mount_pitch_rad=np.deg2rad(-30.0),
        sigma_h_base_m=0.03,
        sigma_h_per_m2=0.005,
        sigma_z_base_m=0.03,
        sigma_z_per_m2=0.005,
        detection_prob=0.85,
        label_correct_prob=0.9,
        label_confusion_to_unknown=False,
        semantic_noise=0.15,
    ),
    "gemini_720s": DetectionSensor(  # Tritech Micron Gemini 720s: 90° H, 50 m raw, 128 beams
        "gemini_720s",
        LandmarkFlags.SONAR,
        Medium.BELOW,
        max_range_m=30.0,
        min_range_m=0.5,
        hfov_rad=np.deg2rad(90.0),
        vfov_rad=np.deg2rad(20.0),  # vertical aperture: UNVERIFIED (typical Gemini value)
        sigma_h_base_m=0.05,
        sigma_h_per_m=0.004,  # 0.7° beams, 8 mm range resolution
        sigma_z_base_m=0.1,
        sigma_z_per_m=0.1,  # elevation ambiguity ≈ r·tan(10°)/√3
        detection_prob=0.7,
        label_correct_prob=0.5,
        extent_rel_noise=0.2,
    ),
    "bluerov2_camera": DetectionSensor(  # BlueROV2 low-light HD camera, turbid harbour water
        "bluerov2_camera",
        LandmarkFlags.CAMERA,
        Medium.BELOW,
        max_range_m=3.0,
        hfov_rad=np.deg2rad(80.0),
        sigma_h_base_m=0.05,
        sigma_h_per_m=0.02,
        sigma_z_base_m=0.05,
        sigma_z_per_m=0.02,
        detection_prob=0.6,
        label_correct_prob=0.8,
        label_confusion_to_unknown=False,
        semantic_noise=0.25,
    ),
}


@dataclass(frozen=True, eq=False)
class Detection:
    """One detection of a landmark part at a keyframe."""

    part_index: int  # index into World.parts (ground truth, never leaves the agent)
    medium: Medium
    modality: LandmarkFlags
    p_body: NDArray[np.float64]  # measured position in gravity-aligned body frame [m]
    sigmas: NDArray[np.float64]  # nominal 1-σ (x, y, z) [m]
    extent: NDArray[np.float64]  # measured (footprint x, footprint y, height) [m]
    class_id: int
    descriptor: NDArray[np.float64]  # zeros if not provided


def detect(
    sensor: DetectionSensor,
    pose: NDArray[np.float64],
    part_positions: NDArray[np.float64],
    part_media: NDArray[np.int64],
    part_extents: NDArray[np.float64],
    part_classes: list[str],
    instance_descriptors: NDArray[np.float64],
    rng: np.random.Generator,
) -> list[Detection]:
    """Simulate all detections of ``sensor`` from ground-truth ``pose = [x, y, z, yaw]``."""
    if not sensor.can_operate(float(pose[2])) or len(part_positions) == 0:
        return []
    rel_world = part_positions - pose[:3]
    rel_body = rel_world @ rot_z(pose[3])  # R^T applied row-wise
    rng_m = np.linalg.norm(rel_body, axis=1)
    bearing = np.arctan2(rel_body[:, 1], rel_body[:, 0])
    visible = (
        (part_media == int(sensor.target_medium))
        & (rng_m <= sensor.max_range_m)
        & (rng_m >= sensor.min_range_m)
        & (np.abs(bearing) <= 0.5 * sensor.hfov_rad + 1e-12)
    )
    if sensor.vfov_rad < np.pi:
        # The part's vertical span [z_lo, z_hi] must intersect the elevation fan.
        d_h = np.maximum(np.hypot(rel_body[:, 0], rel_body[:, 1]), 1e-6)
        half = 0.5 * part_extents[:, 2]
        el_lo = np.arctan2(rel_body[:, 2] - half, d_h)
        el_hi = np.arctan2(rel_body[:, 2] + half, d_h)
        fan_lo = sensor.mount_pitch_rad - 0.5 * sensor.vfov_rad
        fan_hi = sensor.mount_pitch_rad + 0.5 * sensor.vfov_rad
        visible &= (el_hi >= fan_lo) & (el_lo <= fan_hi)
    out: list[Detection] = []
    dim = instance_descriptors.shape[1] if instance_descriptors.ndim == 2 else 0
    for idx in np.flatnonzero(visible):
        if rng.random() > sensor.detection_prob:
            continue
        sig = sensor.sigmas(float(rng_m[idx]))
        meas = rel_body[idx] + rng.normal(0.0, sig)
        ext = part_extents[idx] * (1.0 + sensor.extent_rel_noise * rng.normal(size=3))
        ext = np.maximum(ext, 0.05)
        true_cls = class_id(part_classes[idx])
        if sensor.label_correct_prob > 0 and rng.random() < sensor.label_correct_prob:
            cls = true_cls
        elif sensor.label_correct_prob > 0 and not sensor.label_confusion_to_unknown:
            cls = int(rng.integers(1, len(CLASS_NAMES)))
        else:
            cls = 0
        if sensor.semantic_noise > 0 and dim > 0:
            noise = sensor.semantic_noise * rng.normal(size=dim)
            desc = normalize(instance_descriptors[idx] + noise)
        else:
            desc = np.zeros(dim)
        out.append(
            Detection(
                part_index=int(idx),
                medium=sensor.target_medium,
                modality=sensor.modality,
                p_body=meas,
                sigmas=sig,
                extent=ext,
                class_id=cls,
                descriptor=desc,
            )
        )
    return out
