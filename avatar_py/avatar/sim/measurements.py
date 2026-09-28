"""Pre-generation of all measurements of a scenario.

Measurements are generated **once per seed**, independently of the estimator.
Every estimation mode (independent, decentralized, centralized) then consumes
the *same* noise realisation, which makes comparisons paired (PLAN §6).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from avatar.geometry import between
from avatar.semantics import class_id, class_prototypes, normalize
from avatar.sim.agents import AgentConfig, sample_trajectory
from avatar.sim.sensors import SENSOR_LIBRARY, Detection, detect
from avatar.sim.world import World

ABS_Z_SIGMA_M: dict[str, float] = {"depth": 0.05, "bar30": 0.02, "surface": 0.05, "baro": 0.3}


@dataclass(eq=False)
class KeyframeData:
    """Everything agent *i* measures at keyframe *k*."""

    t: float
    odom: NDArray[np.float64] | None  # noisy increment [dx, dy, dz, dyaw] from keyframe k-1
    odom_sigmas: NDArray[np.float64] | None  # nominal 1-σ reported to the estimator
    detections: list[Detection]
    abs_z: float | None
    abs_z_sigma: float | None


@dataclass(eq=False)
class AgentData:
    """Ground truth and measurements of one agent."""

    config: AgentConfig
    times: NDArray[np.float64]
    gt: NDArray[np.float64]  # (n, 4) ground-truth poses in world ENU
    keyframes: list[KeyframeData]
    yaw_bias_rad_per_m: float

    @property
    def T_world_from_local(self) -> NDArray[np.float64]:
        """Ground-truth transform from this agent's local map frame to world.

        The local frame is gravity-aligned, on the waterline datum, with its
        origin below/above the start position and x along the start heading.
        """
        x0, y0, _, yaw0 = self.gt[0]
        return np.array([x0, y0, 0.0, yaw0])


@dataclass(eq=False)
class SimData:
    """Measurements of all agents for one scenario and seed."""

    world: World
    agents: dict[int, AgentData]
    descriptor_dim: int
    instance_descriptors: NDArray[np.float64]  # per world part, (n_parts, D)
    dt_s: float
    duration_s: float


def generate_measurements(
    world: World,
    agents: list[AgentConfig],
    duration_s: float,
    dt_s: float,
    descriptor_dim: int,
    rng: np.random.Generator,
) -> SimData:
    """Simulate trajectories and all sensor measurements at keyframe rate ``1/dt_s``."""
    protos = class_prototypes(descriptor_dim)
    part_classes = [p.class_name for p in world.parts]
    part_extents = np.array([p.extent for p in world.parts]) if world.parts else np.zeros((0, 3))
    # Instance descriptor per *object* (both parts share it), derived from the class prototype.
    obj_desc: dict[int, NDArray[np.float64]] = {}
    inst = np.zeros((len(world.parts), descriptor_dim))
    for i, part in enumerate(world.parts):
        if descriptor_dim == 0:
            break
        if part.object_id not in obj_desc:
            proto = protos[class_id(part.class_name)]
            obj_desc[part.object_id] = normalize(proto + 0.3 * rng.normal(size=descriptor_dim))
        inst[i] = obj_desc[part.object_id]

    times = np.arange(0.0, duration_s + 1e-9, dt_s)
    data: dict[int, AgentData] = {}
    for cfg in agents:
        if cfg.agent_id in data:
            raise ValueError(f"duplicate agent_id {cfg.agent_id}")
        gt = sample_trajectory(cfg, times)
        noise = cfg.odometry_noise
        bias = float(rng.normal(0.0, noise.yaw_bias_std_rad_per_m))
        scale = 1.0 + float(rng.normal(0.0, noise.scale_bias_std)) if noise.scale_bias_std else 1.0
        sensors = [SENSOR_LIBRARY[name] for name in cfg.sensors]
        kfs: list[KeyframeData] = []
        for k, t in enumerate(times):
            odom = odom_sig = None
            if k > 0:
                true_inc = between(gt[k - 1], gt[k])
                dist = float(np.linalg.norm(true_inc[:3]))
                odom_sig = noise.sigmas(dist)
                odom = true_inc + rng.normal(0.0, odom_sig)
                odom[:3] = scale * odom[:3]
                odom[3] += bias * dist
            dets: list[Detection] = []
            for sensor in sensors:
                dets.extend(
                    detect(
                        sensor,
                        gt[k],
                        world.part_positions,
                        world.part_media,
                        part_extents,
                        part_classes,
                        inst,
                        rng,
                    )
                )
            abs_z = abs_sig = None
            if cfg.absolute_z is not None:
                abs_sig = ABS_Z_SIGMA_M[cfg.absolute_z]
                abs_z = float(gt[k, 2] + rng.normal(0.0, abs_sig))
            kfs.append(KeyframeData(float(t), odom, odom_sig, dets, abs_z, abs_sig))
        data[cfg.agent_id] = AgentData(cfg, times.copy(), gt, kfs, bias)
    return SimData(world, data, descriptor_dim, inst, dt_s, duration_s)
