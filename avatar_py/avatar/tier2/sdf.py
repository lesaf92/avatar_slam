"""Gazebo Harmonic world and bridge configuration from a Tier-1 scenario.

The world is written from the same :class:`~avatar.sim.scenarios.Scenario`
(and hence the same seed) as the Tier-1 run, so both tiers observe identical
structures, trajectories, and odometry (ADR-0003 parity requirement).

Geometry (world ENU, metres, ``z = 0`` on the waterline):

* every :class:`~avatar.sim.world.Structure` becomes a static model; round
  classes (piles, buoys, bollards, poles, trees) are vertical cylinders of
  diameter ``footprint[0]``, the other classes are axis-aligned boxes;
* land is a box ``x < shoreline_x`` from the seabed to ``land_z`` (its
  seaward face is the quay wall), the seabed a box below ``seabed_z``;
* the water surface has **no** visual: range sensors are ray-cast against
  visuals, and medium gating (optical returns above the waterline, acoustic
  returns below) is applied by the front-end (:mod:`avatar.tier2.frontend`).

Vehicles are **kinematic sensor rigs**: one model per SLAM agent, without
dynamics, moved to its ground-truth pose at every keyframe by the recorder.
DAVE, PX4 SITL and ``clearpath_gz`` are not used (not installable on the
development machine without root; see ``docs/LOG.md``), so vehicle dynamics
and hydrodynamics are out of scope for Tier 2 v0. The Gemini 720s is emulated
by a ray-cast "sonar proxy" (``gpu_lidar`` with the sonar's fan and range);
the front-end discards its elevation, as an imaging sonar would.

Sensors are mounted at the rig origin (the Tier-1 sensor models also place
sensors at the agent pose), pitched by ``pitch_rad`` (positive = down).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from xml.sax.saxutils import escape

import numpy as np

from avatar.sim.scenarios import Scenario
from avatar.sim.world import Structure

ROUND_CLASSES = frozenset({"pile", "buoy", "bollard", "light_pole", "tree"})

CLASS_COLORS: dict[str, tuple[float, float, float]] = {
    "pile": (0.45, 0.35, 0.25),
    "hull": (0.8, 0.8, 0.85),
    "buoy": (0.9, 0.5, 0.1),
    "mooring_block": (0.4, 0.4, 0.4),
    "rock": (0.35, 0.3, 0.25),
    "pipeline": (0.2, 0.2, 0.2),
    "bollard": (0.9, 0.8, 0.1),
    "container": (0.1, 0.4, 0.7),
    "crane_leg": (0.9, 0.3, 0.1),
    "light_pole": (0.6, 0.6, 0.6),
    "tree": (0.2, 0.5, 0.2),
}


@dataclass(frozen=True)
class RaySensorSpec:
    """Gazebo emulation of one Tier-1 sensor (``avatar.sim.sensors.SENSOR_LIBRARY`` key).

    Attributes
    ----------
    kind
        ``"lidar"`` (``gpu_lidar``), ``"depth"`` (``depth_camera``),
        ``"sonar"`` (``gpu_lidar`` used as a ray-cast imaging-sonar proxy) or
        ``"sonar_image"`` (a range-azimuth intensity image of DAVE's multibeam sonar,
        see :class:`DaveSonar`; ``h_samples`` beams, ``v_samples`` range bins).
    h_samples, v_samples
        Rays (or pixels) per row / column.
    h_fov_rad, v_fov_rad
        Horizontal / vertical field of view [rad], centred on the boresight.
    min_range_m, max_range_m
        Raw sensor range limits [m] (the front-end applies object-level limits).
    pitch_rad
        Mount pitch [rad], positive tilts the boresight **down**.
    """

    kind: str
    h_samples: int
    v_samples: int
    h_fov_rad: float
    v_fov_rad: float
    min_range_m: float
    max_range_m: float
    pitch_rad: float = 0.0


# Values from docs/hardware.md. Resolutions are reduced where the object-level
# front-end does not need the full rate (documented in docs/LOG.md).
GZ_SENSORS: dict[str, RaySensorSpec] = {
    # Velodyne VLP-16: 16 channels, 360° x 30°, 100 m. 0.4° horizontal step.
    "vlp16": RaySensorSpec("lidar", 900, 16, 2 * np.pi, np.deg2rad(30.0), 0.5, 100.0),
    # RealSense D435i depth: 87° x 58°, 0.3-10 m (depth image 212 x 128).
    "d435i": RaySensorSpec("depth", 212, 128, np.deg2rad(87.0), np.deg2rad(58.0), 0.3, 10.0),
    "d435i_down30": RaySensorSpec(
        "depth", 212, 128, np.deg2rad(87.0), np.deg2rad(58.0), 0.3, 10.0, np.deg2rad(30.0)
    ),
    # Tritech Micron Gemini 720s: 90° horizontal, 128 beams, 50 m. Vertical
    # aperture 20° (datasheet, docs/hardware.md) sampled by 16 rays per beam.
    "gemini_720s": RaySensorSpec("sonar", 128, 16, np.deg2rad(90.0), np.deg2rad(20.0), 0.5, 50.0),
    # Blue Robotics Ping360 (optional, D14): 360°, 2° x 25° beam, 0.9° steps, 0.75-50 m. A
    # mechanical scanner: the whole turn is recorded at every keyframe and the front-end keeps
    # the sector swept since the previous one (FrontEndParams.ping360_sweep_s).
    "ping360": RaySensorSpec("sonar", 400, 16, 2 * np.pi, np.deg2rad(25.0), 0.75, 50.0),
}


def sensor_topic(agent_name: str, sensor_name: str) -> str:
    """Gazebo/ROS topic of a rig sensor (conventions §7 namespace)."""
    return f"/avatar/{agent_name}/{sensor_name}"


def rig_sensors(scenario: Scenario) -> dict[str, list[str]]:
    """SLAM agent name → names of its sensors that Tier 2 emulates."""
    out: dict[str, list[str]] = {}
    for a in scenario.agents:
        if a.role != "slam":
            continue
        out[a.name] = [s for s in a.sensors if s in GZ_SENSORS]
    return out


def _f(v: float) -> str:
    return f"{float(v):.4f}"


def _structure_model(s: Structure, z_max: float | None = None) -> str:
    """Static model of a structure; ``z_max`` clips its top (the acoustic world stops at 0)."""
    z_top = s.z_max if z_max is None else z_max
    height = z_top - s.z_min
    zc = 0.5 * (s.z_min + z_top)
    if s.class_name in ROUND_CLASSES:
        geom = f"<cylinder><radius>{_f(0.5 * s.footprint[0])}</radius><length>{_f(height)}</length></cylinder>"
    else:
        geom = f"<box><size>{_f(s.footprint[0])} {_f(s.footprint[1])} {_f(height)}</size></box>"
    r, g, b = CLASS_COLORS.get(s.class_name, (0.5, 0.5, 0.5))
    return f"""
    <model name="obj_{s.object_id}_{escape(s.class_name)}">
      <static>true</static>
      <pose>{_f(s.center_xy[0])} {_f(s.center_xy[1])} {_f(zc)} 0 0 0</pose>
      <link name="link"><visual name="v"><geometry>{geom}</geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual></link>
    </model>"""


def _box_model(name: str, center, size, rgb) -> str:
    r, g, b = rgb
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{_f(center[0])} {_f(center[1])} {_f(center[2])} 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>{_f(size[0])} {_f(size[1])} {_f(size[2])}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual></link>
    </model>"""


def _sensor_xml(agent: str, sensor: str, spec: RaySensorSpec) -> str:
    topic = sensor_topic(agent, sensor)
    pose = f"0 0 0 0 {_f(spec.pitch_rad)} 0"
    if spec.kind == "depth":
        return f"""
        <sensor name="{sensor}" type="depth_camera">
          <pose>{pose}</pose><always_on>1</always_on><update_rate>1000</update_rate>
          <topic>{topic}</topic>
          <camera><horizontal_fov>{_f(spec.h_fov_rad)}</horizontal_fov>
            <image><width>{spec.h_samples}</width><height>{spec.v_samples}</height><format>R_FLOAT32</format></image>
            <clip><near>{_f(spec.min_range_m)}</near><far>{_f(spec.max_range_m)}</far></clip>
          </camera>
        </sensor>"""
    h, v = 0.5 * spec.h_fov_rad, 0.5 * spec.v_fov_rad
    return f"""
        <sensor name="{sensor}" type="gpu_lidar">
          <pose>{pose}</pose><always_on>1</always_on><update_rate>1000</update_rate>
          <topic>{topic}</topic>
          <lidar>
            <scan>
              <horizontal><samples>{spec.h_samples}</samples><resolution>1</resolution>
                <min_angle>{_f(-h)}</min_angle><max_angle>{_f(h)}</max_angle></horizontal>
              <vertical><samples>{spec.v_samples}</samples><resolution>1</resolution>
                <min_angle>{_f(-v)}</min_angle><max_angle>{_f(v)}</max_angle></vertical>
            </scan>
            <range><min>{_f(spec.min_range_m)}</min><max>{_f(spec.max_range_m)}</max><resolution>0.01</resolution></range>
          </lidar>
        </sensor>"""


def _rig_model(name: str, sensors: list[str], pose0) -> str:
    body = "".join(_sensor_xml(name, s, GZ_SENSORS[s]) for s in sensors)
    x, y, z, yaw = (float(v) for v in pose0)
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>
      <link name="base">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>{body}
      </link>
    </model>"""


def world_sdf(scenario: Scenario, initial_poses: dict[str, np.ndarray], world_name: str) -> str:
    """SDF (1.9) text of the Tier-2 world for ``scenario``.

    Parameters
    ----------
    initial_poses
        Agent name → initial ``[x, y, z, yaw]`` in world ENU (only SLAM agents
        with emulated sensors get a rig).
    world_name
        Gazebo world name (used in service names such as ``/world/<name>/set_pose``).
    """
    w = scenario.world
    x0, x1, y0, y1 = w.bounds_xy
    rigs = rig_sensors(scenario)
    models = [_structure_model(s) for s in w.structures]
    land_w = w.shoreline_x - x0
    land_h = w.land_z - w.seabed_z
    models.append(
        _box_model(
            "land",
            (x0 + 0.5 * land_w, 0.5 * (y0 + y1), w.seabed_z + 0.5 * land_h),
            (land_w, y1 - y0, land_h),
            (0.55, 0.55, 0.5),
        )
    )
    sea_w = x1 - w.shoreline_x
    models.append(
        _box_model(
            "seabed",
            (w.shoreline_x + 0.5 * sea_w, 0.5 * (y0 + y1), w.seabed_z - 0.5),
            (sea_w, y1 - y0, 1.0),
            (0.6, 0.55, 0.4),
        )
    )
    for name, sensors in rigs.items():
        if sensors:
            models.append(_rig_model(name, sensors, initial_poses[name]))
    return f"""<?xml version="1.0"?>
<!-- Generated by avatar.tier2.sdf from scenario '{escape(scenario.name)}'. Do not edit. -->
<sdf version="1.9">
  <world name="{escape(world_name)}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -0.9</direction></light>{"".join(models)}
  </world>
</sdf>
"""


# --- DAVE multibeam sonar (ADR-0008, docs/LOG.md L34) ---------------------------------------

SOUND_SPEED_M_S = 1500.0
SONAR_DB_MIN = -20.0  # dB mapped to 0 in a recorded sonar image (uint8)
SONAR_DB_MAX = 100.0  # dB mapped to 255
# A frame at every 1 ms iteration; the recorder keeps the one of the second iteration, which is
# strictly after the pose update. At 500 Hz the frame fell on the first or the second iteration
# depending on the sensor's phase, and was one keyframe stale in half of the recordings (LOG L35).
SONAR_UPDATE_RATE_HZ = 1000.0
SONAR_FRAMES_PER_KEYFRAME = 2
SONAR_RANGE_POOL = 4  # range bins averaged (in power) into one stored bin


@dataclass(frozen=True)
class DaveSonar:
    """Parameters of one imaging sonar emulated by DAVE's CUDA ``multibeam_sonar``.

    The raw image has ``beams`` columns and about ``2 B r_max / c`` range rows (DAVE's
    ``nFreq``, checked by the recorder); the recording stores ``SONAR_RANGE_POOL`` rows
    averaged in power as one.
    """

    beams: int
    v_rays: int
    h_fov_rad: float
    v_fov_rad: float
    min_range_m: float
    max_range_m: float
    frequency_hz: float
    bandwidth_hz: float
    source_level_db: float = 220.0
    sensor_gain: float = 0.02

    @property
    def image_beams(self) -> int:
        """Columns of the image: DAVE gives one more than the number of beams."""
        return self.beams + 1

    @property
    def raw_range_bins(self) -> int:
        return math.ceil(2.0 * self.bandwidth_hz * self.max_range_m / SOUND_SPEED_M_S)

    @property
    def range_bins(self) -> int:
        """Rows of a stored image."""
        return self.raw_range_bins // SONAR_RANGE_POOL

    def image_spec(self) -> RaySensorSpec:
        """Spec of a stored image, for the front-end (``kind = "sonar_image"``)."""
        return RaySensorSpec(
            "sonar_image",
            self.image_beams,
            self.range_bins,
            self.h_fov_rad,
            self.v_fov_rad,
            self.min_range_m,
            self.max_range_m,
        )


# Tritech Micron Gemini 720s (docs/hardware.md): 720 kHz, 90 deg x 128 beams, 8 mm range
# resolution (bandwidth c / (2 * 8 mm)), 20 deg vertical aperture (datasheet, docs/hardware.md).
DAVE_SONARS: dict[str, DaveSonar] = {
    "gemini_720s": DaveSonar(128, 64, np.deg2rad(90.0), np.deg2rad(20.0), 0.5, 30.0, 720e3, 94e3),
}


def sonar_image_specs() -> dict[str, RaySensorSpec]:
    """Stored-image spec of every sensor that has a DAVE sonar emulation."""
    return {k: v.image_spec() for k, v in DAVE_SONARS.items()}


def sonar_rigs(scenario: Scenario) -> dict[str, list[str]]:
    """SLAM agent name -> its sensors that have a DAVE sonar emulation."""
    out: dict[str, list[str]] = {}
    for a in scenario.agents:
        names = [s for s in a.sensors if s in DAVE_SONARS]
        if a.role == "slam" and names:
            out[a.name] = names
    return out


def _dave_sensor_xml(agent: str, sensor: str, dave: DaveSonar) -> str:
    # DAVE illuminates twice the vertical ray angles of the SDF (measured with a flat seabed:
    # rays +-5 deg give an echo from 8 m / sin(10 deg); docs/LOG.md L34), so the SDF holds a
    # quarter of the vertical FOV on each side; the azimuth angles are taken as written.
    h, v = 0.5 * dave.h_fov_rad, 0.25 * dave.v_fov_rad
    return f"""
        <sensor name="{sensor}" type="custom" gz:type="multibeam_sonar">
          <always_on>true</always_on><update_rate>{SONAR_UPDATE_RATE_HZ:g}</update_rate>
          <topic>{sensor_topic(agent, sensor)}</topic><visualize>false</visualize>
          <gz:multibeam_sonar>
            <ray degrees="false"><scan>
              <horizontal><beams>{dave.beams}</beams><min_angle>{_f(-h)}</min_angle><max_angle>{_f(h)}</max_angle></horizontal>
              <vertical><rays>{dave.v_rays}</rays><min_angle>{_f(-v)}</min_angle><max_angle>{_f(v)}</max_angle></vertical>
            </scan><range><min>{_f(dave.min_range_m)}</min><max>{_f(dave.max_range_m)}</max></range></ray>
            <spec>
              <verticalFOV>{math.degrees(dave.v_fov_rad):.3f}</verticalFOV>
              <sonarFreq>{dave.frequency_hz:.6g}</sonarFreq>
              <bandwidth>{dave.bandwidth_hz:.6g}</bandwidth>
              <soundSpeed>{SOUND_SPEED_M_S:g}</soundSpeed>
              <sourceLevel>{dave.source_level_db:g}</sourceLevel>
              <maxDistance>{_f(dave.max_range_m)}</maxDistance>
              <raySkips>10</raySkips>
              <sensorGain>{dave.sensor_gain:g}</sensorGain>
              <blazingSonarImage>true</blazingSonarImage>
              <writeLog>false</writeLog><debugFlag>false</debugFlag>
              <writeFrameInterval>5</writeFrameInterval>
              <pointCloudTopicName>point_cloud</pointCloudTopicName>
              <sonarImageRawTopicName>sonar_image_raw</sonarImageRawTopicName>
              <sonarImageTopicName>sonar_image</sonarImageTopicName>
              <frameName>{agent}/{sensor}</frameName>
            </spec>
          </gz:multibeam_sonar>
        </sensor>"""


def _sonar_rig_model(name: str, sensors: list[str], pose0) -> str:
    body = "".join(_dave_sensor_xml(name, s, DAVE_SONARS[s]) for s in sensors)
    x, y, z, yaw = (float(v) for v in pose0)
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}</pose>
      <link name="base">
        <gravity>false</gravity>
        <inertial><mass>1.0</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>{body}
      </link>
    </model>"""


def sonar_world_sdf(
    scenario: Scenario, initial_poses: dict[str, np.ndarray], world_name: str
) -> str:
    """SDF (1.9) text of the **acoustic** world for the DAVE sonar pass.

    Sound does not propagate in air and DAVE's sonar has no water surface, so the world holds
    only what is below the waterline: structures clipped at ``z = 0`` (those entirely above
    water are left out), the quay face (land up to ``z = 0``) and the seabed. Rigs carry the
    DAVE sonar of every sensor in :data:`DAVE_SONARS`, at the rig origin looking along the
    body ``x`` axis (level), as the proxy does. Same structures, seed and poses as
    :func:`world_sdf`, so a sonar recording pairs with the range-data recording of the run.
    """
    w = scenario.world
    x0, x1, y0, y1 = w.bounds_xy
    models = [_structure_model(s, z_max=min(s.z_max, 0.0)) for s in w.structures if s.z_min < 0.0]
    land_w = w.shoreline_x - x0
    land_h = 0.0 - w.seabed_z
    models.append(
        _box_model(
            "land",
            (x0 + 0.5 * land_w, 0.5 * (y0 + y1), w.seabed_z + 0.5 * land_h),
            (land_w, y1 - y0, land_h),
            (0.55, 0.55, 0.5),
        )
    )
    sea_w = x1 - w.shoreline_x
    models.append(
        _box_model(
            "seabed",
            (w.shoreline_x + 0.5 * sea_w, 0.5 * (y0 + y1), w.seabed_z - 0.5),
            (sea_w, y1 - y0, 1.0),
            (0.6, 0.55, 0.4),
        )
    )
    for name, sensors in sonar_rigs(scenario).items():
        models.append(_sonar_rig_model(name, sensors, initial_poses[name]))
    return f"""<?xml version="1.0"?>
<!-- Generated by avatar.tier2.sdf.sonar_world_sdf from scenario '{escape(scenario.name)}'. Do not edit. -->
<sdf version="1.9">
  <world name="{escape(world_name)}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 0</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="multibeam_sonar_system" name="custom::MultibeamSonarSystem"/>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.1 0.2 0.3 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.3 0.2 -0.9</direction></light>{"".join(models)}
  </world>
</sdf>
"""


def bridge_yaml(scenario: Scenario, world_name: str) -> str:
    """``ros_gz_bridge`` parameter file: sensor topics (gz → ROS) and world services."""
    lines = []
    for name, sensors in rig_sensors(scenario).items():
        for s in sensors:
            spec = GZ_SENSORS[s]
            topic = sensor_topic(name, s)
            if spec.kind == "depth":
                ros_t, gz_t = "sensor_msgs/msg/Image", "gz.msgs.Image"
            else:
                ros_t, gz_t = "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"
            lines.append(
                f'- ros_topic_name: "{topic}"\n  gz_topic_name: "{topic}"\n'
                f'  ros_type_name: "{ros_t}"\n  gz_type_name: "{gz_t}"\n  direction: GZ_TO_ROS'
            )
    lines.append(
        '- ros_topic_name: "/clock"\n  gz_topic_name: "/clock"\n'
        '  ros_type_name: "rosgraph_msgs/msg/Clock"\n  gz_type_name: "gz.msgs.Clock"\n'
        "  direction: GZ_TO_ROS"
    )
    for srv, ros_t, req, rep in (
        ("set_pose", "ros_gz_interfaces/srv/SetEntityPose", "gz.msgs.Pose", "gz.msgs.Boolean"),
        (
            "control",
            "ros_gz_interfaces/srv/ControlWorld",
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
        ),
    ):
        lines.append(
            f'- service_name: "/world/{world_name}/{srv}"\n  ros_type_name: "{ros_t}"\n'
            f'  gz_req_type_name: "{req}"\n  gz_rep_type_name: "{rep}"'
        )
    return "\n".join(lines) + "\n"
