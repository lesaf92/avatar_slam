# Conventions: frames, units, identifiers

**Status:** contract v0 (changes require an ADR, see AGENTS.md §4.4).

These conventions let an aerial LiDAR point, a ground-robot camera detection,
and an underwater sonar return be combined without sign errors. Every module
in Python, C++, and ROS 2 follows them.

## 1. Units

SI everywhere: metres, seconds, radians, kilograms. Bandwidth is in bits per
second (`_bps`), payload sizes in bytes (`_B`). Angles in configuration files
may use degrees only when the key ends in `_deg`. Time stamps inside the
simulator are `float` seconds since mission start. On the wire they are
`uint32` milliseconds since the mission epoch (see the wire spec).

## 2. Vertical datum: the waterline

The **vertical datum is the mean water surface**: `z = 0` at the waterline,
with `z` positive **up**.

- Underwater points have `z < 0`. AUV depth `d` (pressure sensor, positive
  down) maps to `z = -d`.
- A point is **above** water if `z > 0` and **below** if `z < 0`.
- Tides and waves are modelled as a datum uncertainty (a `z` prior on
  inter-agent frame transforms), not as a moving datum. v0 assumes a static
  water level.

Why: the waterline is the one horizontal reference that every agent in a
harbour can observe or infer (pressure sensor, barometer plus take-off height,
USV hull, quay height). With it, all local frames share `z` and gravity, which
leaves only `(x, y, yaw)` unknown between agents (see §4).

## 3. Axes and handedness

| Frame kind | Convention | Notes |
|---|---|---|
| World / map / odom | **ENU** (x East, y North, z Up), right-handed | REP-103 |
| Robot body | **FLU** (x Forward, y Left, z Up) | REP-103 |
| Camera optical | z forward, x right, y down | REP-103 `_optical` suffix |
| Legacy marine (NED / FRD) | converted **at the driver boundary only** | `avatar::frames::ned_to_enu`, `avatar.geometry.ned_to_enu` |

No algorithmic code uses NED/FRD internally. Drivers and simulator bridges
that emit NED (for example ArduSub, some DVLs, and some underwater simulators)
must convert before publishing into the Avatar stack.

Conversions (vectors):

```
ENU from NED:  (x_e, y_e, z_e) = (y_n, x_n, -z_n)
FLU from FRD:  (x_f, y_f, z_f) = (x_r, -y_r, -z_r)
```

Yaw: `yaw_enu = pi/2 - yaw_ned`, wrapped to `(-pi, pi]`.

## 4. Frames used by the estimator

| Name | Symbol | Definition |
|---|---|---|
| World | `W` | Ground-truth ENU frame of a scenario (simulation / evaluation only) |
| Agent local map | `L_i` | Gravity-aligned, `z` on the waterline datum, origin at agent *i*'s start position `(x0, y0)`, x-axis along its start heading |
| Agent body | `B_i` | FLU body frame of agent *i* |
| Team frame | `L_a` | Local map frame of the **anchor agent** `a` (config), in which team-level results are expressed |

Because every `L_i` is gravity-aligned and on the same vertical datum, the
transform between two local maps is **4-DoF** `(x, y, z, yaw)`, with `z ≈ 0`.
A small `z` term is still estimated with a tight prior to absorb datum errors.

**Pose parameterisation (v0): 4-DoF** `[x, y, z, yaw]`. Roll and pitch are
observable from gravity (IMU), so front-ends deliver roll/pitch-compensated
("yaw-frame") measurements. This matches the 4-DoF pose-graph practice of
VINS-Mono (Qin et al., T-RO 2018). Full 6-DoF is a possible later extension
(task `T-B1-05`).

Notation in code and papers: `T_A_from_B` (Python) or `T_A_B` (C++) maps
coordinates expressed in `B` into `A`: `p_A = T_A_from_B * p_B`.

## 5. Media, domains, modalities

| Enum | Values (stable integer codes) |
|---|---|
| `Domain` | `AERIAL = 0`, `GROUND = 1`, `SURFACE = 2`, `UNDERWATER = 3` |
| `Medium` (of an observed landmark part) | `ABOVE` (z > 0 part), `BELOW` (z < 0 part) |
| `Modality` | `CAMERA`, `LIDAR`, `SONAR` (bit flags on the wire) |
| `LinkType` | `RF = 0`, `ACOUSTIC = 1` |

**Landmark parts.** A physical object that crosses the waterline, such as a
pier pile, a hull, or a buoy, is represented as up to **two landmarks**: its
above-water part and its below-water part. Each is a separate estimator
variable. They share `(x, y)` but not `z`, and are linked by a **coaxial**
constraint. The centroid of what a sensor sees depends on the medium it sees,
so the two parts differ in `z` by construction.

## 6. Identifiers

| ID | Type | Rule |
|---|---|---|
| Agent id | `uint8` (0–254; 255 reserved = broadcast) | Unique per mission, fixed in config |
| Agent name | string | `<domain>_<index>`: `uav_0`, `ugv_0`, `usv_0`, `auv_1` |
| Landmark id | `uint16` | **Private to the owning agent**, never derived from ground truth |
| Keyframe index | `uint32` | Per agent, monotonically increasing |
| Semantic class id | `uint8` | Index into `avatar.semantics.CLASS_NAMES` (0 = unknown) |

**Anti-leak rule.** In simulation, ground-truth object ids are remapped to
per-agent random private ids before anything leaves the agent. Inter-agent
data association must never compare ids across agents. Tests enforce this
(`test_no_ground_truth_leak`).

## 7. ROS 2 naming (for `ros2/` packages)

- Namespace per agent: `/<agent_name>/...` (e.g. `/auv_0/sonar/image`).
- TF: `<agent_name>/map`, `<agent_name>/odom`, `<agent_name>/base_link`; team
  frame `team_map` (alias of the anchor's map frame).
- Inter-agent topics: `/avatar/digest` (`avatar_msgs/AgentDigest`),
  `/avatar/packets/<link>` (`avatar_msgs/EncodedPacket`), `/avatar/alignment`
  (`avatar_msgs/FrameAlignment`).
- QoS: sensor streams `SensorDataQoS`; inter-agent `reliable` + `transient_local`
  (depth 10) on RF, and a best-effort gateway for acoustic.
