# ADR-0007: Tier 2 v0 uses kinematic sensor rigs and a ray-cast sonar proxy

- **Status:** Accepted (v0; revisit when DAVE/PX4 can be installed)
- **Date:** 2026-09-29
- **Amends:** ADR-0003 (two-tier simulation), item 2
- **Amended by:** ADR-0008 (proposed): the sonar proxy is to be complemented by DAVE's multibeam sonar

## Context
ADR-0003 plans Tier 2 as one Gazebo Harmonic world with DAVE (underwater
vehicles, multibeam sonar, DVL), PX4 SITL (aerial) and a UGV. On the lab
machine available for this work (Ubuntu 24.04, ROS 2 Jazzy, RTX 3050) there is
no root access and no Docker socket, so DAVE, PX4 SITL and `clearpath_gz`
could not be installed. Gazebo Harmonic itself (gz-sim 8, ros_gz 1.0) could be
extracted from the ROS Jazzy vendor packages into a user directory
(`experiments/gazebo/README.md`), and its GPU ray-cast sensors run headless.

The paper's claims concern mapping and communication, not vehicle control.
What Tier 1 abstracts away is **perception geometry**: which parts are
visible (occlusion, fields of view, vertical fans), where a segmented cluster
puts an object's centre (partial views), sonar elevation loss, spurious
clusters, and intra-agent association.

## Decision
1. **Kinematic rigs.** Each SLAM agent is a massless model carrying its
   sensors, placed at its Tier-1 ground-truth pose at every keyframe
   (`set_pose` + two 1 ms steps). Odometry, depth and barometer come from the
   Tier-1 models with the same seed, so a Tier-1 and a Tier-2 run form a
   paired comparison in which only perception differs.
2. **Sensors.** VLP-16 and D435i are Gazebo `gpu_lidar` / `depth_camera`
   with the devices' fields of view; noise is added offline. The Gemini 720s
   is a **ray-cast proxy** (`gpu_lidar`, 128 beams x 16 elevation rays,
   90° x 20°, 50 m). Its per-ray elevation is used only to drop seabed and
   surface returns and is then discarded.
3. **Recorder in C++ over gz-transport**, not rclpy: the extracted
   `ros_gz_interfaces` Python type support is ABI-incompatible with the
   installed Fast-CDR (`symbol lookup error`), and upgrading Fast-CDR broke
   rclpy node creation. The recorder has no ROS dependency (AGENTS.md §3).
4. Results from Tier 2 are labelled "Tier 2 (Gazebo, kinematic rigs, sonar
   proxy)" everywhere; they are simulation results.

## Consequences
- Tier 2 tests perception geometry and the front-end, but not vehicle
  dynamics, sonar image formation (speckle, multipath, shadows), or
  DAVE's physics-based multibeam. Gate G1 on Tier 2 v0 is therefore a
  necessary, not a sufficient, check; it must be repeated with DAVE sonar
  images or real data (R2 in PLAN §7).
- The same recorder and front-end accept DAVE's multibeam output once
  available: only the sensor spec and the ray model change (T-S2-02/03 stay
  open for the DAVE/PX4/Clearpath rigs).

## Alternatives considered
- *Ask for root / Docker*: preferred long-term (T-I1-03); not available now.
- *Build DAVE and PX4 from source in user space*: DAVE's Jazzy branch needs
  many ROS packages missing here (each would need the same extraction), and
  PX4 SITL adds a toolchain; weeks of work for no change in the mapping
  claims.
- *Actors with scripted trajectories*: Gazebo actors do not carry sensors
  reliably.
