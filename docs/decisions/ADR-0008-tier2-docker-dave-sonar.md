# ADR-0008: Tier 2 v1 records in Docker and adds DAVE's multibeam sonar (Jazzy / Harmonic)

- **Status:** Proposed (2026-09-30); accept when Tier 2 is recorded with the sonar and G1 is repeated on it (T-S2-05)
- **Date:** 2026-09-30
- **Amends:** ADR-0007 (kinematic rigs and a ray-cast sonar proxy), item 2

## Context
ADR-0007 named the sonar proxy as the main limit of Tier 2 (risk R2): no speckle,
reverberation or acoustic shadows, so gate G1 there is necessary, not sufficient. Its
fix, DAVE's multibeam sonar, could not be installed on the old lab machine (no Docker
access, 21 GB of disk). The project moved to a machine with Docker, the NVIDIA container
toolkit, an RTX 4070 (8 GB) and 330 GB of free disk. What was found there, on
2026-09-30:

- **Gazebo Harmonic has no sonar sensor.** No file of `gz-sim`, `gz-sensors` or
  `gz-rendering` (`main`) mentions one.
- **DAVE is alive, under another name and layout.** The active repository is
  `IOES-Lab/dave` (branch `ros2`, pushed daily); `Field-Robotics-Lab/dave` is the old
  upstream (last push 2024-08-19). On 2026-09-09 (commit `ec1db10`) the `ros2` branch
  moved to **ROS 2 Lyrical and Gazebo Jetty**. Its parent, `d2121a5`
  (`d2121a5b4457361e60106aaa029b0a448977d70e`, 2026-09-08), is the last **Jazzy /
  Harmonic** commit and has a `.docker/jazzy.amd64.dockerfile`.
- **The sonar** (`gazebo/dave_gz_multibeam_sonar`) is a Gazebo custom sensor of type
  `multibeam_sonar`: ray-based, point scattering, phase, reverberation and speckle, CUDA
  kernels (`sonar_calculation_cuda.cu`). It builds against gz-sim8, gz-sensors8,
  gz-rendering8 and gz-transport13, the packages Tier 2 already uses. It publishes a
  point cloud over gz-transport and the sonar image over **ROS 2** (`sensor_msgs/Image`,
  `marine_acoustic_msgs/ProjectedSonarImage`). A CUDA-free WGPU backend is an open pull
  request (#44 on `IOES-Lab/dave`).
- **The new machine reproduces the old recordings.** In a `ros:jazzy` container with
  `ros-jazzy-ros-gz`, 30 s of seed 0 recorded on the RTX 4070 equals the recording made on
  the old RTX 3050 (100 % of returns agree, zero difference, all five sensors).

## Decision
1. **Tier-2 recording runs in Docker** (`docker/Dockerfile.tier2`: `ros:jazzy`,
   `ros-jazzy-ros-gz`, the pinned Python environment; `docker/tier2.sh` runs a command in
   it as the host user on the GPU). The user-space Gazebo extraction of
   `experiments/gazebo/README.md` is no longer the reference setup.
2. **DAVE's sonar is built from the pinned Jazzy-era commit** `d2121a5` on top of that image
   (`docker/Dockerfile.dave`: CUDA 12.6 toolkit, compute capability 89, only the sonar
   packages) with one patch: the speckle seed of a "blazing" image, `time(NULL)`, becomes a
   fixed seed plus a frame counter, so that a recording is reproducible (LOG L34).
3. **Rigs stay kinematic** (ADR-0007 item 1). The sonar becomes a sensor of the two
   BlueROV2 rigs with the Gemini 720s parameters of `docs/hardware.md`, next to the ray-cast
   proxy, so the two can be compared on the same recording. It is recorded in a **second
   pass** over an existing recording (`record_sonar.py`), in an **acoustic world** (only what
   is below the waterline: DAVE has no water surface), one frame per keyframe through a small
   ROS 2 client (the C++ recorder stays gz-transport only). DAVE illuminates twice the vertical
   ray angles of the SDF; the SDF is written accordingly (LOG L34).
4. **A sonar-image front-end** (range-azimuth intensity image → peaks against a per-range noise
   floor → landmark parts, `avatar.tier2.sonar_image`) replaces the proxy's cluster step for
   the DAVE sonar (T-F2-03 v1).
5. Results are labelled "Tier 2 (Gazebo, kinematic rigs, DAVE sonar)". The proxy results
   stay available and labelled.
6. **PX4 and Clearpath vehicles stay deferred** (remainder of T-S2-02): vehicle dynamics
   do not change what the mapping claims rest on.

## Consequences
- R2 is addressed for **image formation** (speckle, reverberation, ambiguity), not for real
  sonar data; the sim-to-real gap stays to be reported.
- The sonar needs an NVIDIA GPU with CUDA. Until PR #44 is merged, a machine without one
  can only use the proxy.
- The pin is a commit that upstream no longer targets. Following upstream means
  Lyrical / Jetty (Ubuntu 26.04, gz-transport14): a new ADR superseding ADR-0001, and the
  recorder to port. Risk: bit rot of the pinned stack (the image is built from the pinned
  commit and the ROS `noble` repositories, which keep changing).
- Recording the sonar takes 138 s per seed on the RTX 4070 (two sonars, 601 keyframes), and
  77 MB per seed.
- **Results (LOG L34-L35, development seeds):** after two pipeline defects were fixed, G1 on
  the sonar images holds in 18/20 runs with ground-truth ids (proxy 20/20) and in 6/20 with the
  EKF tracker. A sonar image measures no diameter, so the wire format now says that a footprint
  of 0 x 0 means "not measured". Sonar recordings are not bit-reproducible; the stored ones are
  the reference data. The status stays Proposed until the PI has read these results.

## Alternatives considered
- *Move Tier 2 to Lyrical / Jetty now*: DAVE's current target, published Docker images,
  but it supersedes ADR-0001 and ports the recorder for a stack released months ago.
- *Stonefish* (`patrykcieslak/stonefish`, ROS 2 bridge `stonefish_ros2`): a marine simulator
  with imaging sonars, but a second simulator with its own world description; the
  air and ground robots would have to stay in Gazebo. Kept as an independent check of the
  sonar model if DAVE's proves inadequate.
- *OceanSim / Isaac Sim*: what the DAVE group itself uses now; a different platform and
  licence.
- *Our own image-formation model on the recorded depth rays*: quick, but it would validate
  the estimator against our own assumptions, which is the weakness of the proxy.
