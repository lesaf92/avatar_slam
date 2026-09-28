# ADR-0001: ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04

- **Status:** Accepted
- **Date:** 2026-09-28
- **Deciders:** PI, Claude (foundation session)

## Context
We need one middleware and simulator stack that hosts aerial, ground, surface,
and underwater robots together. ROS 2 Jazzy (LTS, May 2024 → May 2029) pairs
officially with Gazebo Harmonic (LTS). Project DAVE's ROS 2 branch targets
exactly Jazzy + Harmonic, PX4 SITL supports Gazebo Harmonic, and Swarm-SLAM
(a baseline) is ROS 2 native.

## Decision
Target **ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04** (Python 3.12, GCC 13).
The pure-Python core must also run on Python 3.10–3.12 without ROS.

## Consequences
- Docker base image: `ros:jazzy` + Gazebo Harmonic + DAVE (task T-I1-03).
- ROS 1-only baselines (e.g. parts of Kimera-Multi, DRACo-SLAM2) run in their
  own containers and are bridged via bags/`ros1_bridge` or offline replay (task T-E2-*).
- Newer pairs (e.g. ROS 2 Lyrical + Gazebo Jetty) are not targeted until an ADR says so.

## Alternatives considered
- *Humble + Fortress/Harmonic*: older and non-default pairing; DAVE's port targets Jazzy.
- *Isaac Sim / UE5 (HoloOcean, AirSim)*: better photorealism but heavier,
  closed or partly closed, and weaker at hosting all four domains in one
  ROS 2 world. Kept as possible photoreal add-ons.
