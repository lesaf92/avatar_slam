# ADR-0003: Two-tier simulation, fast Python sim + Gazebo Harmonic/DAVE

- **Status:** Accepted
- **Date:** 2026-09-28

## Context
Algorithm research needs thousands of seeded runs (bandwidth sweeps, ablations)
that must run in CI without a GPU. The paper also needs sensor-realistic data
(sonar image formation, LiDAR, cameras) in a standard robotics simulator.
The initial scoping proposed porting `uuv_simulator` to ROS 2. The 2026-09-28
literature check found that **DAVE already provides a ROS 2 Jazzy + Gazebo
Harmonic branch** (GSoC 2024/2025, including a GPU multibeam sonar), and that
**LOTUSim** (IROS 2026) is a Gazebo + ROS 2 multi-domain maritime simulator.

## Decision
1. **Tier 1: `avatar.sim`** (Python). A kinematic multi-domain simulator with
   abstracted sensors (landmark-part observations), odometry drift models, and
   channel models. It is deterministic given a seed. Used for algorithm
   development, sweeps, and CI.
2. **Tier 2: Gazebo Harmonic.** One world with DAVE (underwater vehicles, sonar,
   DVL), PX4 SITL (aerial), a UGV, and a USV. Used for sensor-realistic
   experiments.
3. Both tiers read the **same scenario YAML** (world objects, agents, links),
   so results are comparable (task T-S4-01).
4. LOTUSim is evaluated as an alternative host for Tier 2 (task T-S2-04). We
   do **not** port uuv_simulator.

## Consequences
- Tier-1 sensor models are deliberately abstract (landmark-part level). Paper
  claims about perception accuracy must come from Tier 2 or real data.
- A Tier-1 → Tier-2 parity check (same scenario, same metrics) is required
  before paper experiments.
