# Architecture Decision Records (ADRs)

Short, immutable records of decisions that constrain the project. To change a
decision, write a new ADR that *supersedes* the old one. Do not edit
accepted ADRs, except to add a "Superseded by" line.

| ADR | Title | Status |
|---|---|---|
| [0001](ADR-0001-ros2-jazzy-gazebo-harmonic.md) | ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04 | Accepted |
| [0002](ADR-0002-ros-agnostic-cores.md) | ROS-agnostic cores, Python reference + C++ port | Accepted |
| [0003](ADR-0003-simulation-stack.md) | Two-tier simulation: fast Python sim + Gazebo/DAVE | Accepted |
| [0004](ADR-0004-backbone-estimator.md) | 4-DoF backbone on a waterline datum with condensed landmark sharing | Accepted |
| [0005](ADR-0005-contracts.md) | Wire format and ROS messages are versioned contracts | Accepted |
| [0006](ADR-0006-reference-fleet-and-comm-stack.md) | Reference fleet, communication stack, transparent relay | Accepted |
| [0007](ADR-0007-tier2-kinematic-rigs.md) | Tier 2 v0: kinematic sensor rigs and a ray-cast sonar proxy (amends 0003) | Accepted (v0) |
| [0008](ADR-0008-tier2-docker-dave-sonar.md) | Tier 2 v1: recording in Docker and DAVE's multibeam sonar (amends 0007) | Proposed |

Template: copy `ADR-template.md`.
