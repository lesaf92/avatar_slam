# ADR-0009: Python (`rclpy`) agent nodes for simulation experiments (amends AGENTS.md §2)

- **Status:** Proposed (2026-10-06); the PI accepts or rejects it (T-I1-09)
- **Date:** 2026-10-06
- **Deciders:** PI; proposed by Claude
- **Amends:** AGENTS.md §2 ("Real-time / onboard code, ROS 2 nodes: C++17"), for simulation only

## Context
Milestone M2 (2026-12-20) asks for ROS 2 end to end: Gazebo sensors → front-ends → backbone →
comm emulator (goal G-5, PLAN §10). The agent runtime (`avatar.agent.AvatarAgent`: local and
fused graphs, association, cycle check, digests, scheduler) and the EKF tracker exist only in
Python; `avatar_core` holds frames and the wire codec. Under AGENTS.md §2 the ROS 2 nodes would
wait for a C++ port of all of it (T-B2-01 and more), which is Paper-B-sized work and would make
M2 depend on it. The Python reference is also the test oracle (ADR-0002), so a node that calls it
runs exactly the code that produced the paper's Tier-1 and Tier-2 numbers.

## Decision
For **simulation experiments** (Gazebo, replays, the comm emulator), ROS 2 agent and front-end
nodes may be thin `rclpy` wrappers around `avatar_py`, in a package under `ros2/` that depends on
`avatar_py` (never the reverse, ADR-0002). Code that runs **onboard** a robot stays C++17 and is
ported when the hardware needs it (T-B2-01, T-H1-05), matched against the Python reference on
`testdata/` vectors as before. Messages and the wire format stay the contracts (ADR-0005), so a
C++ node can replace a Python one without changing its peers.

## Consequences
- M2 no longer waits for the C++ port; G-5 compares the ROS 2 pipeline with the offline Tier-2
  pipeline on the same seeds, with the same code.
- Python nodes are not real-time; the comm emulator and Gazebo run in simulated time, so this
  matters only for timing measurements, which stay out of the paper until the C++ port exists.
- Two node implementations may coexist later; the contracts keep them interchangeable.

## Alternatives considered
- **C++ port first** (AGENTS.md §2 as written): delays M2 by the size of T-B2-01 plus the
  association and agent runtime, with no new result for Paper A.
- **No ROS 2 pipeline for Paper A** (offline Tier 2 only): leaves goal G-5 and the claim of a
  ROS 2 system unsupported.
