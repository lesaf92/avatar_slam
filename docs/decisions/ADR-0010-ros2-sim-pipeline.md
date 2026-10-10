# ADR-0010: The ROS 2 simulation pipeline: keyframes as messages, a lockstep clock

- **Status:** Proposed
- **Date:** 2026-10-09
- **Deciders:** PI; proposed by Claude

## Context
Goal G-5 (milestone M2, 2026-12-20) asks for the pipeline in ROS 2 end to end, with the same
gate G1 as the offline Tier-2 pipeline on the same seeds. Two pieces exist:
- the comm emulator node (`avatar_sim/comm_emulator`, T-S3-01), whose replay of a Tier-1 run's
  transmissions gives that run's link statistics exactly (LOG L55);
- the per-node exchange logic, factored out of the runner (`avatar.runner.TokenBuckets`,
  `agent_packets`, `gateway_packets`), so that agent and gateway nodes run the runner's code.

Agent and gateway nodes come next. Two questions decide their shape:

1. **Where an agent node gets its keyframes** (odometry increment and the front-end's
   detections). `avatar_msgs` has no such message, and messages are contracts that change only
   through an ADR (ADR-0005).
2. **How simulated time advances.** The offline runner processes one keyframe at a time in a
   fixed order: keyframes, then deliveries up to t, then the exchange at t. ROS 2 nodes run
   concurrently, and Python nodes are not real-time (ADR-0009).

## Decision (proposed)
1. **A `Keyframe` message** (`avatar_msgs` v0.2): odometry increment and its σ, absolute depth if
   measured, and the detections (body-frame point, σ, medium, class, extent, descriptor, track
   id). It is published by front-end nodes on `/avatar/<agent>/keyframe` and consumed by the
   agent node.

   The first front-end node *replays* a Tier-2 run, publishing the detections the offline
   front-end computed, so the backbone, the network and the gateway are tested in ROS 2 before
   the front-end itself moves there. A live front-end node (Gazebo sensor topics in, keyframes
   out) follows, and on the robots a C++ front-end publishes the same message.
2. **A lockstep clock for experiments.** A clock node publishes `/clock` one keyframe at a time
   and advances only when every node has reported the step done. Each step has two phases:
   - the emulator's deliveries up to t;
   - then the agents' keyframes and exchange, then the gateways'.

   The order is the offline runner's, so the parity test can ask for the *same* G1 run by run.
   A free-running mode (a fixed real-time factor) stays possible for timing studies once nodes
   are C++.

## Consequences
- **New contract:** `avatar_msgs` v0.2 with `Keyframe` and `Detection`, and a step-done message
  for the lockstep. The C++ side gets the same interface the hardware needs (front-end →
  backbone).
- **Parity becomes an exact test:** the ROS 2 pipeline on seed s must give the offline run's frame
  errors, which catches ordering and serialization bugs that a statistical comparison would hide.
- **Cost:** the lockstep makes the pipeline as slow as its slowest node. That's fine for
  simulation, but its timing says nothing about real robots.
- **Tasks:** T-S3-02 (agent, gateway and clock nodes; replay front-end; launch file; parity on
  the development seeds) and T-S2-02 (the fleet in Gazebo) build on this; live front-end nodes
  follow.

## Alternatives considered
- **Agent nodes read their keyframes from the run's files** (no new message). This is quickest,
  but the front-end → backbone interface stays undefined, and it is the one the robots need.
- **Free-running time from the start.** It is closer to robots, but results then depend on
  scheduling, and parity with the offline pipeline can only be statistical: a G1 count that moves
  by two runs could not be told apart from a bug (L28, L54).
