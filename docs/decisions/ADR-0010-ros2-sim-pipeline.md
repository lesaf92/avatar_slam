# ADR-0010: The ROS 2 simulation pipeline: keyframes as messages, free-running time

- **Status:** Accepted with a change (PI, 2026-10-10): item 1 as proposed; item 2 **free-running time**,
  closer to real robots, instead of the lockstep clock
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

## Decision
1. **A `Keyframe` message** (`avatar_msgs` v0.2): odometry increment and its σ, absolute depth if
   measured, and the detections (body-frame point, σ, medium, class, extent, descriptor, track
   id). It is published by front-end nodes on `/avatar/<agent>/keyframe` and consumed by the
   agent node.

   The first front-end node *replays* a Tier-2 run, publishing the detections the offline
   front-end computed, so the backbone, the network and the gateway are tested in ROS 2 before
   the front-end itself moves there. A live front-end node (Gazebo sensor topics in, keyframes
   out) follows, and on the robots a C++ front-end publishes the same message.
2. **Free-running time** (the PI's choice, 2026-10-10). A clock node publishes `/clock` at a
   fixed rate (simulated seconds per wall second), or Gazebo does; every node acts on what has
   arrived by then, as robots do. Parity with the offline pipeline is statistical: G1 counts and
   frame errors over the development seeds, not run by run.

   *Proposed, not adopted:* **A lockstep clock for experiments.** A clock node publishes `/clock` one keyframe at a time
   and advances only when every node has reported the step done. Each step has two phases:
   - the emulator's deliveries up to t;
   - then the agents' keyframes and exchange, then the gateways'.

   The order is the offline runner's, so the parity test can ask for the *same* G1 run by run.
   A free-running mode (a fixed real-time factor) stays possible for timing studies once nodes
   are C++.

## Consequences
- **New contract:** `avatar_msgs` v0.2 with `Keyframe` and `Detection` (no step-done message:
  time runs free). The C++ side gets the same interface the hardware needs (front-end →
  backbone).
- **Parity is statistical** (free-running time): the ROS 2 pipeline is compared with the offline
  one on G1 counts and frame errors over the development seeds. A count that moves by about two
  runs between the two cannot by itself show a bug (L28, L54), so the comparison also checks the
  pieces that are deterministic: the comm emulator's replay (L55) and a keyframe round trip.
- **Timing:** Python nodes are not real-time (ADR-0009). The clock's rate must be low enough that
  every node keeps up, and timing results stay out of the paper until the nodes are C++.
- **Tasks:** T-S3-02 (agent, gateway and clock nodes; replay front-end; launch file; statistical
  parity on the development seeds) and T-S2-02 (the fleet in Gazebo) build on this; live front-end nodes
  follow.

## Alternatives considered
- **Agent nodes read their keyframes from the run's files** (no new message). This is quickest,
  but the front-end → backbone interface stays undefined, and it is the one the robots need.
- **Free-running time from the start.** It is closer to robots, but results then depend on
  scheduling, and parity with the offline pipeline can only be statistical: a G1 count that moves
  by two runs could not be told apart from a bug (L28, L54).
