# ADR-0004: 4-DoF backbone on a waterline datum with condensed landmark sharing

- **Status:** Accepted
- **Date:** 2026-09-28

## Context
Agents in different media observe different parts of the same structures, and
talk over links whose bandwidth differs by ≥ 10³×. Distributed pose-graph
solvers (DGS, DPGO) iterate many rounds of boundary-pose exchange, which is
impractical on acoustic links with seconds of latency. All agents carry IMUs
(gravity), and every agent can reference the water surface (pressure sensor,
barometer plus known take-off height, USV hull, known quay height).

## Decision
1. **State:** 4-DoF poses `[x, y, z, yaw]` and 3-D landmark *parts*
   (`ABOVE` / `BELOW`), in gravity-aligned local frames whose `z = 0` is the
   waterline. Inter-agent frame transforms are 4-DoF, with a tight `z ≈ 0` prior.
2. **Per agent, two graphs:**
   - *Local graph*: own measurements only (odometry, landmark observations,
     depth/surface priors, intra-agent coaxial links).
   - *Fused graph*: local graph + frame variables `T_i_from_j` + inter-agent
     landmark factors built from neighbours' condensed landmarks.
3. **Sharing:** agents send **condensed landmarks** (position in their local
   frame, isotropic-horizontal and vertical σ, class, extent, flags,
   descriptor), computed from the **local graph only**. Because shared
   information never contains received information, double counting is
   impossible (the DDF-SAM rationale).
4. **Cross-medium factors** (`ABOVE` ↔ `BELOW`) constrain only horizontal
   position (the coaxial model). Same-medium factors constrain all three axes.
5. **Team frame:** the anchor agent's local frame. Other agents are expressed
   via the chain of estimated `T_i_from_j`, shared in `FRAME_ALIGNMENT` messages.

## Consequences
- An agent benefits from neighbours' *own* sensing, not from their fused
  knowledge. Multi-hop benefit comes through frame chaining and the gateway
  relay (C3), not through re-broadcasting fused estimates. A consistent
  multi-hop information-sharing scheme is research task T-B3-02.
- Roll/pitch must be compensated by front-ends (6-DoF is T-B1-05).
- The isotropic horizontal σ keeps shared covariances invariant under yaw.

## Alternatives considered
- *Distributed PGO (DPGO / DGS) everywhere*: too chatty for acoustic links.
  Kept as an option inside RF clusters (T-B3-01).
- *Centralized server (A&B-style)*: a baseline, not our design.
