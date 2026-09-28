# ADR-0005: Wire format and ROS messages are versioned contracts

- **Status:** Accepted
- **Date:** 2026-09-28

## Context
Independent agents implement senders and receivers in Python, C++, and ROS 2.
Silent drift in byte layouts or field meanings would corrupt experiments.

## Decision
- The inter-agent byte protocol is specified in `docs/spec/wire_format_v0.md`
  (version nibble = 0). Golden vectors live in `testdata/wire_v0_vectors.json`
  and are checked by **both** the Python (`avatar.comm.codec`) and C++
  (`avatar::comm`) test suites.
- `ros2/avatar_msgs` mirrors the protocol semantically; `EncodedPacket`
  carries raw wire bytes across simulated or real links.
- Any change needs a new ADR, a version bump, new vectors, and updates to all
  implementations in the same PR.

## Consequences
Implementations can be written by different agents in parallel without
coordination beyond the spec.
