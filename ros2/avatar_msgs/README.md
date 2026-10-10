# avatar_msgs

ROS 2 (Jazzy) interface definitions for Avatar SLAM. These are a **contract**
(ADR-0005): change them only through a new ADR.

| Message | Purpose |
|---|---|
| `LandmarkRecord` | One condensed landmark part (semantic mirror of a wire record) |
| `AgentDigest` | An agent's condensed landmark map (own measurements only) |
| `FrameAlignment` | Estimate of `T_sender_other` (4-DoF) with uncertainty |
| `EncodedPacket` | Raw wire bytes over an RF or acoustic link (comm emulator / modem gateway) |
| `RangeMeasurement` | Inter-agent acoustic range (task T-B4-01) |
| `Keyframe`, `Detection` | One agent's measurements at one keyframe: front-end → agent node (v0.2, ADR-0010) |

Frames, units and ids: `docs/conventions.md`. Byte layout: `docs/spec/wire_format_v0.md`.
