# ADR-0006: Reference fleet, communication stack, and transparent relay

- **Status:** Accepted
- **Date:** 2026-09-28
- **Deciders:** PI (platforms, sensors, venue, authorship), Claude (communication stack)

## Context
The PI fixed the reference platforms: Husky UGV (VLP-16 + D435i), Tarot 680 UAV
(D435i + Cube), BlueROV2 UUV (DVL A50 + Micron Gemini 720s). The communication
stack was delegated to Claude. The fleet has **no surface vessel**, yet the plan
(and ADR-0004) assumed a USV bridging RF and acoustic media. The BlueROV2 is
normally tethered, and a tether would make any claim about acoustic
communication meaningless.

## Decision
1. **Fleet.** The platforms and sensors are listed in `docs/hardware.md` §1–2.
   The Tier-1 scenario `harbor_fleet` models them, with `ugv_0` as the anchor.
   A BlueBoat USV is optional.
2. **RF.** A 5 GHz Wi-Fi mesh carries inter-robot traffic, with ROS 2 Jazzy +
   `rmw_zenoh_cpp`. Only `/avatar/*` is shared between robots.
3. **Acoustic.** Primary modem: Water Linked Modem-M64 (64 bps). Upgrade path:
   SeaTrac X150 (USBL + ~100 baud). Wire packets on acoustic links are
   ≤ 64 B. Fragmentation into modem frames belongs to the modem driver.
4. **Tether policy.** The tether carries safety, teleoperation, logs, and time
   sync only. It **never** carries Avatar traffic. Without a modem, the ROS 2
   comm emulator throttles traffic to the M64 profile, and results are labelled
   "emulated acoustic".
5. **Surface gateway.** A new node role `gateway` (`AgentConfig.role`): a
   sensorless, store-and-forward relay between link types
   (`avatar.comm.gateway`, policy in its module docstring).
6. **Relay semantics (wire v0 clarification, no byte change).** A relayed
   `LANDMARK_DIGEST` keeps the **originator's** `sender_id`, so the relay is
   transparent. The relay may drop descriptors (`descriptor_dim = 0`) and may
   split records across packets. It never alters record contents. Receivers
   keep a record's previous descriptor when a descriptor-free copy arrives.
   Duplicate suppression uses (originator, type, seq).
7. **Budgets.** Each (node, link) has a token bucket. Unspent budget carries
   over to the next exchange tick, capped at max(4 ticks, 1 MTU). Frame-alignment
   packets count against the same budget. Acoustic digests omit descriptors
   (`AvatarParams.acoustic_descriptor_dim = 0`).
8. **Naming (conventions §6 update).** Agent names are `<class>_<index>` with
   class ∈ {`uav`, `ugv`, `usv`, `uuv`, `auv`, `gw`}.
9. **Venue and authorship (PLAN D1, D3).** Target RA-L or T-RO. Sole confirmed
   author: Luiz Eugenio Santos Araujo Filho.

## Consequences
- ADR-0004's no-double-counting invariant still holds: a relay forwards an
  originator's own-measurement digests and never fuses them.
- Cross-medium association in the default fleet is **direct** (UGV/UAV above
  water ↔ UUV below water). No surface agent sees both parts. This puts the
  weight on contribution C2 and raises aliasing risk R4.
- At 64 bps the team frame closes in minutes, not seconds. Mission lengths in
  experiments grow to ≥ 10 min. This makes simulator performance (T-S1-05)
  more urgent.
- New tasks: T-H1-01..03 (hardware bring-up, comm bring-up, ground truth),
  T-C6-01 (M64 driver and fragmentation).

## Alternatives considered
- *Tethered SLAM traffic*: rejected, because it removes the acoustic constraint
  that motivates the paper.
- *Buy a USV now*: kept optional. The gateway is cheaper, and a BlueBoat can be
  added later for the *Above and Below* comparison.
- *DDS over Wi-Fi without Zenoh*: rejected as the default because of known
  multicast-discovery problems on multi-robot Wi-Fi. It remains a fallback.
