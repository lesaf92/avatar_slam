# Lab notebook

Dated, append-only record of findings, results, and negative results (AGENTS.md §5).
Newest entries first. Every result gives the command that reproduces it.

---

## 2026-10-10 (luiz-predator-neo, evening): Gazebo rendering live for ROS 2 (Claude)

Branch `wp/T-S3-03-live-frontend`; task T-S3-03 (G-5); **simulation (Tier 2, rendered live)**.
At `c0e8a8f`: `python experiments/ros2_parity.py --ws <install> --runs results/tier2 --tracking
ekf --live ugv_0,uav_0 --gazebo --seeds 0 ... 19 --jobs 4 --rate 5`
(`results/ros2_parity_gazebo.csv`).

### L58. The Gazebo rigs render live on the ROS 2 clock and give the recorded frames

**Stream mode.** `gz_recorder` gains a stream mode: after "RDY", each keyframe's rig poses
come in on stdin and its frames go out on stdout. It shares the keyframe step (stamp checks,
four physics steps; L28, L39) with the batch mode. A batch re-recording of Tier-2 seed 0 with
the new binary is **bit-identical** to the stored one: Gazebo's ray casts are deterministic.

**Node.** `experiments/gazebo/stream.sh` starts Gazebo and the streaming recorder in the
`avatar-tier2` image. On the host, the node `gazebo_rigs` writes the world and the plan, feeds
the rigs' ground-truth poses as `/clock` reaches each keyframe, and publishes the frames on the
image topics `sensor_replay` uses. It subscribes to `/clock` only once Gazebo is up, so the
clock cannot start without it.

**Checks:**
- **Frames:** 30 keyframes of seed 0, live, equal the recording at its float16 precision
  (VLP-16, both D435i).
- **Parity**, the Husky and the Tarot rendered and processed live (the BlueROV2s on replay):

| | G1 | frame error, ROS 2 − offline |
|---|---|---|
| offline | 19/20 | – |
| ROS 2, recorded frames (L57) | 19/20 | median 0.008 m, at most 0.15 m |
| **ROS 2, Gazebo live** | **18/20** | median −0.015 m, at most 0.12 m |

**Seed 6.** The one difference ran completely (every keyframe and exchange, no error); the
anchor never linked `uuv_1`, which offline and on recorded frames it did. The live frames are
float32, not the recording's float16, and time runs free, so a marginal run can go either way.
Over the three ROS 2 variants against offline, 3 runs are lost and none gained: the small cost of
free-running time noted in L56.

**Left for G-5:** the sonar robots live. DAVE renders at about 1.3 s per keyframe, so the clock
must run below real time.

---

## 2026-10-10 (luiz-predator-neo, later): live front-ends in ROS 2 (Claude)

Branch `wp/T-S3-03-live-frontend`; task T-S3-03 (G-5; PI 2026-10-10: LiDAR and depth robots
first, sonar on replay); **simulation (Tier 2)**. At `0fc2761`: `python experiments/ros2_parity.py
--ws <install> --runs results/tier2 --tracking ekf --live ugv_0,uav_0 --seeds 0 ... 19 --jobs 4
--rate 5` (`results/ros2_parity_live.csv`).

### L57. With live front-ends the team in ROS 2 passes G1 on the same runs as offline

**Front-end, keyframe by keyframe.** The Tier-2 front-end's batch loop becomes
`AgentFrontEnd.step(k, scans)`; `detections_for_agent` loops over it, with bit-identical
detections and statistics (EKF and ground-truth tracking, Tier-2 seed 3).

**Nodes:**
- `sensor_replay` publishes a recording's LiDAR scans and depth images as `32FC1`
  `sensor_msgs/Image`. The frames are stored as float16, so float32 carries them exactly.
- `frontend_live` runs `AgentFrontEnd` on those images.

**Release of detections, online.** Offline, an EKF track's detections count if its landmark
proves static at any time before the end of the run. Online, a keyframe cannot wait that long.

A fixed delay (10 or 30 keyframes) fails:
- it dropped about a fifth of the Tarot's detections (Tier-2 seed 6: 119 against 151 with no
  limit). Its depth camera sees a pile for one to three frames, and most of its landmarks prove
  static only on a revisit;
- live G1 fell to 15/20 (seeds 0, 6, 10 and 14 lost).

The fix uses what the back-end is, a factor graph:
- a keyframe leaves at once with the detections already released;
- a landmark released later sends its earlier detections as an *amendment*, a `Keyframe` that
  repeats an index, which the agent adds to its graph retroactively
  (`AvatarAgent.add_detections`; the default path is bit-identical).

Over a run this delivers the batch pass's detections: on the fixture both robots receive late
detections, and the only extras are those of landmarks that failed their static test after
release. The node test checks the message sequence against a direct pass.

**Parity** (development seeds 0-19, proxy, EKF tracker; the BlueROV2s on replay):

| | G1 | frame error, ROS 2 − offline |
|---|---|---|
| offline | 19/20 | – |
| ROS 2, every front-end replayed (L56) | 18/20 | within 0.20 m |
| **ROS 2, the Husky and the Tarot live** | **19/20, the same runs** | median 0.008 m, at most 0.15 m |

**Left for G-5:**
- a node that drives the Gazebo rigs and publishes these image topics live;
- the sonar robots' front-ends live (DAVE renders at about 1.3 s per keyframe; the clock must
  then run slower).

---

## 2026-10-10 (luiz-predator-neo): the team in ROS 2 (Claude)

Branch `wp/T-S3-02-agent-nodes`; task T-S3-02 (goal G-5; ADR-0010 accepted with free-running
time, D18); **simulation (Tier 2, replayed)**. At `d71d72d`, with a colcon install of `ros2/`:
`python experiments/ros2_parity.py --ws <install> --runs results/tier2 --tracking ekf [--sonar
sonar] --seeds 0 ... 19 --jobs 4 --rate 5` (`results/ros2_parity_ekf.csv`, `..._sonar.csv`).

### L56. The team in ROS 2 matches the offline pipeline, at most a few marginal runs worse

**Pipeline.** `avatar_msgs` v0.2 adds `Keyframe` and `Detection`. `avatar_sim` gains five nodes:
- `frontend_replay`: an offline front-end's keyframes, published as the clock reaches them;
- `agent_node`: `AvatarAgent` with the runner's exchange code;
- `gateway_node`;
- `comm_emulator`: L55;
- `sim_clock`: free-running, starting once every node listens.

`replay.launch.py` runs the team, one process per node.

**Two bugs found on the way, both fixed:**
- **Start-up race.** A keyframe published before its agent had subscribed was lost, and every
  later one was then refused (each is relative to the last). The clock now waits for all its
  listeners, and a front-end holds its keyframes until its agent listens.
- **Shutdown race.** The anchor (the Husky, the largest map) was interrupted in its last solve and
  wrote nothing; 16 of 20 runs lost their estimate. An agent now marks itself done only once
  its estimate is written, and gets 60 s before SIGTERM.

**Parity** (development seeds 0-19; G1 from the anchor's estimate of every frame):

| Front-end | G1 offline | G1 ROS 2 | passed by both | frame error, ROS 2 − offline |
|---|---|---|---|---|
| proxy, EKF tracker | 19/20 | 18/20 | 18 | median −0.02 m, at most 0.20 m |
| DAVE sonar, EKF tracker | 11/20 | 9/20 | 9 | median +0.10 m (12 seeds below 5 m in both) |

ROS 2 never passes a run that offline fails. Three runs pass offline only (proxy seed 17: 0.90
against 1.04 m; sonar seeds 0 and 1). Three to none over 40 runs is not significant (sign test
p = 0.25), but the direction is consistent with free time. A packet that arrives as an agent
exchanges can be read after the exchange and wait 20 s for the next one, which matters where an
alignment is marginal (sonar images). That is the cost of the choice in ADR-0010, not a defect.

**Left for G-5:** a live front-end node on Gazebo's sensor topics, in place of the replay
(T-S2-02).

---

## 2026-10-09 (luiz-predator-neo): the ROS 2 comm emulator (Claude)

Branch `wp/T-S3-01-comm-emulator`; task T-S3-01 (goal G-5). `colcon test --base-paths ros2
--packages-select avatar_sim` (ROS 2 Jazzy, `PYTHONPATH=avatar_py`).

### L55. The comm emulator node reproduces Tier 1's links exactly

`ros2/avatar_sim` (new, `ament_python`) holds simulation-only `rclpy` nodes (ADR-0009). Its
`comm_emulator` node:
- wraps the Tier-1 link model, now one function shared with the runner
  (`avatar.runner.make_network`: channels, memberships, ground-truth positions, losses drawn from
  seed + 20000);
- takes every agent's wire packets on one topic, `/avatar/comm/tx`, so they arrive in one order
  and the loss draws follow that order;
- publishes each delivery on `/avatar/<agent>/rx` once `/clock` passes its arrival time.

**Test** (the task's acceptance): a 300 s Tier-1 decentralized run records its transmissions; the
same packets, in order, go to the node, and `/clock` jumps to the run's end. The node's link
statistics equal the run's exactly (packets and bytes sent, deliveries, bytes delivered, losses,
per link), and every delivery is a packet its sender sent. CI's ROS job now installs SciPy and
puts `avatar_py` on `PYTHONPATH` for `colcon test`.

Not yet: positions come from the scenario's ground truth (right for the kinematic rigs, ADR-0007;
vehicles with dynamics need a pose topic), and the gateway's relaying is a node of its own (next,
with the agent nodes, T-S2-02).

---

## 2026-10-08 (luiz-predator-neo, evening): where the sonar tracker loses runs, with the Ping360 (Claude)

Branch `wp/T-F3-09-ekf-sonar-tuning`; task T-F3-09; **simulation (Tier 2)**. At `49604b1`:
`python experiments/sonar_tracker_study.py truth --runs results/tier2_ping360 --sonar sonar360
--jobs 28` (development seeds 0-19, the Ping360 fleet, sonar images; `results/f309_truth.jsonl`).

### L54. Association costs two development runs, the filter one: too few to tune on 20 seeds

| Tracking | merged | G1 | wrong: cross / BlueROV2 pair / above |
|---|---|---|---|
| ground-truth ids | 19/20 | 19/20 | 1/105, 0/40, 1/39 |
| EKF given ground-truth ids (duplicates only) | 18/20 | 18/20 | 1/101, 0/40, 2/37 |
| EKF | 17/20 | 16/20 | 5/93, 3/40, 3/38 |

With the Ping360 the filter itself costs one run (L41: three, without it) and association two.
Three runs of headroom is within the ±2 runs that unrelated changes move a 20-seed count (L28),
so tuning the sonar tracker on these seeds would fit noise; the held-out gap is larger (44 against
53 of 60, L51) but must stay untouched. Tuning needs a larger development set for the Ping360
fleet, e.g. 40 new seeds (80-119; about two hours of recording). The seed sets are the PI's
decision (D13), so this is proposed as D17 and nothing is tuned yet.

---

## 2026-10-08 (luiz-predator-neo, later): clutter on the Ping360's images (negative) (Claude)

Branch `wp/T-F3-08-ping360-clutter`; task T-F3-08; **simulation (Tier 2)**, the Ping360 fleet's
development seeds 0-19, sonar images, EKF tracker. The whole-turn option is in `b03c538`
(reverted in the next commit): `alignment_acceptance_study.py --configs "Ping360 DAVE turn, sonar
EKF" "Ping360 DAVE turn, sonar GT ids"` at `b03c538`. The other diagnostics are scratch scripts
(not kept).

### L53. Clutter is not what keeps the tracker from true-track accuracy; a better detector gains at most one run

1. **What the clutter is.** On seeds 0-4, of the Ping360 detections kept in the swept sectors, 70 %
   lie within 1 m of a pile, 23 % on the quay face, 6 % on hulls and 1 % elsewhere. A wall seen
   square-on gives one compact echo at its nearest point, and that point slides along the wall
   as the vehicle moves.
2. **Detecting on the whole turn** (the previous turn's pings are on the vehicle), with only the
   detections in the swept sector kept, judges noise floor, side lobes and extent on the whole
   turn. Spurious detections fall from 35 % to 31 % (`uuv_0`) and from 34 % to 28 % (`uuv_1`),
   and wrong alignments from 7 to 4. G1, however, is 15/20 against 16/20 with the sector cut
   (1 run gained, 2 lost), and 20/20 with ground-truth tracks. The option is reverted.
3. **A wall test on one image does not separate the quay from piles.** I tested the echo on the
   neighbouring beams at r0/cos(dθ), the range a plane at range r0 would have:
   - with all beams within ±10-45° and thresholds of 3-20 dB, the statistic is as high for
     piles as for the quay;
   - with only the beams where a plane would lie more than 0.5 m beyond r0, it is equally low
     for both.

   DAVE's quay echo is specular (near normal incidence only), and dense pile fields fill the
   neighbouring beams.
4. **The ceiling (diagnostic, uses ground truth):** dropping every detection that matches no part
   before the tracker leaves G1 at **17/20**, from 16/20. Wrong alignments fall to 0-1 (from 7)
   when the Ping360's clutter, or all sonar clutter, is removed. The three failures are teams that
   never merge. A perfect clutter filter gains about one run here; the gap to ground-truth tracks
   (19/20) is how the tracker associates detections of real piles (duplicates, L41, T-F3-06).

**Outcome.** T-F3-08 is closed as a negative result: no clutter filter is worth building for the
G1 count. Clutter still costs wrong alignments, so a temporal test may matter for map quality
later: a landmark whose world position slides with the vehicle is a wall, not a pile.

---

## 2026-10-08 (luiz-predator-neo): both fleets in the paper (Claude)

Branch `wp/T-E3-04-ping360-paper`; task T-E3-04 (PI, 2026-10-08: report both fleets; D16:
bench-test three M64s first); **simulation (Tier 2)**. At `aa62590` (clean): `make -C experiments
tier2-sonar-ping360 tables` (`paper/data/tier2_sonar_ping360.csv`, `..._heldout.csv`).

### L52. The sonar tables show both fleets

The sonar tables gain a block "with a Ping360 on each BlueROV2" (tiers `T2p` and `T2pekf`, its own
recordings `results/tier2_ping360`); the sonar paragraph cites it from macros. G1 with the EKF
tracker: 16/20 development, 44/60 held out (Gemini only: 11/20 and 32/60); with ground-truth
tracks 19/20 and 53/60 (Gemini only: 19/20 and 50/60). These reproduce L51. The two fleets are
different recordings (adding a sensor changes the simulated noise streams), so the comparison
between them is not paired; L51's paired comparison within the Ping360 recordings is the evidence.

---

## 2026-10-07 (luiz-predator-neo, late night): the Ping360 rendered by DAVE (Claude)

Branch `wp/T-S1-12-ping360-dave`; task T-S1-12 (D14); **simulation (Tier 2)**. Code `87ef367`
(rebased since; the recordings' `meta.json` say commit "unknown": in a git worktree the
container cannot resolve `.git`, which points outside its mount). `make tier2-record-ping360-sonar`
for seeds 0-79 (`results/tier2_ping360/*/sonar360.npz`: the Gemini's images copied from the
run's `sonar.npz`, the Ping360 as four DAVE fans; two seeds re-recorded after the driver's
frame-count guard caught an extra frame), then `alignment_acceptance_study.py --weak 8 --configs
"Ping360 off, sonar EKF" "Ping360, sonar EKF" "Ping360 DAVE, sonar EKF" "Ping360 DAVE, sonar GT
ids"` on seeds 0-19 and once on 20-79 (`results/l48_ping360_dave.jsonl`,
`results/l51_ping360_dave_heldout.jsonl`), with the D15 default.

### L51. A realistic Ping360 helps about half as much as the idealized one

**Rendering.** DAVE's multibeam sonar ignores a sensor's `<pose>` and looks along its model's
`x` axis: fans yawed inside one model all rendered the same view (read at yaw 0, each fan's
detections matched the true piles as well as fan 0's). Each fan is therefore a model of its
own, moved with the BlueROV2 at its pose turned by the fan's yaw. Check on seed 0, 30
keyframes: read at its own yaw, each fan's detections lie within 1.5 m of the ray-cast proxy's
returns in 87-100 % of cases, at any other yaw in 0-37 %. Behind the vehicles many detections
are hulls and the quay face, not piles.

G1 on sonar images, EKF tracker unless stated (merged; wrong cross / BlueROV2 pair):

| | development 0-19 | held out 20-79 |
|---|---|---|
| Ping360 ignored (control) | 12 (18; 9/89, 1/40) | 34 (51; 20/246, 6/118) |
| Ping360, ray-cast proxy (as in L45, re-run with D15) | 18 (18; 4/88, 0/40) | 54 (56; 6/268, 1/120) |
| **Ping360 rendered by DAVE** | **16** (17; 4/92, 3/40) | **44** (52; 18/259, 6/120) |
| Ping360 by DAVE, ground-truth tracks | 19 (19; 0/104, 0/40) | 53 (55; 2/266, 0/120) |

1. **The DAVE Ping360 helps, less than the proxy.** Held out, paired against the control: 17
   runs gained, 7 lost (sign test p = 0.06; the proxy: 22 and 2, p = 4e-5). On the development
   seeds it meets T-F3-06's target (16/20).
2. **The gap is intra-agent association on 360° images.** With ground-truth tracks the same
   images give 53/60. With the EKF tracker, wrong cross alignments stay at the control's level
   (18 against 20; the proxy 6). The full turn adds the hulls and the quay face around the
   vehicle as compact-looking echoes, which become landmarks.
3. **The decision stands** (D14: the Ping360 is bought). The next lever is clutter rejection
   on the Ping360's images (T-F3-08). The paper still reports the Gemini-only fleet; whether its
   sonar tables move to the Ping360 fleet is the PI's call.

---

## 2026-10-07 (luiz-predator-neo, night): the realism study on 20 seeds (Claude)

Branch `wp/T-S1-10-realism-20`; task T-S1-10; **simulation (Tier 1)**. At `6dbf59a`:
`make -C experiments realism tables JOBS=24` (seeds 0-19; `paper/data/realism.csv`,
`tab_realism`, now median / mean and the number of failed runs: not merged, or team ATE
above 1 m).

### L50. On 20 seeds GNC holds everywhere, Huber only at low error rates

1. **GNC-TLS** keeps the team at its error-free accuracy at every error level and on both
   presets (median and mean 0.11-0.13 m against 0.11 m without errors; no failed run).
2. **Huber** contains low rates and bursts (median 0.13-0.44 m, at most 3 failed runs of 20)
   but fails in all 20 runs at 15 % switches plus clutter. The 3-seed table (L48) hid this:
   its means mixed a few lucky runs.
3. **No kernel** fails in 19-20 of 20 runs at continuous errors, and in 8-11 with low-rate
   bursts. Its median is far below its mean (15 %: 0.78 m against 6.1 m): a few diverged runs
   dominate a mean, which is why the table now gives both (L33).
4. **Split revisits:** half split costs little (median 0.13-0.14 m); all split fails in 10-12 of
   20 runs whatever the kernel. A missing loop closure is not an outlier.

The paper's paragraph now says "Huber limits the damage at low error rates but not at high
ones" and "with any kernel the team fails in many runs".

---

## 2026-10-07 (luiz-predator-neo, continued): the Modem-M64 adapter (Claude)

Branch `wp/T-C6-01-m64-driver`; task T-C6-01. `pytest avatar_py/tests/test_m64.py`.

### L49. The M64 sends 8-byte packets and syncs in pairs; an adapter for Avatar's wire packets

**Protocol** (docs.waterlinked.com, Modem-M64 protocol page, read 2026-10-07):
- UART 115200 8-N-1, 3.3 V. Lines are `w` + `c`/`r` + command + `,`-fields + `*` + checksum.
- **8 bytes of payload per acoustic packet**, which may be binary; an all-zero payload is
  reserved for sync packets.
- Two roles, `a` and `b`, on channels 1-7. The modem "tries to pair with another modem".
- The checksum is CRC-8 with unstated parameters. A search over all CRC-8 variants (polynomial,
  initial value, reflection, final XOR, coverage) finds exactly one that reproduces the six
  examples of the page: polynomial 0x07, initial value 0, no reflection (CRC-8/SMBUS), over
  everything before `*`.

**Adapter** (`avatar/comm/m64.py`): commands with checksums; a parser that reads a received
packet's binary payload by length; and fragmentation of a wire packet into modem packets of
1 header byte (packet id, last flag, index) + 7 data bytes. A 64 B wire packet becomes 10
modem packets (80 B on the air, +25 %), and no modem packet is ever all zeros. A wire packet
missing any modem packet is dropped (the codec's CRC-16 also guards it). The loopback test is
lossless at 0 % loss; at 10 % loss every delivered packet is intact, and the incomplete ones
are dropped. No serial-port code yet: that comes with the hardware bring-up (T-H1-02).

**Consequences for the simulation and the purchase (D16, T-C6-02).**
1. The 25 % framing overhead is not modelled: every M64 number in the paper assumes 64 bps of
   wire payload.
2. If M64 modems only work in pairs, the simulated shared channel of two BlueROV2s and the
   gateway does not exist. The gateway would then need one modem per BlueROV2, with
   BlueROV2-to-BlueROV2 traffic relayed through it.
3. Whether the 64 bps is shared by the two directions is not stated.

Items 2 and 3 are UNVERIFIED (the page does not exclude three modems on a channel); a bench
test with three modems settles them before the gateway's modems are bought.

---

## 2026-10-07 (luiz-predator-neo): paper data with the scale-error states (D15) (Claude)

Branch `wp/D15-scale-default`; decisions D14, D15 and H2 (PI, 2026-10-07); **simulation**
(Tier 1 and Tier 2). At `4285992` (clean): `make -C experiments all-data JOBS=40` regenerated
every CSV in `paper/data`; tables and macros by `make_paper_tables.py`.

### L48. With the scale-error states every team is more accurate; the conclusions stand

Before (`8eafe5c` / `617db22`) -> after (`4285992`):

| Result | Before | After |
|---|---|---|
| Team ATE, M64 / X150 / 1 kbit/s (Avatar) | 0.120 / 0.128 / 0.107 m | 0.108 / 0.122 / 0.095 m |
| Team ATE, oracle | 0.084 m | 0.077 m |
| Server at 1 kbit/s (stride 10) | 0.121 m | 0.103 m (Avatar still better, earlier) |
| Drift: team ATE (scale states off -> on) | – | 0.49 -> 0.32 m |
| Drift: small-drift runs, mean change | −6 % (9 runs) | −7 % (9 runs; also −7 % without the states) |
| Drift: small-drift runs worse than solo | 4 | 5, up to +18 % |
| Tier 2 proxy, true tracks, G1 dev / held out | 20/20, 58/60 | 20/20, 59/60 |
| Tier 2 proxy, EKF, G1 dev / held out | 19/20, 55/60 | 19/20, 56/60 |
| Tier 2 UAV LiDAR, EKF, G1 | 78/80 | 80/80 |
| Tier 2 sonar images, true tracks, G1 dev / held out | 20/20, 49/60 | 19/20, 50/60 |
| Tier 2 sonar images, EKF, G1 dev / held out | 11/20, 34/60 | 11/20, 32/60 (merged 53 -> 45) |
| Loss test, VoI vs FIFO / vs quality (400 runs) | 212:173 (p = 0.0004) / 212:223 (p = 0.28) | 226:182 (p = 1e-4) / 226:214 (p = 0.22) |

1. **Tier 1:** every team is more accurate (Avatar −4 … −11 %, the oracle −8 %); the server
   comparison and the bandwidth and loss conclusions are unchanged. The pre-registered loss
   test, re-run with the new estimator, gives the same answer (it was not re-specified: same
   seeds, same test).
2. **Drift (H1):** the average gain on the drifting AUV is unchanged (−7 % with or without the
   states); the states lower the team's error by a third. The drift table's oracle column is
   now the oracle's trajectory of the vehicle aligned on its own (it was its team-frame error),
   like the solo and Avatar columns: on seed 7 it is no better than solo (L46).
3. **Tier 2:** G1 counts move by one or two runs, within the ±2 runs that a NumPy version also
   causes (L28). The exception is the EKF tracker on sonar images held out: G1 34 -> 32/60 and
   fewer merged teams (53 -> 45); its failures are still the BlueROV2 with sparse fixes (22 of
   28), which the Ping360 addresses (D14, T-S1-12).
4. **Prose checked against the new data** (scripts in the session, not kept): the held-out
   proxy failure is now one run (a BlueROV2 at 1.1 m); EKF failures are the Tarot in 2 of 4
   held-out runs ("most often" -> "often"); "VoI no better than quality" -> "not significantly
   better"; "missing half of the revisits costs nothing" -> "costs little" (GNC 0.113 vs 0.095 m);
   the drift paragraph now gives the number of runs that got worse and the largest increase
   from macros, and the discussion's second limitation follows L46.

---

## 2026-10-07 (luiz-predator-neo, late): bandwidth to 10 kbit/s and packet loss; H2 (Claude)

Branch `wp/T-C5-01-bandwidth-loss-sweep`; task T-C5-01 (goal G-4); **simulation (Tier 1)**,
`fleet_default`, 600 s. At `617db22`: `make -C experiments bandwidth loss loss-confirm tables`
(`paper/data/bandwidth_sweep.csv`, `loss_sweep.csv`, `loss_confirm.csv`; tables `tab_bandwidth`,
`tab_loss` and the macros `loss_confirm.tex`, now in the paper's ablations).

### L47. Under heavy loss an informed digest order beats FIFO; VoI does not beat the quality order

1. **Rate.** From 512 bit/s to 10 kbit/s the team merges at 100 s whatever the order (10/10,
   team ATE 0.106 m): above that the link is not the limit. At 1 kbit/s the team sends 17 KB
   in 600 s, well under its budget: it runs out of content.
2. **Loss** (the same loss at every range, seeds 0-9, `tab_loss`). At 1 kbit/s every team
   merges even at 80 % loss, later (100 -> 310-330 s). At 64 bit/s merging fails between 40 and
   80 % loss (60 %: FIFO 2/10, quality 4/10, VoI 7/10).
3. **Pre-registered test.** Seeds 10-49 at 64 bit/s gave the same direction, not significant
   (VoI vs FIFO at 50 / 60 % loss: 28 vs 24 and 15 vs 10 of 40, sign test p = 0.42 / 0.23).
   I then fixed a test before running it, recorded with a timestamp in
   `results/c501_prereg.txt`:
   - seeds 50-249, 50 and 60 % loss pooled (400 paired runs);
   - two-sided sign test on team merged, VoI vs FIFO and VoI vs quality, α = 0.05;
   - reported whatever the outcome.

   **VoI merges more often than FIFO:** 212 vs 173 of 400 (discordant 78:39, p = 0.0004).
   **VoI does not beat quality:** 212 vs 223 (37:48, p = 0.28).
4. **H2 as worded.**
   - "≥ 90 % of full-communication accuracy at ≤ 20 % of the bytes" holds for *every* order,
     FIFO included: at 64 bit/s the team sends 13.6 % of the bytes it sends at 1 kbit/s, for
     team ATE 0.118-0.121 m against 0.106 m (ratio 0.88-0.90; `loss_sweep.csv`, 0 % loss).
     It is a property of the digests, not of the VoI order.
   - "Beats FIFO at equal bytes on acoustic links" holds only when few records get through
     (heavy loss, or about 32 bit/s, `tab_bandwidth`), and then for the quality order as well.

**Proposed rewording of H2 (G-4, PI):** "At equal bytes, ordering map digests by how likely they
are to be matched merges the team more often than FIFO when few records get through (low rate,
heavy loss); a value-of-information order adds nothing over a simple quality order." The
default stays the quality order; T-C2-01 is a negative result against it.

---

## 2026-10-07 (luiz-predator-neo, night): does collaboration hurt the drifting AUV? (Claude)

Branch `wp/T-X1-04-never-hurt`; task T-X1-04; **simulation (Tier 1)**, M64. At `590e02e`:
`python experiments/never_hurt_study.py --presets fleet_transit_anchored fleet_default --seeds
0 1 2 3 4 5 6 7 8 9 --out results/never_hurt.csv`. "Oracle" is the centralized oracle's own
trajectory of the agent (all data, true associations), aligned per agent like solo and fused.

### L46. Strict "never hurt" is unattainable; the harm comes from unmodelled scale errors

`uuv_1` ATE [m], mean of 10 seeds, and the runs in which fused is worse than solo:

| `fleet_transit_anchored` | solo | fused | oracle | fused > solo | team ATE dec / oracle |
|---|---|---|---|---|---|
| defaults | 0.586 | 0.549 | 0.182 | 4 (+1 … +18 %) | 0.486 / 0.138 |
| simulation without scale errors | 0.574 | 0.437 | 0.113 | **0** | 0.335 / 0.111 |
| scale-error states estimated | 0.520 | 0.476 | 0.114 | 6 (+0 … +18 %) | **0.322 / 0.108** |

On `fleet_default` the same three rows give team ATE 0.120 / 0.084, 0.107 / 0.076 and
0.108 / 0.077 m (fused worse than solo for `uuv_1` in 1, 2 and 3 runs, at most +6 %).

1. **The acceptance criterion cannot be met by any estimator.** On seed 7 even the oracle is
   worse than solo (0.078 m solo, 0.089 oracle; +9 % with the scale states): that solo run is
   lucky. "Fused ≤ solo in every run" asks for more than the optimal estimator delivers.
2. **Cause of the harm: the BlueROV2's systematic odometry errors that the estimator does not
   model**, a 1 % DVL scale error and a 1 % gyro scale error. In a simulation without them
   (the draws kept at σ = 1e-12, so all other noise is identical) fused ≤ solo in all 10 runs,
   with larger gains (seed 1 −82 %, seed 6 −70 %).
3. **Estimating them** (`AvatarParams.model_odometry_scale`, off by default: two scalar states
   per platform with zero-mean priors of the spec σ, as for the heading bias, D9) recovers the
   team accuracy of the error-free simulation: team ATE −34 % decentralized and −22 % oracle on
   the transit, −10 % and −8 % on the default fleet. It does not remove the harm counts,
   because solo improves too (it can partly estimate them on its own landmarks).
4. **What is left is a gap, not a fault.** Where fused is worse than solo the oracle gains
   43-91 %. These are diagnostics on seeds 3, 5, 6 and 7 (scratch scripts, not kept):
   - every pair in `uuv_1`'s fused graph is correct (ground truth);
   - the remote landmarks' σ are honest (NEES/dof ≤ 1.1);
   - the solve has converged (200 iterations from cold give the same result);
   - GNC cuts no extra own observation;
   - holding b, s and g at their local values in the fused graph, or doubling the remote σ,
     leaves the harm (seed 7 +15 … +18 % in every variant).

   Decentralized fusion uses correct information poorly: one free frame per neighbour, and
   remote landmarks as fixed measurements (L23). That is T-X1-03.

**Outcome.** T-X1-04 as stated is closed as unattainable; the criterion that can be tested is
"no avoidable harm": fused ≤ solo in every run where the oracle improves on solo, which moves
to T-X1-03 together with the decentralized–oracle gap (`uuv_1` fused 0.476 m against oracle
0.114 m with the scale states). Whether the scale states become the default is decision D15
(PI): it changes every number in the paper.

---

## 2026-10-07 (luiz-predator-neo, evening): a Ping360 on each BlueROV2 (Claude)

Branch `wp/T-S1-11-ping360-sim`; task T-S1-11 (decision D14); **simulation**. At `2ea4d40`:
`make tier2-record-ping360` for seeds 0-79 (range data with `--scenario-arg uuv_ping360=true` in
`results/tier2_ping360`, all through the geometry gate; the Gemini's DAVE images are hard links
to the reference fleet's), then `alignment_acceptance_study.py --weak 8 --configs "Ping360 off,
sonar EKF" "Ping360, sonar EKF" "Ping360 full turns, sonar EKF" "Ping360, sonar GT ids"` on the
development seeds 0-19, and once, after them, on 20-79 (rows in `results/l45_ping360*.jsonl`).

### L45. With a Ping360 the EKF tracker works on sonar images: held-out G1 34 -> 56 of 60 (idealized Ping360)

**Model.** Tier 1: a 360° sonar detection model (2° x 25° beam, 30 m range setting). Tier 2: a
ray-cast proxy recorded as a full turn at each keyframe, of which the front-end keeps the sector
the head swept since the previous keyframe, at 21 s per turn (the product page gives 3.4-4.3 s at
a 1 m range setting and 33 s at 50 m; 21 s interpolates to 30 m, UNVERIFIED). The Gemini keeps
DAVE's sonar images. The control is the same recording with the Ping360 ignored, so the Tier-1
noise is identical (adding a sensor shifts the random streams: the control's G1 differs from the
reference fleet's by chance, 14 against 11 of 20 on the development seeds).

G1 (merged; wrong alignments: cross / BlueROV2 pair), sonar images:

| | development 0-19 | held out 20-79 |
|---|---|---|
| EKF, Ping360 ignored (control) | 14 (17; 6/89, 4/40) | 34 (53; 24/242, 5/117) |
| **EKF, Ping360 (21 s per turn)** | **18** (18; 3/86, 1/40) | **56** (58; 6/272, 4/120) |
| EKF, Ping360, a full turn each keyframe (bound) | 18 (19; 3/81, 1/40) | not run |
| ground-truth tracks, Ping360 | 20 (20; 0/95, 0/40) | 60 (60; 0/282, 0/120) |

1. **The Ping360 closes most of the gap.** Held out, paired by seed: 33 runs pass with and
   without it, 23 only with it, 1 only without (sign test p = 3e-6); wrong cross alignments fall
   from 24 to 6, the median time to merge from 400 to 360 s. On the development seeds it meets
   T-F3-06's target (>= 16/20) with the tracker unchanged.
2. **The slow sweep costs little**: 21 s per turn gives the same development G1 as a full turn at
   every keyframe (18/20): what matters is that every pile around the vehicle is seen within a
   turn, not at once.
3. **Caveat: the Ping360 is idealized.** It is a ray-cast proxy (no speckle, side lobes, or the
   quay and hull clutter that DAVE's images have), so these numbers are an upper bound for the
   real sensor. The vehicle's motion within one keyframe and the sweep time at 30 m are also
   approximations. A Ping360 rendered by DAVE (T-S1-12) is the next fidelity step before the
   paper relies on it.

**For D14:** in simulation the Ping360 is a strong candidate for the BlueROV2s (analogous to the
VLP-16 on the Tarot, L31); the PI decides the purchase. Nothing in the paper changes yet.

---

## 2026-10-07 (luiz-predator-neo, later): every cited reference verified; the PDF build (Claude)

Branch `wp/T-R1-04-verify-bib`; task T-R1-04. Not a result: bibliography hygiene (AGENTS.md §5).

### L44. All 33 cited entries verified against their DOI records; the local PDF build needed the IEEEtran class

1. **The PDF was not blocked by the citations.** `make -C paper` stopped at line 4 of `main.tex`:
   "File `IEEEtran.cls' not found". The host's TeX Live lacked `texlive-publishers`. Its two files
   (`IEEEtran.cls`, `IEEEtran.bst`) were unpacked into `~/texmf` (no root); the paper builds (8
   pages, no undefined reference). `paper/README.md` now names the requirement. Draft builds never
   depended on citation status: only `make submission-check` does.
2. **Every entry checked field by field** against its Crossref record
   (`api.crossref.org/works/<doi>`: title, year, volume, issue, pages, number of authors), LOTUSim
   against arXiv 2607.03072v1 (no DOI registered yet). Status: 21 UNVERIFIED + 12 verified-web ->
   33 verified (`python tools/check_citations.py`). Corrections:
   - completed author lists that read "and others": MNE-SLAM (11 authors), Z-Splat (8), DAVE (11),
     SonarSplat (6), LOTUSim (7, was "LOTUSim authors");
   - SonarSplat is now an RA-L article (10(12):13312-13319), no longer an arXiv preprint;
   - ROMAN: "Yi Xuan" -> "Yixuan"; RSS proceedings named "Robotics: Science and Systems XXI";
   - Factor Graphs for Robot Perception: the journal issue (doi 10.1561/2300000043), not the book;
   - DOIs and page ranges added everywhere; volumes and issues for SlideSLAM, GRAND-SLAM, HAMMER,
     DiNNO, Above and Below (11(1):129-136).
   No title, year or venue claimed before was wrong.
3. `make submission-check` still fails, now only on the 36 drafting markers (`\todo`,
   `\draftnote`) the sections still hold, which belong to the writing tasks (T-P2 ... T-P6).

---

## 2026-10-07 (luiz-predator-neo): 200 clique seeds on the held-out seeds, and the paper data (Claude)

Branch `wp/T-F3-07-window-association`; tasks T-F3-06/07; **simulation**. 200 clique seeds are the
default since `1c96113` (chosen on the development seeds, L41). Held-out seeds evaluated once, as
part of `make -C experiments all-data JOBS=28` at `8eafe5c` (paper data committed as `adaf085`;
previous paper data at `1666748`, 40 seeds).

### L43. On fresh seeds 200 clique seeds hold: sonar-EKF G1 30 -> 34/60 and wrong alignments 123 -> 53; with ground-truth tracks fewer teams merge

G1 (merged; wrong alignments), 40 -> 200 clique seeds:

| Perception | development 0-19 | held out 20-79 |
|---|---|---|
| sonar images, EKF | 6 -> **11** (17 -> 17; 38/157 -> 13/163) | 30 -> **34** (51 -> 53; 123/502 -> 53/490) |
| sonar images, ground-truth tracks | 19 -> 20 (20 -> 20) | 51 -> **49** (57 -> 52; 18/513 -> 7/495) |
| proxy, EKF / ground-truth tracks | 19 / 20, unchanged | 55 / 58, unchanged |
| UAV-LiDAR fleet (0-79), EKF / ground-truth tracks | 78 / 78, unchanged | |

1. **The gain holds on fresh seeds** for the EKF on sonar images (+4 runs held-out, wrong
   alignments more than halved), the perception it was chosen for.
2. **It costs merges with ground-truth tracks on sonar**: held-out 57 -> 52 teams merged, G1 51 ->
   49; wrong alignments 18 -> 7. More seeds find more competing hypotheses, and the ambiguity test
   refuses more alignments (not traced run by run). Of the 11 held-out failures, 8 leave a robot
   out of the team and 3 misalign one (1.4, 1.4 and 58 m).
3. **Sonar-EKF failures** (26 held-out): 7 unmerged, 19 off (1-60 m); the worst robot is BlueROV2 1
   in 16, as on the development seeds (L41).
4. **Tier 1** moves by at most one run or a few millimetres: server ATE +0.001-0.002 m, cycle
   check 833 -> 830 accepted of 1000, bandwidth 32 bit/s one cell 8/10 -> 7/10, drift -6 -> -7 %,
   realism no-kernel and Huber columns (the GNC-TLS column the paper uses changes by 0.02 m in
   one cell). Diagnostic NN and registration tiers move by one or two runs either way.
5. Paper text updated (`62476cf`): the clique seeds and why (method), the sonar failures as they
   are now (experiments), the sixth limitation (sparse fixes, the window's negative result, a
   sensor as the remedy).

**T-F3-06 stays open, blocked on a decision** (PLAN D14): its target (sonar-EKF G1 >= 16/20 on the
development seeds) is not reached by software (11/20); the remaining failures are BlueROV2 1's
sparse fixes, whose remedy for the Tarot was a sensor (D6).

---

## 2026-10-06 (luiz-predator-neo, late night): a window of keyframes for sparse fixes does not help (Claude)

Branch `wp/T-F3-07-window-association`; task T-F3-07; development seeds 0-19 only; **simulation**.
At `1c96113` (the window code of `499503f`, and 200 clique seeds by default):
`tier2_study.py --runs results/tier2 --seeds 0 ... 19 --tiers T2ekf T2sekf --ekf window_frames=W`
and the same with `--runs results/tier2_uavlidar --tiers T2ekf` (outputs in `results/l42/`).

### L42. Joint pairing over a window of keyframes admits wrong matches; deferring alone does not help (negative result)

**What was tried** (`EkfTrackerParams.window_frames`, `499503f`): a detection with landmarks in its
gate but no confident match waits up to W keyframes, carried with the odometry into the current
body frame with the odometry's variance added; each keyframe the pending and the undecided
detections go into the joint pairing together (which then also takes fixes that do not beat
"new" on their own); after W keyframes a pending detection is decided as before. With W = 0 the
front-end is bit-identical to the code before it (three recordings); unit tests showed sparse
fixes of an irregular row matched one per keyframe.

G1 (merged; wrong alignments) on the development seeds:

| Window | proxy, EKF | sonar, EKF | UAV-LiDAR fleet, EKF |
|---|---|---|---|
| 0 | 19 (20; 6/174) | 11 (17; 13/163) | 20 (20; 0/181) |
| 3 | 14 (15; 21/167) | 7 (12; 37/154) | 18 (19; 13/173) |
| 6 | 14 (17; 29/168) | 9 (14; 42/151) | 18 (19; 13/171) |
| 10 | 13 (16; 29/167) | 5 (14; 46/150) | 18 (19; 13/171) |
| 3, widened pairing for pending detections only | 15 (15; 20/167) | 7 (14; 35/156) | 18 (19; 8/174) |
| 3, no widened pairing (deferral only) | 18 (19; 7/177) | 9 (17; 19/167) | 20 (20; 0/181) |

(The last two rows used an exploratory switch on the widened criterion, not committed.)

1. **Every variant is worse than no window**, also for the LiDAR fleet, whose frames are dense:
   pairing fixes from several keyframes jointly admits wrong matches (wrong alignments triple),
   and deferral alone loses runs. The relative geometry of fixes carried over a few keyframes is
   less certain than the model assumed, and holding a detection back withholds its update from
   the pose.
2. **Reverted** (`1488623`): the code and its tests are gone, the front-end is bit-identical to
   the code before it; the clique-seed default of L41 stays.
3. Sparse fixes (BlueROV2 1 on sonar, the camera-only Tarot) remain the limit of the EKF tracker.
   The Tarot's was removed by a sensor (L31, decision D6); BlueROV2 1's analogue is a wider or a
   second sonar, or a path that keeps more piles in view, a decision for the PI.

---

## 2026-10-06 (luiz-predator-neo, night): what the EKF's wrong alignments on sonar are made of (Claude)

Branch `wp/T-F3-06-ekf-sonar`; task T-F3-06; development seeds 0-19 only; **simulation**. Defaults
of D13 (veto rule, threshold 8). Commands at `f7f858e` (outputs in `results/l41/`):
`experiments/sonar_alignment_anatomy.py {anatomy,history,seeds} [--tracking oracle]
[--clique-seeds 200]`, `alignment_acceptance_study.py --weak 8 --clique-seeds {40,200,400}`,
`sonar_tracker_study.py {clutter,minlen,birth,truth} --clique-seeds 200`; at `7b751f2`:
`sonar_alignment_anatomy.py seeds [--tracking oracle]`, `sonar_tracker_study.py density
--clique-seeds 200`. All with `--jobs 28`.

### L41. The wrong alignments are a seeding failure of the alignment search; 200 clique seeds take sonar-EKF G1 from 6 to 11 of 20; the rest is BlueROV2 1's tracker

1. **Not clutter.** The 22 wrong cross alignments (BlueROV2 with UGV or UAV) of the sonar-EKF
   runs are built of real piles matched to the wrong pile: of their inlier pairs 72 % pair two
   different objects, 4 % involve a clutter landmark, 11 % a split track (right ones: 99 %
   correct). The receiver gets about the same landmarks as with ground-truth tracks (median 18-19
   piles, 15 of them its own; 4 clutter), and the same pairs align right with ground-truth tracks.
2. **Adopted late, not stuck from early on**: at keyframes 360-600; four replaced a right one.
3. **The true hypothesis is there but never grown.** At the end of the run the true transform
   is supported as well as the wrong one in several cases (8 against 8, 12 against 11, 13 against
   9 landmarks within 1 m). Sonar landmarks carry no footprint, so every pair is a candidate:
   median 934 candidate pairs per cross pair with the EKF's tracks, 386 with ground-truth tracks
   (where 40 seeds give 80 right alignments and no wrong one). Cliques are grown from the `clique_seeds` = 40 highest-degree candidates; in the denser
   graph none is a true pair, the true hypothesis is not among the hypotheses, and the ambiguity
   test has no competitor to see. Re-aligning the final maps of every cross pair (160):

   | clique seeds | 40 | 100 | 200 | 400 | 1000 |
   |---|---|---|---|---|---|
   | right | 44 | 52 | 61 | 61 | 60 |
   | wrong | 11 | 9 | 5 | 4 | 4 |
   | `align()` median [ms] (at `7b751f2`, idle machine) | 34 | 41 | 56 | 85 | 151 |

4. **Whole runs** (G1, merged; development seeds): 40 / 200 / 400 seeds:

   | Perception | 40 | 200 | 400 |
   |---|---|---|---|
   | Tier 1, proxy with ground-truth ids, LiDAR fleet (both) | 20 | 20 | 20 |
   | proxy, EKF | 19 (20) | 19 (20) | 18 (19) |
   | sonar, ground-truth ids | 19 (20) | **20** (20) | 20 (20) |
   | sonar, EKF | 6 (17) | **11** (17) | 11 (17) |

   Sonar-EKF wrong alignments used fall from 35/141 to 13/160. **200 is the choice** (400 loses a
   proxy-EKF run). It is not made the default yet: it changes every alignment, so it waits for
   the rest of T-F3-06, one held-out evaluation and one regeneration of the paper data.
5. **At 200 seeds clutter no longer matters.** Removing every clutter detection with the ground
   truth: G1 10/20 (L36, at 40 seeds, it gained two runs; the clutter's harm was the candidate
   graph). Minimum track length 8/12/16/24: 11/7/6/7; the birth-test variants of L36: 5-11/20.
6. **What is left is BlueROV2 1.** All nine failing runs have BlueROV2 1 off (1.3-30 m) or out of
   the team; BlueROV2 0 is within the gate in every run. BlueROV2 1 gets half the detections
   (median 1204 clusters per run against 2650). The EKF given ground-truth identities
   (`ekf_truth`) reaches G1 15/20 (ground-truth tracks 20/20): association errors cost four runs,
   the filter five. In the failing seeds the filter, even with ground-truth identities, splits
   BlueROV2 1's piles (seed 12: 39 tracks for 29 piles; 13: 41 for 30): when its pose is
   uncertain between sparse fixes, the new-landmark test prefers a duplicate to a match, the
   duplicates lose the loop closures, and the map drifts.
7. **The new-landmark test is a trade, not a fix** (`landmark_density_per_m2`; default 0.005):

   | density | 0.005 | 0.002 | 0.001 | 0.0005 |
   |---|---|---|---|---|
   | EKF with ground-truth identities | 15 | 18 | 18 | 18 |
   | EKF | 11 | 10 | 8 | 7 |

   Fewer duplicates help only when identities are known; without them the same setting admits
   wrong matches. BlueROV2 1, like the camera-only Tarot (L30-L31), sees too few piles per frame
   to pair them reliably.

**Next (T-F3-06).** Pair detections over a window of keyframes rather than one (the software
remedy L31 left open for the Tarot), so a sparse sensor gets the joint geometry of several fixes;
then make 200 clique seeds the default, evaluate once on the held-out seeds and regenerate the
paper data. The hardware analogue of D6 (a wider or second sonar on that vehicle) is the
alternative.

---

## 2026-10-06 (luiz-predator-neo, evening): the paper data under decision D13 (Claude)

Branch `wp/T-E3-03-d13-decisions`; task T-E3-03; **simulation**. The PI accepted L39's
recommendation (D13): the veto rule (threshold 8) by default, Tier 2 over every clean recording,
seeds 0-79. New recordings, all through the geometry gate: UAV-LiDAR fleet seeds 20-29; DAVE sonar
seeds 20-39 (share of sonar detections that match a part 0.69 and 0.76, as on the other seeds).
Paper data at `1666748` (committed as `e485901`): `make -C experiments server cycle bandwidth drift
realism tier2 tier2-heldout tier2-lidar tier2-sonar tier2-sonar-heldout tables JOBS=28`. Previous
paper data: `58d11da`.

### L40. G1 over 80 seeds with the veto rule; what the default changed

G1 passes (development 0-19 / held out 20-79 / all):

| Perception | 0-19 | 20-79 | 0-79 |
|---|---|---|---|
| Tier 1 | 20/20 | 60/60 | 80/80 |
| proxy, ground-truth tracks | 20/20 | 58/60 | 78/80 |
| proxy, EKF | 19/20 | 55/60 | 74/80 |
| proxy, NN | 8/20 | 38/60 | 46/80 |
| UAV-LiDAR fleet, ground-truth tracks | | | 78/80 |
| UAV-LiDAR fleet, EKF | | | 78/80 |
| sonar images, ground-truth tracks | 19/20 | 51/60 | 70/80 |
| sonar images, EKF | 6/20 | 30/60 | 36/80 |

1. **Tier 1**: every table is unchanged except realism, whose no-kernel and Huber columns move as
   L37 item 9 found (default preset, 15 % + 1.0, Huber 0.76 -> 0.54 m; exploration, 15 % + 1.0,
   Huber 0.75 m with 3/3 merged -> 0.64 m with 2/3); the GNC-TLS column is unchanged. One server
   value differs in the 15th digit.
2. **The default on the development seeds** (`tier2.csv`, every tier against `58d11da`): EKF seed 8
   fixed (frame error 59.5 -> 0.25 m); sonar with ground-truth tracks seed 8 fixed (18 -> 19/20).
   Among the diagnostic trackers it stops six wrong merges that already failed G1 (registration
   seeds 0, 1, 5, NN seed 5, NN on the BlueROV2s seed 8, NN on Husky and Tarot seed 12; frame
   errors 1.3-46 m before) and **breaks one run**: NN on Husky and Tarot, seed 1, 0.28 -> 60.9 m.
   L39 item 8 ("never breaks a run") holds for the perceptions the paper relies on, not for that
   diagnostic tier.
3. **What still fails.** With ground-truth tracks on the proxy and on the LiDAR fleet: BlueROV2 1
   just above the gate (1.04-1.13 m; seeds 45, 71 and 46, 57). EKF on the proxy: the Tarot (dev 19,
   held-out 21 and 65: 8-10 m), BlueROV2 1 (71: 1.14 m, 77: 4.1 m), one run with a robot left out
   (41); no half-revolution flip remains. Sonar images: wrong alignments with 24-61 m frame errors
   or a robot left out, and one marginal run (seed 10, 1.01 m) with ground-truth tracks; the clutter
   landmarks of L36 with the EKF (T-F3-06).
4. Paper text updated to these data (`03894a5`): the reverse-estimate test in the method, the
   held-out G1 no longer "every run", the sonar paragraph and two tables, a sixth limitation.

---

## 2026-10-06 (luiz-predator-neo, later): why fresh seeds failed G1 (Claude)

Branch `wp/T-E3-02-heldout-gap`; task T-E3-02; **simulation**. Before: every perception on every
recorded seed at `d35ab54`, `alignment_acceptance_study.py --weak 0 --jobs 28 --seeds ...`
(rows in `results/heldout_gap_{ref,sonar}.jsonl`). Geometry: `experiments/gazebo/check_geometry.py`
on all 150 recordings. After: the 22 late recordings re-recorded at `7a10f5f` and the same study
at `7a10f5f` (`results/heldout_gap2_{ref,lidar,sonar}.jsonl`). Seed blocks:
`python experiments/heldout_gap.py results/heldout_gap2_*.jsonl`.

### L39. Fresh seeds were not harder: 22 of their recordings were one keyframe late

1. **At one commit the gap stays** (ground-truth ids, proxy: seeds 0-39 40/40, 40-79 31/40), but
   the distribution of the largest frame error per run does not move (median 0.48-0.53 m in every
   block of 20; Mann-Whitney p = 0.21): the failures are a tail. Six are the UAV (a wrong UGV-UAV
   alignment; its frame error p90 0.47 m on seeds 0-39, 1.22 m on 40-79, p = 0.012), three are
   BlueROV2 1 just above the gate (1.04-1.13 m). Tier 1 shows no difference at all.
2. **The UAV sees ghosts only in the failing seeds.** Spurious UAV detections occur in seeds 40, 41,
   58, 60 and 72 and in no seed below 40: clusters about 2 m up at fixed places, nowhere near a
   structure, once per UAV loop. Same world layout, same front-end: the recordings differ.
3. **22 recordings are one keyframe late** (all made on 2026-10-02 with several recordings in
   parallel, 50-100 s each instead of 16-25 s): reference fleet seeds 40, 41, 56, 58, 60, 72, 74;
   UAV-LiDAR fleet 40, 46, 47, 52, 54, 57, 60, 61, 64, 65, 67, 68, 70, 73, 75. Their returns farther
   than 0.2 m from any surface: UAV D435i 36-46 %, VLP-16 13-19 %, sonar proxy 47-63 %; every
   other recording ≤ 3.0 % (LiDAR, depth) and ≤ 8.2 % (sonar proxy). 95-99 % of their frames are
   bit-identical to the *previous* keyframe's frame of a clean re-recording, and their stamps were on
   schedule. They hold 7 of the 9 G1 failures with ground-truth ids of the reference fleet and all 8
   of the LiDAR fleet. DAVE's sonar recordings (their own driver, L35) are not affected: the share
   of sonar detections that match a part is the same in every seed group.
4. **Reproduced and fixed.** One extra `/world/<w>/control` step while recording
   (`gz service ... --req 'pause: true, multi_step: 1'`) makes the two-step recorder write every
   later frame one keyframe late, with on-schedule stamps; injected at start-up it does so for the
   original recorder and for one that waits for a quiescent world (`e99f0c3`, whose commit message
   names a wrong mechanism). With four steps per keyframe (`7a10f5f`) the same start-up injection
   gives a recording bit-identical to the clean one, and an injection while recording stops the
   recorder ("stamped 593000000 ns before keyframe 143, expected 592000000"). What sent the extra
   step on 2026-10-02 is not known: load alone did not reproduce it (four recordings in parallel
   under 24 CPU burners, or next to two DAVE sonar passes, were clean with the old recorder).
   `record.py` now keeps `raw.npz` only if the geometry check passes, and the Makefile loops stop
   at the first failure. Clean recordings are bit-identical with the old and the new recorder.
5. **Two cache holes closed on the way** (`5086dce`, `2b8fb20`): the detection cache was keyed by
   `frontend.py` and `sonar_image.py` only (not the EKF tracker, rays or scenario code) and not by
   `raw.npz`. No stored cache was stale (all rebuilt on 2026-10-02, after the tracker's last
   change), but a re-recorded run would have reused the late recording's detections.
6. **After re-recording, G1 is the same on old and fresh seeds** (passes per block of 20; LiDAR
   fleet 20-39 means 30-39; Fisher's test 0-39 against 40-79):

   | Perception | 0-19 | 20-39 | 40-59 | 60-79 | p |
   |---|---|---|---|---|---|
   | Tier 1 | 20 | 20 | 20 | 20 | 1.0 |
   | proxy, ground-truth ids | 20 | 20 | 19 | 19 | 0.49 |
   | proxy, EKF | 18 | 18 | 18 | 17 | 1.0 |
   | UAV-LiDAR fleet, ground-truth ids | 20 | 10/10 | 18 | 20 | 0.50 |
   | UAV-LiDAR fleet, EKF | 20 | 10/10 | 18 | 20 | 0.50 |
   | sonar, ground-truth ids | 18 | – | 16 | 17 | 0.70 |
   | sonar, EKF | 6 | – | 11 | 12 | 0.058 |

   The largest-frame-error distributions agree too (Mann-Whitney p = 0.75-0.86 for proxy and
   LiDAR fleet). The remaining failures with ground-truth ids on the proxy are BlueROV2 1 at
   1.12-1.13 m (seeds 45, 71); on the sonar, wrong alignments with 23-61 m frame errors or a robot
   left out of the team (one marginal, seed 10 at 1.01 m; T-F3-05/06). Sonar with the EKF does better on fresh seeds than on the development seeds it was
   studied on (p = 0.058); not interpreted here.
7. **Corrections.** L37 item 6 and L38 item 2 ("fresh seeds are harder") are wrong, and the held-out
   numbers of L37 (seeds 40-59) and L38 (60-79) rest on 7 late reference-fleet and 15 late
   LiDAR-fleet recordings: they are void. The paper data (seeds 0-39, recorded on the old host)
   are not affected: every one of those recordings passes the geometry check.

8. **The veto rule of T-F3-05 on clean held-out seeds** (40-79, threshold 8, `--weak 8 --policy
   veto` at `7a10f5f`, rows in `results/veto_clean_*.jsonl`, against threshold 0 in
   `heldout_gap2_*`): it never breaks a run and fixes two (proxy EKF seed 45: 35 -> 36/40; sonar
   EKF seed 40: 23 -> 24/40); merges, wrong alignments used and every other perception unchanged.
   With L37 (development seed 8 fixed, Tier 1 unchanged) it is safe but small.

**Recommendation for D13 (PI decides):** report Tier-2 G1 over every clean recording, seeds 0-79
(development 0-19, held out 20-79), from `heldout_gap2_*`; with ground-truth ids that is 78/80
(proxy), 68/70 (LiDAR fleet) and 51/60 (sonar). The veto rule (threshold 8) can be made the
default: no run got worse in any perception or seed set.

---

## 2026-10-06 (luiz-predator-neo): held-out seeds 60-79 for the alignment rule (Claude)

Branch `wp/T-F3-05-alignment-acceptance`; task T-F3-05; **simulation**. The evaluation announced
at the end of L37 ran on 2026-10-02 (rows in `results/alignment_acceptance_heldout3.jsonl`,
written after commit `a7ccf76`) and was not written up then. The rows do not record the policy;
re-running two perceptions with `--policy veto` reproduces all 80 of their rows exactly
(`alignment_acceptance_study.py --seeds 60 ... 79 --weak 0 8 --policy veto --configs "proxy EKF"
"sonar GT ids"`, 3 min with 28 jobs), so the file is the veto policy at code `cace54f`.

### L38. The veto rule changes nothing on seeds 60-79; fresh seeds fail G1 more often than seeds 0-39

G1 (merged) by threshold, policy "veto"; wrong alignments used, UGV-UAV / cross / BlueROV2 pair:

| Perception | 0 | 8 | Wrong, 0 | Wrong, 8 |
|---|---|---|---|---|
| Tier 1 | 20 (20) | 20 (20) | 0/40, 0/120, 0/40 | 0/40, 0/121, 0/40 |
| proxy, ground-truth ids | 16 (20) | 16 (20) | 4/40, 1/96, 0/40 | 4/40, 1/96, 0/40 |
| proxy, EKF | 14 (19) | 14 (19) | 6/37, 4/90, 0/40 | 6/37, 4/91, 0/40 |
| sonar, ground-truth ids | 14 (18) | 14 (18) | 1/35, 6/78, 0/39 | 2/37, 5/78, 0/39 |
| sonar, EKF | 10 (16) | 10 (16) | 8/36, 12/69, 15/40 | 6/34, 14/72, 15/40 |
| UAV-LiDAR fleet, ground-truth ids | 16 (20) | 16 (20) | 1/40, 0/102, 0/40 | same |
| UAV-LiDAR fleet, EKF | 12 (18) | 12 (18) | 4/36, 5/75, 0/40 | same |

Seeds that fail G1 (both thresholds): proxy ground-truth ids 60, 71, 72, 74; LiDAR fleet
ground-truth ids 60, 61, 65, 70. Median time to merge is the same at 0 and 8 in every row.

**Findings.**
1. **The veto rule is neutral on fresh seeds**: identical G1 and merges at 0 and 8 in all seven
   perceptions. Its measured benefit stays on the development seeds (seed 8, L37 item 8), and it
   costs nothing in Tier 1 (L37 item 9). It is **not** made the default here
   (`confirm_weak_inliers = 0` in `AvatarParams`): a rule with no held-out effect does not change
   the paper data on its own; the choice joins the analysis of item 2 (PI, 2026-10-06: what goes
   into the paper is decided once that analysis gives meaningful results, PLAN D13).
2. **Fresh seeds are harder, again.** With ground-truth ids on the proxy G1 holds in 40/40 runs
   of seeds 0-39 and in 31/40 of seeds 40-79 (15/20 and 16/20); Fisher's exact test p = 0.002.
   The LiDAR fleet with ground-truth ids: 16/20 on 40-59 and 16/20 on 60-79. Tier 1 holds 80/80.
   Ground-truth ids take the tracker out, so the gap is in perception geometry, the association
   and acceptance rules, or the seed's world, not in T-F3-02/03. Two things are not yet
   comparable: seeds 20-39 were last evaluated at an older commit (`paper/data/tier2_heldout.csv`),
   and the rules were developed while seeds 0-39 were in view. New task **T-E3-02** finds the
   cause before any paper number changes.

---

## 2026-10-02 (luiz-predator-neo, later): few-inlier alignment acceptance (Claude)

Branch `wp/T-F3-05-alignment-acceptance`; task T-F3-05; **simulation**. Development seeds 0-19:
`python experiments/alignment_acceptance_study.py --jobs 28` at commit `822f2c5` (rows in
`results/alignment_acceptance_dev.jsonl`). Held-out: new seeds 40-59 (below).

### L37. Weak frame estimates need the other direction of the pair: fixes the 4-5 inlier flips, not the sonar-EKF failures

**Rule** (`AvatarParams.confirm_weak_inliers`): a frame estimate with fewer inliers than the
threshold (own, received, or about this agent) is used only when the other direction of the same
pair exists and agrees (the 2-cycle test of the cycle check); weak own estimates are still
advertised so that the neighbour can confirm them. Wrong alignments below count only those used.

Development seeds 0-19, G1 (merged) by threshold (0 = off):

| Perception | 0 | 6 | **8** | 10 | 13 |
|---|---|---|---|---|---|
| Tier 1 | 20 (20) | 20 (20) | 20 (20) | 20 (20) | 20 (20) |
| proxy, ground-truth ids | 20 (20) | 20 (20) | 20 (20) | 19 (19) | 13 (13) |
| proxy, EKF | 18 (20) | 19 (20) | **19 (19)** | 17 (17) | 8 (8) |
| sonar, ground-truth ids | 18 (20) | 19 (20) | **19 (20)** | 18 (19) | 15 (15) |
| sonar, EKF | 6 (17) | 6 (17) | 6 (17) | 5 (8) | 4 (4) |
| UAV-LiDAR fleet, ground-truth ids | 20 (20) | 20 (20) | 20 (20) | 20 (20) | 16 (16) |
| UAV-LiDAR fleet, EKF | 20 (20) | 20 (20) | 20 (20) | 20 (20) | 15 (15) |

Wrong alignments used at threshold 8 against 0: proxy EKF UGV-UAV 1/34 against 3/38 (cross 4/96
both); sonar ground-truth ids cross 0/98 against 1/97; sonar EKF cross 21/78 against 21/79. The
median time to merge grows by 0-20 s at 8 (60-200 s at 13).

**Findings (development).**
1. **The 180-degree flips of L33 are fixed.** Seed 8 (proxy EKF): `ugv_0 <- uav_0` (4 inliers,
   wrong) and `uav_0 <- ugv_0` (4 inliers, right) disagree, so neither is used; the team merges
   through the BlueROV2s and G1 passes (team ATE 39.6 -> 0.27 m).
2. **No regression at 6 or 8**: Tier 1, ground-truth ids and the LiDAR fleet keep every merge;
   at 8 one proxy-EKF run merges no more (G1 unchanged, 19/20).
3. **Higher thresholds cost merges.** Many alignments have no reverse estimate (one agent aligns
   the pair, the other does not), so requiring one for 10-12 inliers leaves teams split.
4. **The sonar-EKF failures are not fixed** (6/20 at every threshold that keeps merges; wrong
   cross alignments 21/78 at 8): their wrong alignments have 8-11 inliers, above the threshold, and
   need better maps (T-F3-06: clutter landmarks) rather than this rule.

**Choice, before the held-out evaluation:** threshold **8**, the value of `min_inliers_cross_only`
(an alignment that would not pass the cross-only test needs its reverse); chosen on the
development seeds only. The held-out evaluation runs once on new seeds 40-59 (recorded for it,
not looked at before this entry was committed): thresholds 0 and 8, all seven perceptions,
`python experiments/alignment_acceptance_study.py --seeds 40 ... 59 --weak 0 8`.

**Held-out evaluation, new seeds 40-59** (recorded for it on 2026-10-02 with the container route,
reference fleet and UAV-LiDAR fleet, sonar for the reference fleet; run once after the entry above
was committed, `alignment_acceptance_study.py --seeds 40 ... 59 --weak 0 8`, rows in
`results/alignment_acceptance_heldout.jsonl`). G1 (merged), threshold 0 -> 8:

| Perception | 0 | 8 | Wrong used, 0 -> 8 (UGV-UAV) |
|---|---|---|---|
| Tier 1 | 20 (20) | 20 (20) | 0/40 -> 0/40 |
| proxy, ground-truth ids | 15 (19) | 15 (19) | 4/35 -> 4/34 |
| proxy, EKF | 15 (17) | **16** (17) | 4/32 -> 4/32 |
| sonar, ground-truth ids | 13 (18) | 13 (18) | 3/31 -> 3/30 |
| sonar, EKF | 9 (17) | 9 (16) | 7/27 -> 4/24 |
| UAV-LiDAR fleet, ground-truth ids | 16 (20) | 16 (20) | 1/40 -> 1/40 |
| UAV-LiDAR fleet, EKF | 13 (19) | 13 (19) | 6/39 -> 6/39 |

5. **The rule holds up on fresh seeds**: no regression anywhere, one more proxy-EKF run passes
   G1 (the default is decided below: item 8 and after).
6. **The fresh seeds are harder than all earlier ones, independently of the rule.** With
   ground-truth ids on the proxy G1 holds in 15/20 (seeds 0-19: 20/20; 20-39: 20/20); the LiDAR
   fleet with the EKF in 13/20 (seeds 0-19 and 30-39: 30/30). The recordings are not the cause: a
   new recording of seed 21 with today's container is bit-identical to the stored one for all
   five sensors. The proxy failures: seeds 40 and 41 involve UGV-UAV alignments of 7 inliers
   shifted by 2-5 m (UAV frame error 1.10 and 1.98 m; in seed 41 both directions agree with each
   other, so no 2-cycle test can see it), seeds 45 and 56 are marginal (a BlueROV2 at 1.12 and
   1.04 m), and seed 58 does not merge (one BlueROV2 missing, UAV at 1.73 m). Over
   all 60 seeds G1 with ground-truth ids is 55/60 (92 %), not "every run". PLAN G1 is updated;
   the paper still reports seeds 0-39 only, and adding seeds 40-59 to it is left to the PI
   (`make tier2-heldout2 tier2-lidar-heldout2` writes the data).
7. Correction to item 3 above: "many alignments have no reverse estimate" is an interpretation of
   the lost merges, not a measurement.

**Revision: the "confirm" policy delays merges in Tier 1.** Regenerating the paper data with it
(threshold 8; not committed) left Tier-1 G1 unchanged but slowed merging: the bandwidth sweep's
time to a common frame grew from 100-130 s to 120-160 s at 256-1024 bit/s and one scheduler
merged 7/10 instead of 8/10 runs at 32 bit/s; the server comparison at 1 kbit/s 90 -> 110 s. The
task asks for no regression in Tier 1, so "confirm" is not the default (`confirm_weak_inliers = 0`).

8. **A second policy, "veto"** (`confirm_weak_policy = "veto"`): a weak estimate is used unless
   the reverse estimate of the pair exists and disagrees; then neither is used. It does not wait.
   The reverse is looked up among all estimates, also one the cycle check rejected, which with
   equal support keeps an arbitrary one of two conflicting directions (seed 8). Development seeds,
   thresholds 6, 8 and 13 alike: proxy EKF G1 18 -> 19/20 (seed 8), sonar ground-truth ids
   18 -> 19/20, every other perception and every merge unchanged; median time to merge 0-20 s
   later (`alignment_acceptance_study.py --weak 6 8 13 --policy veto`, commit `cace54f`, rows in
   `results/alignment_acceptance_dev_veto.jsonl`).
9. **Tier 1 with "veto", threshold 8** (`make server cycle bandwidth drift realism
   DATA=../results/veto_t1` with that default, tables made from them): the server, bandwidth,
   drift and cycle-check tables are identical to the committed ones. The realism table changes in
   its no-kernel and Huber columns only, mostly for the better (default preset 15 % switches +
   1.0 clutter, Huber 0.76 -> 0.54 m; exploration 5 % + 0.5, no kernel 2.25 -> 1.46 m), with one
   cell worse (exploration, 15 % + 1.0, Huber: 0.75 m with 3/3 merged -> 0.64 m with 2/3); the
   GNC-TLS column the paper relies on is unchanged.

**Choice, before a second held-out evaluation:** policy "veto", threshold 8. Seeds 40-59 have been
looked at (above), so the evaluation runs once on new seeds 60-79 (both fleets and the sonar,
recorded on 2026-10-02 for it): `alignment_acceptance_study.py --seeds 60 ... 79 --weak 0 8
--policy veto`. (Some of the sonar passes ran while a temporary default was in the working tree,
so their `meta.json` label reads `cace54f-dirty`; the range-data recordings read `836b644`;
recording does not use the estimator.)

---

## 2026-10-02 (luiz-predator-neo): the EKF tracker on sonar images (Claude)

Branch `wp/T-F3-06-ekf-sonar`; task T-F3-06; development seeds 0-19 only (sonar recordings of
L35); **simulation**. Commands: `python experiments/sonar_tracker_study.py {alignments,clutter,
birth,confirm,tracks,minlen} --jobs 28` at commit `b33409c` (per-run rows in
`results/sonar_tracker_*.jsonl`);
`experiments/tier2_tracker_diagnostics.py <run> --mode ekf --sonar sonar`.

### L36. On sonar images the EKF tracker turns quay and hull echoes into landmarks, and few-inlier cross-medium alignments go wrong; three remedies fail

With ground-truth ids G1 holds in 18/20 runs on sonar images (L35); with the EKF tracker in 6/20.
Accepted alignments by type (wrong / accepted):

| Perception | Merged | G1 | Cross (BlueROV2 with UGV/UAV) | BlueROV2 pair | UGV-UAV |
|---|---|---|---|---|---|
| proxy, ground-truth ids | 20/20 | 20/20 | 0/100 | 0/40 | 1/39 |
| proxy, EKF | 20/20 | 18/20 | 4/96 | 0/40 | 3/38 |
| sonar, ground-truth ids | 20/20 | 18/20 | 3/99 | 0/40 | 1/39 |
| **sonar, EKF** | 17/20 | **6/20** | **22/80** | **13/40** | 2/38 |

Cross alignments of sonar-EKF by inliers: 8-9: 14/23 wrong, 10-12: 7/23, 13 or more: 1/34 (proxy
EKF: 1/18, 3/38, 0/40). The runs that fail G1 have frame errors of 2-55 m
(`paper/data/tier2_sonar.csv`, tier T2sekf).

**Findings.**
1. **The pile tracks are mostly good** (`tracks`): 971 tracks of at least four detections on real
   piles over 40 BlueROV2-runs (24 per run), centre error median 0.08 m; 31 piles split into more
   than one track and 47 tracks (5 %) mix in more than 10 % of another part's detections (the
   pile-spacing confusions of `tier2_tracker_diagnostics.py`, 5-6 m apart).
2. **Clutter becomes landmarks.** 1431 clutter tracks, 36 per BlueROV2 and run (the EKF keeps 83
   and 61 tracks per BlueROV2 on sonar, 28 and 36 on the proxy: medians in `paper/data/tier2*.csv`);
   61 % lie on the quay face and 38 % on hulls: real echoes of extended surfaces (specular glints
   that slide along the face; spread 0.49 m against 0.24 m for piles, elongation 3.6 against 1.8),
   short-lived (median 7 detections against 43 for piles).
   The EKF's static-birth test confirms 49 and 43 landmarks per BlueROV2 and rejects 1 and 1
   (proxy: confirms 23 and 27, rejects 18 and 34; medians, same files):
   with the sonar's 0.24 m detection sigma, which includes the 0.2 m centre error that is mostly a
   per-pile constant, a sliding glint passes. Removing the BlueROV2s' unmatched detections after
   tracking (a ground-truth control): BlueROV2-pair wrong alignments 13 -> 4/38, cross 22 -> 15/87,
   G1 6 -> 8/20.
3. **Without clutter, the tracker's maps still mislead few-inlier cross alignments.** With the
   unmatched detections removed, ground-truth ids are unchanged (18/20, cross 3/99), but the EKF
   keeps 15/87 wrong cross alignments: its split and impure pile tracks and its dropped detections
   are enough. The fatal ones have 8-11 inliers, where the acceptance rule (`min_inliers_cross_only`
   = 8) admits them. The cycle check cannot catch them: the UGV is a leaf, and `consistent_subset`
   keeps the first of two conflicting directions of a pair in order of inliers, so with equal
   support (L33, seed 8: 4 and 4) the wrong one can win.

**Negative results (remedies that do not work).**
- *Birth test for sonar agents* (`FrontEndParams.ekf_sonar`): 8 or 12 confirming frames, chi2 gate
  5.99, no floor, and combinations: G1 4-8/20; the best halves the clutter tracks (72 -> 37/run).
- *A minimum track length before release* (`minlen`, 8-24 detections, a post-hoc control): G1 7, 5,
  7 and 9/20; at 24 it removes 84 % of the clutter detections and 6 % of the pile ones, and the
  BlueROV2-pair wrong alignments fall from 13 to 1/36, but cross ones stay at 13/74.
- *Alignment confirmation* (`align_confirm` 2 or 3 agreeing attempts): sonar-EKF G1 8 and 7/20
  (wrong 11/58 and 9/48 cross); it costs merges everywhere (sonar, ground-truth ids: 18 -> 14/20).
  Wrong alignments persist across attempts; they are not random flips.

**Consequence.** Two levers remain: the acceptance of cross-medium alignments with few inliers
(T-F3-05), which imperfect maps expose, and clutter from extended surfaces, best rejected before
it becomes a landmark (the chain rule of L34 catches only frames with several glints). T-F3-06
waits for T-F3-05, which is next.

---

## 2026-10-01 (luiz-predator-neo): why the team did not merge on sonar images (Claude)

Branch `wp/T-I1-03-docker-tier2`; task T-F2-06; development seeds 0-19 only; **simulation**.
`paper/data/tier2_sonar.csv` records commit `58d11da`; the controls are
`results/sonar_diag_assoc.jsonl`, `results/sonar_diag_cand.jsonl` and
`results/sonar_diag_reasons_v2.jsonl` (`experiments/sonar_diagnostics.py`).

### L35. Two causes, both fixed: a tie in the association's candidate ranking and stale sonar frames; with ground-truth ids G1 is 18/20 on sonar images

L34 left the cause open. Two were found.

**1. The association ranked candidates by a size the sonar cannot measure.**
`candidate_pairs` keeps, for each own landmark, the 12 remote landmarks most similar in
footprint. Every sonar landmark carried the same nominal footprint (0.76 m), so the ranking was a
tie and dropped true partners at random. Controls (`sonar_diagnostics.py assoc`, ground-truth
ids, stale recordings of L34):

| Variant | Merged | G1 | Wrong |
|---|---|---|---|
| as in L34 | 2/20 | 2/20 | 3/85 |
| sonar landmarks get the **true** pile footprint | 20/20 | 18/20 | 1/185 |
| footprint ratio limit 1.5 -> 3 | 3/20 | 3/20 | 3/119 |
| no class or descriptor check | 2/20 | 2/20 | 3/81 |
| 30 (or 60) candidates per landmark instead of 12 | 16/20 | 14/20 | 2/122 |

A first look at the true alignment (seed 3): it had 8 or more inliers in 11 % of the sonar
attempts (proxy 38 %) and none of those was accepted. Fix (487748b, 58d11da): a footprint of
0 x 0 means "not measured" (wire format v0 §3); such pairs are neither size-checked nor ranked
by size (all kept); the agent averages a footprint only over the observations that measured it;
the sonar front-end reports 0 x 0; and both codecs send a *measured* footprint that would round
to 0 x 0 with its larger axis as one LSB, so no measured footprint changes meaning. A first
version without that last rule changed proxy results (a measured footprint below 0.125 m was
read as unmeasured; e.g. the development EKF mean team ATE 2.31 -> 0.84 m); it was not
committed as data. Golden vectors are unchanged; 11/11 C++ tests and 170 Python tests pass.

**2. Half of the sonar recordings were one keyframe stale.** With ground-truth ids the solo ATE
of one BlueROV2 stayed near 0.40 m in 10 of 20 seeds (proxy 0.07-0.11 m). Its detections had a
constant range bias of +0.50 m (seed 13: IQR +0.35..+0.62), the distance travelled per keyframe;
against the *previous* keyframe's pose the bias was +0.04 m. At 500 Hz the sonar rendered on the
first or the second of the two 1 ms iterations depending on its phase, and the first one renders
the old pose. At 1000 Hz with both iterations stepped together, the slow CUDA frame made the
sensors system skip frames. The driver now steps one iteration at a time, waits for its frame,
and keeps the second (`SONAR_UPDATE_RATE_HZ = 1000`, two frames per keyframe; 265 s per seed).
All 20 development seeds were re-recorded (with this code, from a copy of the tree, while the
paper data were regenerated). Range bias after the fix: +0.03 and -0.03 m (seed 13).

**Correction to L34 item 2.** The recordings are **not** bit-reproducible on the harbour world:
two recordings of seed 13 differ in 537 and 49 of 601 frames (mean 11 and 2.6 codes, about 5 and
1.2 dB, where they differ), although both are geometrically right (bias +0.02/+0.03 m, 1 % of
keyframes biased by more than 0.3 m in each). The three-frame smoke test was bit-identical; the
cause in the large world is not known. The estimator gives the same outcome on both (ground-truth
ids: frame errors 0.24/0.61 m against 0.22/0.56 m; EKF: the same failure). The stored recordings
are the reference data.

**Result (development seeds, `make -C experiments tier2-sonar`, commit 58d11da):**

| Perception | Merged | G1 | Wrong alignments |
|---|---|---|---|
| proxy, ground-truth ids (T2) | 20/20 | 20/20 | 1/179 |
| **sonar images, ground-truth ids (T2s)** | **20/20** | **18/20** | 4/178 |
| sonar, EKF tracker (T2sekf) | 17/20 | 6/20 | 37/158 |
| sonar, nearest-neighbour tracker (T2snn) | 12/20 | 2/20 | 71/134 |

Alignment attempts on sonar data: accepted 39 % (L34: 26 %; proxy 49 %), ambiguous 35 % (proxy
14 %), median inliers 9 (as the proxy). The two T2s failures: seed 8 (a 23 m wrong alignment,
the few-inlier flip of L33, T-F3-05) and seed 10 (1.01 m, at the gate).

**What it means.** With correct intra-agent identities, cross-medium association works on DAVE's
sonar images about as well as on the proxy (G1 18/20 against 20/20): L34's negative result was
two defects in the pipeline, not a property of sonar data. What does not work yet is the
realistic front-end on top: the EKF tracker, tuned on the proxy, makes catastrophic wrong
alignments on sonar data (G1 6/20; several 17-55 m frame errors; two seeds with 118-179
wrong-track events on one BlueROV2). That is the next task (T-F3-06). The held-out seeds 20-39
and the LiDAR fleet are still not recorded with the sonar.

Reproduce:
```
make -C experiments tier2-record-sonar          # 265 s per seed; or several in parallel
make -C experiments tier2-sonar JOBS=28
python experiments/sonar_diagnostics.py assoc --jobs 24   # item 1 ran on the recordings of L34, since replaced
```

---

## 2026-09-30 (luiz-predator-neo, later): DAVE's multibeam sonar (Claude)

Branch `wp/T-I1-03-docker-tier2`; container `avatar-dave` on the RTX 4070; **simulation**.
Development seeds 0-19 only; the held-out seeds 20-39 and the LiDAR fleet have **not** been
recorded with the sonar, so they stay clean for the detector as it is frozen.
`paper/data/tier2_sonar.csv` records its commit; the controls are `results/sonar_diag_*.jsonl`.

### L34. DAVE's multibeam sonar runs in the harbour; on its images the team does not merge (T-S2-05)

Setup: `docker/Dockerfile.dave` on top of `avatar-tier2`: CUDA 12.6 toolkit and DAVE
(`IOES-Lab/dave`) at `d2121a5b4457361e60106aaa029b0a448977d70e` (2026-09-08, the last ROS 2
Jazzy / Gazebo Harmonic commit; the next one moves DAVE to Lyrical / Jetty), building
`dave_interfaces`, `multibeam_sonar_system` (which builds the CUDA sensor `multibeam_sonar`)
and `dave_multibeam_sonar_demo` for `sm_89`. The BlueROV2 rigs get the Gemini 720s of
`docs/hardware.md` (720 kHz, 90 degrees, 128 beams, 8 mm range resolution, 30 m; the 20 degree
vertical aperture is unverified) in an **acoustic world**: only what is below the waterline
(structures clipped at z = 0, the quay face, the seabed), because DAVE has no water surface and
sound does not travel in air. A second pass over an existing recording
(`experiments/gazebo/record_sonar.py`, `sonar_driver.py`) steps the world one keyframe at a
time (`set_pose`, two 1 ms iterations, one frame per sonar) and stores each image as uint8 dB
(940 range rows of 3.2 cm by 129 beams; four raw rows averaged in dB) in `<run>/sonar.npz`.
Cost: 138 s per seed with two sonars and 601 keyframes on the RTX 4070 (143 s each with four
in parallel), 77 MB.

**What DAVE's sonar does, measured.**
1. **It works.** Headless, on the GPU, ROS 2 `.../sonar_image_raw` (`ProjectedSonarImage`,
   float32 dB) and gz-transport `.../point_cloud`. A box face at 5.50 m gives a peak at 5.52 m,
   a cylinder at 8.08 m and -17.4 degrees one at 8.10 m and -17.0 (`sonar_smoke`, PASS).
2. **Speckle was not reproducible, and is now.** With `blazingSonarImage` the seed is
   `time(NULL)`. `docker/patch_dave_seed.py` makes it the simulation time in ms plus a hash of
   the sensor name, and the driver steps to a fixed simulation time before keyframe 0 (ROS
   discovery makes the first frames go unheard). Two runs are bit-identical; consecutive frames
   differ (mean 3.8-4.4 dB). Header stamps are 0 or wall-clock: frames are matched by order.
3. **DAVE illuminates twice the vertical ray angles written in the SDF.** Found through a ring
   of clutter at 23.4 m in every frame: a flat seabed 8 m below the sonar gives its first echo at
   23-24 m with rays of +-10 degrees, at 12 m with +-20 degrees and beyond 30 m with +-6
   degrees, that is, at 8 m / sin(2 x angle) (predicted 23.4, 12.4, 38.5 m). At 4 m altitude:
   +-5 degrees gives 23-23.5 m (predicted 23.0), +-10 degrees 11.5-12 m (predicted 11.7). The SDF
   therefore holds a quarter of the vertical FOV on each side. Before the fix a frame had 17
   false detections; after it 3 (same detector). The azimuth angles are taken as written.
4. **What an image looks like** (seed 0, the piles in the fan). The median pile echo is 41-43 dB
   above the row-median noise floor at every range; the 10th percentile falls from 39 dB within
   14 m to 20 dB at 14-20 m and 3 dB beyond 20 m (occlusion, side lobes of a stronger echo).
   Around every strong echo there are arcs of constant range across the fan (azimuth side
   lobes); a hull or the quay gives a chain of glints along its face; the echo of a cylinder
   trails a few tenths of a metre behind its face (0.5 m for a pile of radius 0.53 m). Speckle
   and the side lobes are what the proxy did not have.
5. **Parallel recordings need separate ROS domains.** Containers on one docker network heard
   each other's sonar topics (same names) through DDS multicast: 4 of 4 parallel runs failed
   with inflated frame counts. `record_sonar.py` sets a ROS domain per run directory, on localhost.

**The detector** (`avatar/tier2/sonar_image.py`, task T-F2-03 v1; 9 unit tests): floor = median
over the beams of each range row; candidates = peaks more than 20 dB above it (non-maximum
suppression 0.6 m by 4 beams); rejected as a side lobe (a stronger echo more than 12 dB above at
the same range), as extended (the -6 dB region longer than 2.5 m), for low local contrast (the
90th percentile of the surroundings within 15 dB), or as part of a chain (two comparable peaks
within 3 m, unless the peak stands 6 dB above every neighbour); the face range is the leading edge
3 dB below the peak, the azimuth the power-weighted centroid; the centre is one nominal radius
(0.38 m) behind the face, a nominal pile footprint is reported (the image cannot measure a
diameter: the echo's range extent has correlation -0.10 with the radius), and 0.2 m centre error
goes into the covariance. Each rule and value was chosen on seeds 0 and 19 (development) against
the ground truth. On those seeds (204 frames): 2.2 pile detections per frame, 1.2 false ones (0.56
of them hull, quay or pipeline echoes), 55 % of the piles in the fan found (seed 0: 23, 63, 82
and 50 % at 0-8, 8-14, 14-20 and 20-29 m), face range error median +0.01 m (robust sd 0.15 m;
seed 0), cross-range error sd 0.12 m (seed 0), position error median 0.24 m, 6 ms per frame.

**Gate G1 on the sonar images (development seeds 0-19; `make -C experiments tier2-sonar`).**

| Perception | Merged | G1 | Wrong alignments |
|---|---|---|---|
| proxy, ground-truth ids (T2) | 20/20 | 20/20 | 1/179 |
| **sonar images, ground-truth ids (T2s)** | **2/20** | **2/20** | 3/85 |
| sonar, nearest-neighbour tracker (T2snn) | 1/20 | 0/20 | 37/83 |
| sonar, EKF tracker (T2sekf) | 1/20 | 0/20 | 19/93 |

**Findings** (controls: `make -C experiments sonar-diagnostics`, ground-truth ids, 20 seeds).
6. **With DAVE's sonar images the team does not merge, even with ground-truth ids.** The
   proxy's 20/20 overstated feasibility (R2 confirmed in direction). The UUVs' own SLAM is fine
   (solo ATE 0.10-0.11 m on seed 0, as with the proxy); what is missing is the link between a
   BlueROV2 and the UAV or UGV, which the team frame needs.
7. **The back end refuses the alignments as ambiguous.** Over all attempts: accepted 49 %
   (proxy) against 26 % (sonar); ambiguous 14 % against 40 %; too few inliers 23 % against 24 %;
   median inliers of an accepted alignment 9 against 7. Relaxing the ambiguity ratio (0.8 by
   default) on the sonar data: 0.9 gives 9/20 merged, G1 4/20, wrong 9/113; 0.95 gives 11/20,
   7/20, 10/116. The test does real work (wrong alignments triple) and even relaxed G1 is 7/20.
8. **Not position accuracy, spurious detections or the covariance.** The proxy's detections with
   independent 0.1 or 0.2 m noise on the UUVs still merge 20/20 (G1 19/20 and 18/20, wrong 1/179
   and 1/181). Sonar detections with the unmatched ones removed: 2/20 merged; with the matched ones
   moved to the true centre plus 0.1 m noise: 4/20 merged, G1 3/20; both: 4/20 and 3/20. On seed 0
   the centre error in the covariance from 0.05 to 0.3 m changes nothing.
9. **Nor, as far as tried, a more permissive detector.** Without the four rejection rules:
   1/20 merged (wrong 3/93); without them and with a 12 dB threshold: 1/20, G1 0/20; with the
   12 dB threshold alone: 2/20 (same as the default). So the hypothesis I held while writing this
   (too few shared piles seen at a time) is **not supported by this lever**; what differs from the
   proxy is not pinned down. Candidates not yet tested: which piles are detected when (side
   lobes hide weaker ones: a point-spread-function subtraction would test it), the association's
   size and class checks on sonar landmarks, and the time at which shared landmarks accumulate.
10. **Trackers on top are worse** (wrong alignments 37/83 and 19/93): they were tuned on the
    proxy and see the wall glints and side-lobe clutter; not analysed further here.

**Caveats (what this is not).** DAVE's speckle amplitude is attached to the ray index, not to the
surface point; one sonar model with DAVE's default source level and gain (the detection range is
not calibrated to a real Gemini); no water surface, multipath or bottom reverberation
variability; the vertical aperture is unverified; the detector is a first design tuned on
two of the development seeds; the held-out seeds and the LiDAR fleet are not recorded yet. The
result is therefore "G1 is not met by this sonar, this detector and this back end", not
"cross-medium association fails on sonar".

Reproduce (in the `avatar-dave` image; the recordings of `results/tier2` are `make tier2-record`):
```
docker build -f docker/Dockerfile.tier2 -t avatar-tier2 . && docker build -f docker/Dockerfile.dave --build-arg CUDA_ARCH=89 -t avatar-dave .
make -C experiments tier2-record-sonar          # about 2 min per seed; one at a time, or four in parallel
make -C experiments tier2-sonar sonar-diagnostics JOBS=24
```

---

## 2026-09-30 (luiz-predator-neo): the paper data in the pinned environment (Claude)

Branch `wp/T-I1-03-docker-tier2`; new host (i9-14900HX, 32 threads; RTX 4070 8 GB; Docker
with the NVIDIA toolkit; 333 GB free); Tier 1 and Tier 2, **simulation**.
`paper/data/*.csv` record commit `4f380e1` (clean); `paper/data/environment.txt` records
the versions (Python 3.12.3, NumPy 2.5.3, SciPy 1.18.1).

### L33. Results depend on the NumPy version: Tier 1 is unchanged, Tier 2 loses two seeds to a fragile alignment decision

**Environment.** On this host the Tier-2 numbers differed from the old host's. The
dependence was traced to the NumPy major version (1.26 there, 2.x here); the SciPy version
and the hardware do not change them. Fixes: `avatar_py/requirements-lock.txt` (pip freeze
of the venv), NumPy and SciPy versions in the front-end cache key
(`avatar/tier2/dataset.py`), `environment.txt` written by `make_paper_tables.py`, and
`AGENTS.md` §6 installing from the lock file. The CI fixture and its golden values, made
under NumPy 1.26, still pass under 2.5.3 (158 tests). Then every study was regenerated
(28 jobs, 64 min).

Tier-2 results, old (NumPy 1.26) -> new (2.5.3). G1 = runs with every frame error < 1 m;
wrong = accepted alignments above 2 m or 5 degrees, of all accepted:

| Tracker | Dev seeds 0-19, G1 | wrong | Held-out seeds 20-39, G1 | wrong |
|---|---|---|---|---|
| ground-truth tracks | 20/20 -> 20/20 | 1/184 -> 1/179 | 20/20 -> 20/20 | 0/188 -> 0/183 |
| NN (dead reckoning) | 10/20 -> 8/20 | 44/167 -> 40/162 | 9/20 -> 11/20 | 40/175 -> 39/174 |
| registration | 5/20 -> 4/20 | 71/152 -> 63/148 | not run | |
| **EKF** | **19/20 -> 18/20** | 8/181 -> 7/174 | **19/20 -> 18/20** | 6/183 -> 7/178 |
| EKF, UAV LiDAR fleet (30 seeds: 0-19, 30-39) | 30/30 -> 30/30 | 1/276 -> 1/271 | | |

Mean team ATE of the EKF tracker: 0.35 -> 2.31 m (dev), 0.27 -> 2.27 m (held-out). The
medians are 0.219 m and 0.247 m (ground-truth tracks: 0.207 and 0.246 m).

**Findings.**
1. **The recorder is reproducible across GPUs.** 30 s of seed 0 recorded in the container
   on the RTX 4070 equals the recording made on the old RTX 3050 for all five sensors
   (100 % of the returns agree, maximum |dr| = 0).
2. **Tier 1 is unchanged.** Server, bandwidth, drift and cycle-check data differ by at
   most 1.1e-7 in relative terms (the cycle check is identical). The realism table
   (3 seeds) keeps its conclusions (L32): the robust-kernel columns move by at most
   0.2 m (all revisits split, default preset, GNC-TLS: 1.24 -> 1.44 m); only the
   no-kernel column at 15 % random switches and 1.0 clutter worsens a lot
   (3.14 -> 12 m, the team merging in 0/3 runs both times).
3. **Two seeds that used to pass now fail catastrophically (dev seed 8, held-out
   seed 28).** The front-end output is identical (seed 8: 7878 detections, the same
   clusters per agent, solo ATEs equal to 1e-14) but the back end accepts a wrong
   pairwise alignment `ugv_0 <- uav_0`: 4 inliers, 58.5 m and 177 degrees off (seed 28:
   5 inliers, 42.3 m, 179 degrees). The reverse alignment `uav_0 <- ugv_0` is right
   (1.1 m and 0.3 m). `ugv_0` is the anchor, so the UAV and both BlueROV2s inherit the
   error (frame error 58-59 m and 42-46 m; team ATE 39.5 and 40.0 m).
   Nothing vetoes it: `min_inliers` is 4, and the cycle check (T-X2-02) compares cycles
   of the team frame graph, but `ugv_0` is a leaf attached only to `uav_0`. Why a
   perturbation of 1e-14 selects the flipped solution is **not yet known**; the guess is
   a near tie between the true transform and its 180-degree mirror on 4-5 landmarks
   (T-F3-05).
4. **The other two failures are unchanged** (seeds 19 and 21, 9.6 m): the UAV's frame is
   about 10 m and 10 degrees off against every partner, the aliasing that the LiDAR fleet
   removes (L31). The LiDAR result stands (30/30, 1 wrong alignment in 271).
5. **What this does to the claims.** EKF G1 is 36/40 (90 %), not 38/40. The count of a
   20-seed study moves by about two runs between environments (NN 10 -> 8 and 9 -> 11),
   so the EKF-versus-NN gap (18 against 8-11) is larger than that noise, while the
   EKF-versus-ground-truth gap (18 against 20) is not larger. The failure has a second
   mechanism (few-inlier alignments involving the UAV), not only the duplicate landmarks
   of L29-L30; the paper's "the runs in which it fails involve the Tarot" is right, but
   its reason was incomplete. Means of team ATE are dominated by one run each; the paper
   now also gives medians.
6. **Wording errors I made, corrected in the paper.** "No accepted alignment is wrong on
   the development seeds (1/184 wrong)" contradicted itself; it now says how many are
   wrong.
7. **A trap in the Makefile.** `make paper-data tier2 tier2-heldout tier2-lidar tables`
   built the tables before the Tier-2 CSVs were rewritten (`paper-data` already lists
   `tables`, so the last goal was a no-op), and the Tier-2 macros were stale until
   `make tables` was run again. Fixed with `make all-data`.

Reproduce (Python environment of `requirements-lock.txt`, recordings in `results/`):
```
pip install -r avatar_py/requirements-lock.txt -e "avatar_py[dev]"
make -C experiments all-data JOBS=28
```
Follow-ups: T-F3-05 (few-inlier alignment acceptance; needs fresh held-out seeds, since
seeds 20-39 have been looked at), T-S1-10 (realism study on 20 seeds).

---

## 2026-09-30 (night): structured association errors in Tier 1 (Claude)

Branch `wp/T-S1-09-structured-errors`; Tier 1, `harbor_fleet`, presets `fleet_default`
and `fleet_exploration`, 3 seeds, 600 s, M64; **simulation**. `paper/data/realism.csv`
records commit `4306589` (clean).

### L32. Bursts of wrong identities are absorbed by GNC; missing loop closures are what hurts (T-S1-09)

L28 item 7 guessed that Tier 1's random identity switches "probably understate the
danger" of the persistent errors seen in Tier 2. Two error models were added to
`FrontEndErrors` to test it (both off by default; the existing streams and results
are untouched, a test checks it):
- **Bursts** (`id_switch_persist_kf` = 25): a switch lasts 25 keyframes for that sensor
  and part. Onsets 0.003 and 0.010 give 4.7 % and about 13 % wrong attributions,
  matching the random levels (4.3 % and 13.1 %, measured on `harbor_fleet`).
- **Splits** (`id_split_prob`, `id_split_gap_kf` = 30): a part seen again after 30
  keyframes gets a new identity, so the revisit closes no loop. This is what cost the
  Tier-2 trackers their loop closures (L29).

Team ATE [m], mean of 3 seeds (in parentheses: runs in which the whole team merged, if
not all); the last two columns are the robust kernels:

| Preset | Front-end errors | No kernel | Huber | GNC-TLS |
|---|---|---|---|---|
| default | none | 0.10 | 0.10 | 0.11 |
| | 5 % random + 0.5 clutter | 2.06 (1/3) | 0.34 | 0.10 |
| | 5 % in bursts + 0.5 | 1.22 | 0.27 | 0.11 |
| | 15 % random + 1.0 | 3.14 (0/3) | 0.76 (2/3) | 0.12 |
| | 13 % in bursts + 1.0 | 1.56 (1/3) | 0.20 | 0.10 |
| | half of the revisits split | 0.11 | 0.11 | 0.11 |
| | all revisits split | 1.30 | 1.29 | 1.24 |
| exploration | none | 0.13 | 0.13 | 0.13 |
| | 13 % in bursts + 1.0 | 1.31 (1/3) | 0.51 | 0.13 |
| | all revisits split | 1.25 | 1.24 | 1.12 |

```
make -C experiments realism tables PY=python JOBS=12
```

**Findings.**
1. **The L28 hypothesis is not supported.** At the same rate of wrong attributions,
   bursts of 25 keyframes are absorbed by GNC-TLS as well as random switches are (team
   0.10-0.13 m against 0.11-0.13 m error-free). Huber is not always enough
   (0.51 m on the exploration preset with 13 % bursts). Tier 1's random-switch model
   did not understate the danger of persistent wrong identities.
2. **Missing loop closures are the error that matters, and no kernel helps.** With
   every revisit split, the UAV alone goes from 0.14 to 2.38 m and BlueROV2 1 from 0.10 to
   1.47 m (default preset, GNC), the team from 0.11 to 1.24 m. Half of the revisits
   split costs nothing (team 0.11 m): one recognised revisit closes the loop. This
   matches Tier 2, where the lost accuracy came from duplicate landmarks (L29 item 4).
3. **Consequence for the claims.** H1's drift correction and the Tier-1 team results
   assume that at least some revisits are recognised; the discussion says so. The
   robust-kernel result (Table `tab:realism`) stands.
4. **Reproducibility caveat.** Regenerating `realism.csv` on this machine changes the
   previously committed rows slightly: the committed file came from another software
   environment (old and current code give identical results here, checked on two
   cases). The Huber and GNC columns are unchanged to rounding; the no-kernel 15 %
   cell moved from 12 m to 3.1 m (a total failure either way). The other Tier-1 CSVs
   (server, bandwidth, drift, cycle check) were not regenerated and may shift the
   same way.

**Caveats.** Tier 1 only; 3 seeds; two presets; one burst length (25) and one gap
(30 keyframes); splits are random per revisit, whereas the Tier-2 tracker's were
concentrated where its pose uncertainty was high.

---

## 2026-09-30 (evening): the UAV's aliasing is a sensor problem (Claude)

Branch `wp/T-F3-04-uav-aliasing`; `harbor_fleet`, 600 s, M64; **simulation**. The
held-out set is now seeds 20-39 (20-29 were analysed after their run, 30-39 were run
once and looked at only afterwards); `tier2_heldout.csv` and `tier2_uavlidar.csv`
record commit `2a23a57` (clean), `tier2.csv` is unchanged (`0135c61`).

### L31. A LiDAR on the Tarot removes the residual aliasing failure (T-F3-04, decision D6)

After L30 the only recurring failure was the Tarot alone at 1.6-1.7 m (seeds 19 and
21). **Why software will not fix it:** its filter is consistent (L30 item 3), it
sees one pile per keyframe so there is nothing to pair jointly, and the wrong match
is allowed by the map's own uncertainty: two landmarks mapped a hundred keyframes
apart are only known relative to each other to about 2 m, so a pile 2.8 m from one
looks like it. A window of keyframes would not add that information (reasoned, not
implemented). A stricter match rule trades this failure for lost matches (L29
item 5). Instance descriptors would probably separate the piles in the sim (cosine to
the own pile 0.93, to another pile 0.49 on average) but the sim's descriptor model
is not validated against real embeddings, so that would be an upper bound; not tried.

**Test:** a fleet variant with a VLP-16-class LiDAR on the Tarot next to its D435i
(`harbor_fleet(uav_lidar=True)`, a stand-in for the optional Livox-class unit of
D6; `docs/hardware.md`). Its returns pass the geometry check (median 1.9 cm, p95 17
cm, 2.8 % beyond 0.2 m). The tracker and its parameters are unchanged. Seeds 0-19
and 30-39 (30 runs).

| Fleet, perception | Runs | G1 | Wrong alignments | Team ATE [m] |
|---|---|---|---|---|
| Camera only, ground-truth tracks (held-out 20-39) | 20 | 20/20 | 0/188 | 0.233 ± 0.019 |
| Camera only, NN tracker | 20 | 9/20 | 40/175 | 3.102 ± 1.927 |
| **Camera only, EKF tracker** | 20 | 19/20 | 6/183 | 0.275 ± 0.072 |
| LiDAR on the Tarot, ground-truth tracks | 30 | 30/30 | 0/283 | 0.189 ± 0.015 |
| LiDAR on the Tarot, NN tracker | 30 | 23/30 | 30/247 | 2.399 ± 0.624 |
| **LiDAR on the Tarot, EKF tracker** | 30 | **30/30** | **1/276** | 0.260 ± 0.132 |

```
make -C experiments tier2-record-lidar tier2-heldout tier2-lidar tables PY=python JOBS=12
```

**Findings.**
1. **The Tarot goes from the weak link to the best mapper**: alone, 0.10 m in all 30
   runs (max 0.11 m), against 0.23-0.24 m mean with one run above 1.5 m in each of
   the camera-only sets (development seed 19, held-out seed 21). It sees many piles per
   keyframe (7 323 matched detections on seed 19 instead of 217), so joint pairing
   works for it.
2. **The team improves as well**: with ground-truth tracks the team ATE is 0.189 m
   against 0.233 m, and the NN tracker also does better (23/30 against 9/20), so the
   LiDAR helps more than the tracker.
3. **One failure is left, and it moved to an AUV**: seed 33, BlueROV2 1 alone at
   3.29 m and one wrong alignment. Same mode as development seed 6 (3.02 m): a
   confident wrong match of a single far detection (28 m) that rotates the pose by
   about 5° (L29 item 5). Camera-only fleet: development seed 6 and none in the 20 held-out
   seeds; LiDAR fleet: seed 33 (1 in 30).
4. **Recommendation for D6 (the PI decides):** the light LiDAR removes the tracker's
   only recurrent failure in simulation and improves the team even with perfect
   tracks. Check first: payload and endurance of the Tarot 680 Pro (1.5-2.5 kg, about
   15 min; a VLP-16 is 0.83 kg, a Mid-360 class unit is lighter), power, and that a
   LiDAR in the propwash and near steel piles behaves like the model.

**Caveats.** The LiDAR is modelled as a VLP-16 (360° × 30°, 100 m), not a Livox
Mid-360 (specs UNVERIFIED); the LiDAR and camera-only runs are not paired (the extra
sensor changes later random draws, so odometry noise differs); 30 and 20 seeds; one
scenario and the sonar proxy of ADR-0007.

---

## 2026-09-30 (later): joint pairing and validation on fresh seeds (Claude)

Branch `wp/T-F3-03-joint-association`; `harbor_fleet`, 600 s, M64; **simulation**.
**Seed sets redefined:** seeds 0-19 are now the development set (seeds 10-19 informed
T-F3-03, so they are no longer held out) and seeds 20-29 the held-out set, recorded
after the design was fixed and used once. `paper/data/tier2.csv` (seeds 0-19) and
`tier2_heldout.csv` (20-29) record commit `0135c61` (clean) and replace the files of
L29; the ablation figures below replace the 10-seed ones of L28.

### L30. Pairing a frame's ambiguous detections jointly: G1 9/10 on fresh seeds (T-F3-03)

L29 left two failures on the then-held-out seeds. One was BlueROV2 1 on seed 12
(3.13 m, **no wrong tracks**, but 324 detections dropped as ambiguous between
landmarks 5.9 m apart, the pile spacing): with a pose σ of a metre, a single
detection cannot tell adjacent piles apart, but the relative geometry of the piles
in one frame is known to centimetres. `EkfTracker` now pairs the individually
ambiguous detections of a keyframe **jointly**: branch and bound over the joint
likelihood under the full state covariance (JCBB), "new landmark" as one option
per detection, accepted when the best pairing beats the runner-up by 3 nats
(`joint_pairing`, `joint_margin_ll`). Two unit tests: an irregular row is
resolved, a perfectly regular row stays a tie.

| Perception | Dev G1 (20 seeds) | Dev wrong | Dev team ATE [m] | Held-out G1 (10 seeds) | Held-out wrong | Held-out team ATE [m] |
|---|---|---|---|---|---|---|
| Tier 1 | 20/20 | 1/200 | 0.133 ± 0.015 | 10/10 | 0/100 | 0.133 ± 0.023 |
| Tier 2, ground-truth tracks | 20/20 | 1/184 | 0.235 ± 0.020 | 10/10 | 0/94 | 0.238 ± 0.030 |
| Tier 2, NN tracker | 10/20 | 44/167 | 2.221 ± 0.645 | 6/10 | 16/91 | 3.727 ± 4.122 |
| **Tier 2, EKF tracker + joint pairing** | **19/20** | 8/181 | 0.352 ± 0.200 | **9/10** | 6/92 | 0.304 ± 0.154 |

Development seeds only (20 seeds): registration 5/20 (71/152 wrong, 19/20 merged);
NN on the BlueROV2s alone 9/20 (36/167); NN on the Husky and Tarot alone 16/20 (16/185).

```
make -C experiments tier2-record tier2 tables PY=python JOBS=12
```

**Findings.**
1. **Joint pairing fixes what it targets.** BlueROV2 1 on seed 12: 3.13 m →
   0.28 m, drops 349 → 41, no wrong track. On seeds 0-9 nothing changes (the
   frames it acts on do not occur, or agree with the single-detection decisions:
   0.417 m, 1/89, G1 10/10 before and after); on seeds 10-19, G1 8/10 → 9/10, team ATE
   0.531 → 0.286 m, wrong alignments 8/92 → 7/92.
2. **Fresh seeds confirm it.** Seeds 20-29, nothing tuned on them: G1 9/10 (NN
   tracker 6/10, ground-truth tracks 10/10), team ATE 0.304 m against 0.238 m with
   ground-truth tracks. The Husky (0.16 m), BlueROV2 0 (0.11 m) and BlueROV2 1
   (0.17 m) are at their ground-truth-track values; the Tarot's mean is 0.32 m
   (median 0.16 m, one run at 1.71 m).
3. **The remaining failure is the UAV, every time**: seed 19 (development) and
   seed 21 (held-out) each have one run in which the UAV alone is at 1.6-1.7 m and
   most alignments involving it are wrong (6 of 9 on seed 21). Its filter is
   **consistent**: with ground-truth association the NEES (2 dof) is 0-8 on both seeds
   (ideal 2), and its honest position σ is about 2 m between the rare fixes its
   sparse depth camera gives, while the true error reaches 6-8 m (heading error of
   9-11° after about 120 m). At keyframe 104 of seed 19 a detection of a pile on the
   opposite row lay 2.8 m from another pile's landmark in the filter's frame
   (σ_y = 1.8 m, yaw σ = 4.6°), was matched with a comfortable margin over "new", and
   moved the pose by 2.3 m; the heading error reached 17° by keyframe 296. One
   detection per keyframe leaves nothing to pair jointly. Not a bug: a Bayesian
   tracker at that uncertainty will sometimes take a new pile for a mapped one.
4. **Acceptance of T-F3-02 / T-F3-03** (G1 >= 9/10, at most one wrong alignment):
   G1 is met on the held-out seeds (9/10) and on the development seeds (19/20);
   the wrong-alignment target is not (6/92 held-out, 8/181 development), and all of
   them involve the UAV.

**Caveats.** One scenario and one sonar proxy (ADR-0007); 30 seeds; the failing UAV
runs are 2 of 30; the ekf tracker uses the platform's bias priors (datasheet values);
Tier 2 keeps Tier-1 odometry. Seeds 20-29 have now been looked at (per-seed
breakdown, UAV diagnosis); a further change needs seeds 30-39.

---

## 2026-09-30: EKF-SLAM tracker for Tier 2 (Claude)

Branch `wp/T-F3-02-agent-association`; `harbor_fleet`, 600 s, M64; all **simulation**
(Tier 2 = Gazebo, kinematic rigs, sonar proxy, ADR-0007). Seeds 0-9 are the
**development set**: every design decision below was made on them. Seeds 10-19 were
recorded afterwards (`make -C experiments tier2-record`, same geometry check) and
serve validation, with the default fixed beforehand. Two more density values were
run on them as a sensitivity check, and the two failing seeds were analysed
afterwards, so any further change needs a third set. `paper/data/tier2.csv` and
`tier2_heldout.csv` record commit `fcee201` (clean).

### L29. A joint-covariance EKF-SLAM tracker takes G1 from 4/10 to 8/10 on fresh seeds (T-F3-02)

`avatar.frontend.ekf_tracker` associates detections against the agent's own filtered
estimate: an EKF over pose, odometry biases (heading, translation scale, gyro scale;
priors from the platform spec) and landmark positions, with the **joint** covariance.
Gate: Mahalanobis distance of `m_j - q(x)`, a match must beat the runner-up and a
"new landmark" hypothesis (`landmark_density_per_m2` = 0.005), ambiguous detections
between landmarks are dropped, tentative landmarks must prove static before they
may inform the bias states, detections are released once a landmark is confirmed or
seen three times.

| Perception | Dev G1 | Dev wrong | Dev team ATE [m] | Held-out G1 | Held-out wrong | Held-out team ATE [m] |
|---|---|---|---|---|---|---|
| Tier 1 | 10/10 | 1/100 | 0.119 ± 0.016 | 10/10 | 0/100 | 0.146 ± 0.024 |
| Tier 2, ground-truth tracks | 10/10 | 0/95 | 0.249 ± 0.032 | 10/10 | 1/89 | 0.220 ± 0.028 |
| Tier 2, NN tracker (L28) | 6/10 | 21/83 | 2.375 ± 0.883 | 4/10 | 23/84 | 2.068 ± 1.117 |
| **Tier 2, EKF tracker** | **10/10** | **1/89** | 0.417 ± 0.417 | **8/10** | **8/92** | 0.531 ± 0.570 |

Per-robot ATE alone [m] (development / held-out; ground-truth tracks in parentheses):
Husky 0.17 / 0.16 (0.17 / 0.16), Tarot 0.16 / 0.30 (0.16 / 0.15), BlueROV2 0
0.11 / 0.09 (0.11 / 0.09), BlueROV2 1 0.42 / 0.44 (0.14 / 0.15).

```
make -C experiments tier2-record tier2 tables PY=python JOBS=12     # both seed sets
python experiments/tier2_ekf_sweep.py uuv_1 0 1 2 3 4 -- landmark_density_per_m2=0.01,0.005,0.003
```

**How it got there** (development seeds, team level; the first three rows were run
at `a6a9d12` plus uncommitted changes, so they carry no clean provenance):

| Variant | Merged | Wrong alignments | G1 | Team ATE mean [m] |
|---|---|---|---|---|
| v0: gating with independent pose and landmark covariances | 10/10 | 11/90 | 6/10 | 0.952 |
| v1: + landmarks must prove static before they correct the pose | 0/10 | 15/34 | 0/10 | 0.480 |
| v2: + match vs. new-landmark likelihood, weak tentative updates | 4/10 | 16/59 | 3/10 | 4.335 |
| v3: joint covariance (the default) | 10/10 | 1/89 | 10/10 | 0.417 |

**Findings.**
1. **The filter model is sound.** With ground-truth association
   (`tracking="ekf_truth"`, a diagnostic) BlueROV2 1's estimated scale error is
   +0.0058 against a true +0.0052, and the position error stays at 0.1-0.5 m.
2. **A sliding clutter cluster broke v0, not the association.** On seed 0,
   BlueROV2 1 sees a cluster at exactly 28.9 m in its body frame at every keyframe
   (ground-truth label: clutter; a wall seen along the track). Taken for a static
   landmark, the only way to reconcile it with the odometry was a scale estimate of
   +3.2 % (true +0.5 %), and the pose error grew to 7.2 m against a claimed σ of
   0.07 m. The static birth test fixes it (a unit test covers it). Tuning could not:
   sweeping the measurement floor, odometry inflation and ambiguity margin moved
   BlueROV2 1's error between 0.07 and 5 m for neighbouring settings.
3. **v1 starved the UAV**: its depth camera sees a pile for one to three keyframes, so
   nothing confirmed and its pose was never corrected. v2 (weak updates from
   tentative landmarks, and a match that must beat "new") fixed the UAV but
   BlueROV2 1 stayed at 0.9-1.4 m.
4. **The real cause was the independent-landmark covariance.** With ground-truth
   identities, BlueROV2 1 still ended with 1.6-1.7 landmarks per part on the failing
   seeds (1.0 on the good seed): the gate rejected true revisits, because a filter
   that treats landmarks as independent reports the small *absolute* pose σ, not the
   drift since the landmark was mapped. Each duplicate cost the local SLAM a loop
   closure (about 2 m of error with 1-2 wrong tracks). Keeping the cross-covariances
   (state augmentation) fixed it: on seeds 0-4, all four robots reach their
   ground-truth-track accuracy over a plateau of settings (BlueROV2 1 0.14 m, UAV
   0.14 m, BlueROV2 0 0.11 m, Husky 0.16 m), with the corner (density 0.03,
   margin 1.0) still failing for the UAV.
5. **The plateau is narrower than seeds 0-4 suggested.** On seeds 5-9, which I had not
   looked at when picking density 0.003, BlueROV2 1 had one seed with
   245 wrong tracks and 3.0 m error: a new pile at 28.5 m, 6 m along the row from a
   landmark mapped at the start, looked like a re-detection, and that one match
   rotated the pose by 5°. Density 0.01 avoids it but costs the UAV (0.36 m instead
   of 0.18 m). Team level, development seeds: density 0.01 G1 8/10, **0.005 10/10**,
   0.003 9/10 (wrong alignments 1/91, 1/89, 1/86). Held-out: 0.01 7/10 (10/90
   wrong), **0.005 8/10 (8/92)**, 0.003 7/10 (14/93). 0.005 is the middle of the
   range, fixed before the held-out run; the in-sample 10/10 was optimistic.
6. **Residual failures on the held-out seeds.** Seed 12 (team ATE 2.74 m): BlueROV2 1
   alone at 3.13 m with **no wrong tracks** but 324 of about 1 170 detections dropped as
   ambiguous, all between landmarks 5.9 m apart (the pile spacing), so its pose was
   never corrected; its true scale error is -1.6 % (1.6σ of the prior). Seed 19
   (0.81 m, 6 of 10 alignments wrong): the UAV alone at 1.57 m with 19 wrong tracks
   of 217 matches, again aliasing between adjacent piles. No single detection
   separates adjacent piles when the pose σ is a metre or more. A frame-level
   joint pairing (JCBB, or a multi-hypothesis tracker) should, since the relative
   geometry of the piles in one frame is known to centimetres: task T-F3-03.
7. **Acceptance of T-F3-02 (G1 >= 9/10, no wrong alignment, BlueROV2 <= 2x the
   ground-truth value): met on the development seeds, not on the held-out ones**
   (G1 8/10, 8 wrong alignments of 92).

**Caveats.** One scenario, one sonar proxy (ADR-0007), 20 seeds; parameters were
chosen on the development seeds; the ekf tracker uses the platform's bias priors
(datasheet values), which the graph estimator does not use for the scale error;
Tier 2 keeps Tier-1 odometry.

---

## 2026-09-29: Tier 2 (Gazebo) v0 and the limits of a dead-reckoning tracker (Claude)

All runs: `harbor_fleet`, 600 s, seeds 0-9, M64, **simulation**. Tier 2 means
Gazebo Harmonic with kinematic sensor rigs and a ray-cast sonar proxy (ADR-0007),
not DAVE, PX4 or Clearpath. `paper/data/tier2.csv` records commit `b4681ec`
(clean).

### L28. Tier 2 v0: geometry is enough for G1 with ground-truth tracks; realistic tracking is not there yet

**Pipeline** (`experiments/gazebo/README.md`, ADR-0007): the Tier-1 scenario
becomes an SDF world; kinematic rigs carrying VLP-16, D435i (depth) and a
Gemini-720s proxy (128 x 16 rays, 90° x 20°, 50 m) are set to the Tier-1
ground-truth pose at every keyframe and their ranges are recorded; a geometric
front-end turns them into detections; Tier-1 odometry, depth and ground truth
(same seed) complete the `SimData`. Tier 1 and Tier 2 of one seed therefore
differ only in perception.

**Recording check** (`check_geometry.py`, seed 0; the other nine agree within
about 10 %): returns projected with the ground-truth pose lie a median of 8 mm
(VLP-16, p95 6 cm), 0.1 mm (D435i), 20 / 40 mm (Gemini proxy, BlueROV2 0 / 1)
from a scene primitive. The proxy has the largest residuals (p95 14 / 22 cm;
2 % / 7 % of returns farther than 0.2 m from any primitive); I have not
explained them. The recorder's README credits this check with catching a
pose/scan off-by-one in an early recorder; that recorder no longer exists, so
I did not re-verify it.

**Parity and gate G1** (10 seeds; mean ± 95 % t-interval; "wrong" = accepted
alignment with frame error > 2 m or 5°; G1 = frame error of every robot in the
anchor's frame < 1 m; "oracle" = centralized, ground-truth association, GNC):

| Perception | Team merged | At [s] | Wrong | G1 | Avatar team ATE [m] | Oracle [m] |
|---|---|---|---|---|---|---|
| Tier 1 (abstract) | 10/10 | 280 | 1/100 | 10/10 | 0.119 ± 0.016 | 0.084 ± 0.006 |
| Tier 2, ground-truth tracks (`oracle`) | 10/10 | 400 | 0/95 | 10/10 | 0.249 ± 0.032 | 0.184 ± 0.015 |
| Tier 2, NN tracker (`nn`) | 10/10 | 430 | 21/83 | 6/10 | 2.375 ± 0.883 | 0.187 ± 0.011 |
| Tier 2, registration (`registration`) | 10/10 | 520 | 37/77 | 2/10 | 7.437 ± 8.394 | 0.184 ± 0.015 |
| Tier 2, NN on BlueROV2s only | 10/10 | 390 | 14/84 | 6/10 | 1.934 ± 0.848 | 0.185 ± 0.015 |
| Tier 2, NN on Husky + Tarot only | 10/10 | 310 | 7/92 | 9/10 | 0.913 ± 0.639 | 0.187 ± 0.011 |

```
source ~/opt/gz_env.sh
make -C experiments tier2-record                           # 10 recordings, needs Gazebo + GPU
make -C experiments tier2 tables PY=python JOBS=10         # -> paper/data/tier2.csv, tab_tier2*.tex
```

**Findings.**
1. **With ground-truth intra-agent tracks, Tier 2 reproduces the Tier-1
   picture.** Every run merges the team, no alignment is wrong, and G1 holds
   10/10 (largest frame error 0.22-0.87 m per run). Avatar stays within 1.36x
   of the oracle (paired mean; Tier 1: 1.41x), a reference value only (D11).
   Team ATE doubles (0.25 vs 0.12 m) because perception noise now enters: the
   Husky alone goes from 0.02 m (Tier 1) to 0.17 m, so Tier 1's UGV was
   unrealistically good. Merging is slower (median 400 vs 280 s) because Tier 2
   yields about half the detections per run (8 994 vs 17 963, mean).
2. **Extended objects have no viewpoint-invariant centre.** Seed 0, detections
   matched to the nearest part (no radius): hull detections lie a median 3.7 m
   (max 7.8 m) from the part centre, containers 2.4 m (max 3.2 m), against
   0.06-0.14 m for piles, bollards, light poles and buoys. `keep_extended=False`
   (the default) keeps 285 hull and 125 container detections instead of 2 809 and
   2 645. (The pipeline (13 m) and rock (4.2 m) medians are probably clutter matched
   to a far part; I did not check. Read only the hull, container and round classes.)
3. **The NN tracker fails, and the first cause was a bug.** An ambiguous
   detection started a new track, which made the next detection near it
   ambiguous too: on seed 0 the Husky made 2 161 tracks for the 48 parts it
   saw, and one crane leg collected 251. Dropping ambiguous detections
   (`track_on_ambiguity="drop"`, regression test) cuts this to 107 tracks, but
   **the end-to-end result barely moves**: team ATE 2.443 → 2.375 m, wrong
   alignments 22/87 → 21/83, G1 5/10 → 6/10.
4. **The remaining cause is structural.** Per-robot error alone under `nn`
   (mean over 10 seeds; ground-truth tracks in parentheses): Husky 0.66 (0.17),
   Tarot 0.90 (0.16), BlueROV2 0 2.22 (0.11), BlueROV2 1 1.42 (0.14) m; BlueROV2 0
   is above 0.5 m in 9/10 runs. Wrong-track detections are 4.6 % of BlueROV2 0's
   matches (1.0-2.2 % for the others). On seeds 0 and 1, the real confusions of
   BlueROV2 0 (28 / 44 events) are between piles a median 8.0 / 8.3 m apart (range
   4.6-8.0 / 7.7-8.3 m), which is the width of the pier, i.e. mostly opposite piles;
   a further 54 / 41 events are real detections joining a track a spurious cluster
   had started. The tracker gates in the **raw
   dead-reckoning frame**, and BlueROV2 0's raw drift there (start-aligned, xy)
   peaks at 10.3 / 8.0 m on seeds 0 / 1 (7.0 / 8.0 m at the end; Husky 2.0 / 3.4 and
   Tarot 1.6 / 3.1 m at the end), as large as the 8 m between the pier rows (piles are
   about 6 m apart along a row). Sweeping the base gate (0.7, 1.0 m) and the gate
   growth (0, 0.005, 0.01, 0.02 m per metre travelled) on seeds 0 and 1 left BlueROV2 0's
   error at 1.3-2.3 m in 15 of 16 settings. The exception, gate 0.7 m with growth 0.01,
   gave 0.18 m on seed 0 and 1.89 m on seed 1. Not a tuning problem.
   Items 3 and 4 (per seed, plus the gate sweep): `python
   experiments/tier2_tracker_diagnostics.py results/tier2/harbor_fleet_seed0 --sweep`.
5. **Controlled ablation** (table): NN tracking on the BlueROV2s alone
   reproduces the failure (G1 6/10, team ATE 1.93 m); on the Husky and Tarot
   alone it mostly passes (G1 9/10, 0.91 m). The underwater tracking accounts
   for most, not all, of it.
6. **Registration is worse** (negative result). Registering each keyframe to the
   track map before gating gives 37/77 wrong alignments, G1 2/10, and a wrong-track
   rate of 11.5 % on BlueROV2 0. Its CI is dominated by one seed (40.8 m team
   ATE on seed 1). My hypothesis, not tested: in a regular row a shifted
   hypothesis explains the detections as well as the true one, and the tie test
   only refuses exact ties.
7. **Lesson for Tier 1.** `realism_study` injects random identity switches
   (up to 15 %, L14, L25) and GNC keeps the team at its error-free accuracy.
   Here 1-5 % *structured* errors (piles 8 m apart merged into one track)
   break a BlueROV2's local SLAM. The two are not the same experiment, but Tier 1's
   random-switch injection probably understates the danger. **[Not supported: L32
   item 1. Bursts of wrong identities are absorbed too; L29 item 4 showed that the
   damage came from lost loop closures.]**

**Consequences.**
- The Tier-2 headline uses ground-truth intra-agent tracks, labelled as such;
  `nn` and `registration` are reported as negative results, not as the method.
  G1 is **necessary, not sufficient**: the sonar is a ray-cast proxy (R2, R12).
- Association must move into the agent and use its own SLAM estimate (T-F3-02).
  Structured errors go into Tier 1 (T-S1-09). DAVE sonar images: T-S2-05.
- Caveats: 10 seeds, one scenario; the diagnostics in items 2 and 4 use seeds 0
  and 1 only; Tier 2 keeps Tier-1 odometry, so drift is not affected by the
  perception change.

---

## 2026-09-28: trajectories, VoI scheduler, cycle check, speed, server baseline (Claude)

All runs below: `harbor_fleet`, 600 s, M64 unless stated, **simulation (Tier 1)**.
The commit is recorded in every CSV under `results/` (git-ignored; re-run the
command to regenerate).

### L27. Paper data at the final defaults (`ad5241f`): GNC, heading bias, frozen bias

`make -C experiments paper-data` at `ad5241f` (clean), which regenerated all
five tables:
- **Server vs. Avatar** (10 seeds): Avatar team ATE 0.119 ± 0.016 m (M64),
  0.127 ± 0.024 m (X150), 0.107 ± 0.013 m (1 kbit/s); server stride 10 at
  1 kbit/s 0.121 ± 0.014 m, merged later (150 vs. 90 s); still 0/10 merges at
  64 bit/s. Avatar improved from L21 (0.129 / 0.169 / 0.109 m) through GNC,
  heading bias and team-frame fusion.
- **Drift (H1, small-drift regime, D8/D11)**, `fleet_transit_anchored`, 10
  seeds: over the 9 runs with solo drift ≤ 1 m, collaboration changes the
  drifting AUV's error by −6 % on average. Per run: −40 % … +18 %. Four runs
  get slightly worse (+2 … +18 %; the largest is +1 cm on a 0.08 m run).
  Reported as measured; the 30 % of H1 is a reference value (D11). T-X1-04
  (never hurt) stays open.
- **Realism** (3 seeds): GNC keeps team ATE at its error-free level with 15 %
  identity switches plus clutter (0.12 m; Huber 0.75–0.76 m; none 12 m).
- **Bandwidth** and **cycle check:** same conclusions as L17 and L21.

### L26. Remote pairs were overfitting the heading bias: freeze it in the fused graph

Hypothesis from L24: pairs from one end cluster also tune the drifting agent's
heading-bias state (a global shape parameter) in its fused graph. Test: hold
the bias at the agent's local estimate in the fused graph
(`fused_freeze_heading_bias`). `fleet_transit_anchored`, M64, `uuv_1` ATE [m],
fused vs. its own local graph in the same decentralized run (commit after
`4b80ebb`):

| Seed | Local | Fused, bias free | Fused, bias frozen | Team ATE, free → frozen |
|---|---|---|---|---|
| 0 | 0.286 | 0.268 (−6 %) | 0.269 (−6 %) | 0.300 → 0.295 |
| 1 | 0.656 | 0.388 (−41 %) | 0.393 (−40 %) | 1.006 → 0.972 |
| 3 | 0.399 | 0.464 (+16 %) | 0.439 (+10 %) | 0.354 → 0.331 |
| 4 | 0.951 | 1.235 (+30 %) | 0.966 (+2 %) | 0.723 → 0.625 |

- Confirmed for seed 4 (harm removed), partly for seed 3 (residual +10 %,
  cause still open). Gains elsewhere are kept, and team ATE improves in all
  four runs.
- **Default changed:** `fused_freeze_heading_bias = True`. T-X1-04 stays open
  for seed 3 and a 10-seed check.

### L24. Heading bias: model it (default), and a correction to every "alone" number

**Heading model (decision D9).** Platforms declare a heading source. A compass
heading gets no bias; a dead-reckoned one (BlueROV2 near steel, LIO, VIO)
keeps a per-agent bias [rad/m], and the estimator now carries a matching
scalar state with a prior from the platform spec (factor `between_bias`).
The oracle and the server use the same model.

**Methodology correction.** `run_independent` ran a single batch solve of 10
LM iterations. On long drifting trajectories that stops short of the optimum,
and the stopping point can score better or worse than the optimum
(`fleet_transit_anchored`, seed 2: 0.56 m unconverged vs. 1.56 m at the
optimum). From commit `980e47d` the solo baseline re-solves every exchange
period, like each agent's local graph inside the decentralized run. **Every
"alone" number before that commit may be biased, in either direction** (L9,
L14, L19, L20, L22). Team, fused, and oracle numbers are not affected.

Corrected comparison, commit `980e47d`, GNC on, seeds 0–4, M64 (bias off →
on; ATE [m]):

| Preset | Team ATE, Avatar | `uuv_1` alone | `uuv_1` fused | UAV alone | Team merged |
|---|---|---|---|---|---|
| `fleet_default` | 0.118 → **0.110** | 0.117 → 0.107 | 0.100 → 0.095 | 0.167 → 0.151 | 5 → 5 |
| `fleet_exploration` | 0.176 → **0.133** | 0.133 → 0.127 | 0.100 → 0.106 | 0.167 → 0.151 | 5 → 5 |
| `fleet_transit_anchored` | 0.726 → **0.633** | 1.649 → 0.770 | 1.485 → 0.755 | 0.167 → 0.151 | 4 → 5 |
| `fleet_transit` | 0.113 → 0.112 (3 agents) | 3.797 → 2.805 | 3.785 → 2.572 | 0.167 → 0.151 | 0 → 0 |

```
python experiments/trajectory_study.py --presets fleet_default fleet_exploration fleet_transit_anchored fleet_transit --seeds 0 1 2 3 4 [--no-heading-bias]
```

- **Default changed:** `model_heading_bias = True`. It improves team ATE on
  every preset, the UAV solo, and halves the drifting AUV's solo error on
  the anchored transit.
- L19 re-checked with the corrected baseline: on `fleet_transit`, solo `uuv_1`
  ATE is 2.4–5.5 m (bias off) and 1.5–4.5 m (bias on), against oracle
  0.11–0.36 m, which is ≥ 5× in 9 of 10 runs. T-S1-04's acceptance still holds.
- **H1 (collaboration helps the drifting AUV) is mixed with the bias
  modelled.** Anchored transit, fused vs. alone per seed: −7 %, −41 %, −9 %,
  +15 %, +31 % (without the bias state: −21 %, −61 %, 0 %, −8 %, −4 %, never
  worse). Hypothesis, not yet tested: pairs from one end cluster also tune
  the bias state (a global shape parameter) and overfit it. In the normal
  presets, collaboration improves `uuv_1` by 11–17 % and the UAV by 13 %:
  real, but below the plan's "≥ 30 %" (H1 text, D8).

### L25. Graduated non-convexity makes the team immune to front-end errors (GNC default)

`experiments/realism_study.py` with three kernels on landmark observations:
none, Huber (k = 3), and GNC-TLS (inlier bound 4.03 = √χ²₃(0.999), warm
started; Yang, Antonante, Tzoumas and Carlone, RA-L 2020). Seeds 0–2, 600 s,
M64, commit `d5337e4`. "low" = 5 % identity switches + 0.5 clutter/keyframe;
"high" = 15 % + 1.0. Team ATE, Avatar [m] (oracle: 0.078 / 0.092, unaffected):

| Preset | Errors | None | Huber k = 3 | **GNC-TLS** |
|---|---|---|---|---|
| `fleet_default` | none | 0.111 | 0.110 | 0.111 |
| `fleet_default` | low | 1.926 | 0.326 | **0.108** |
| `fleet_default` | high | 18.66 (0/3 merged) | 0.727 (2/3) | **0.120** (3/3) |
| `fleet_exploration` | none | 0.170 | 0.170 | 0.173 |
| `fleet_exploration` | low | 2.270 (1/3) | 0.329 | **0.158** |
| `fleet_exploration` | high | 12.38 (0/3) | 0.815 | **0.168** |

```
python experiments/realism_study.py --seeds 0 1 2    # none | huber:3 | gnc:4.03
```

- With GNC every agent stays at its error-free accuracy, even at 15 %
  identity switches plus clutter (UAV solo: 0.153 m error-free, 0.160 m with
  high errors, versus 1.109 m with Huber). The team ATE stays at its
  error-free level.
- Without errors GNC changes nothing (0.111 vs. 0.111 m); it rejects about
  0.1 % of true observations (the χ² tail).
- Cost: about 3× the wall time of a decentralized run (16 → 45–60 s for
  600 s), with warm starts. A cold start at every exchange was 15× slower.
  Early warm-start bug: new factors were cut before they could pull the
  estimate, which rejected up to 75 % of a UAV's true observations. Fixed by
  one LM pass before the weight update.
- **Default changed:** `point_obs_gnc = 4.03` (decision D10). This changes
  the default code path of every run from this commit on. The oracle keeps
  ground-truth association and is unaffected.

### L23. Why drift is not corrected: the start anchor has no correct link to the team

Diagnostic on `fleet_transit_3uuv`, seed 2, 1 kbit/s (commit `f07863c`).
Pairs from **both** ends reach `uuv_1`'s fused graph: 5–6 pairs with `uuv_2`
at keyframes 0–55 (start) and 13 with `uuv_0`/UGV/UAV at keyframes 436–514
(end). All whitened residuals are small (median 0.3–0.4, none > 2). The
trajectory still does not bend:

1. **Each neighbour's frame is a free variable** in the fused graph. The start
   cluster pins T(`uuv_2`), the end cluster pins T(`uuv_0`), and nothing in the
   graph ties the two frames together, so each absorbs its cluster rigidly.
   Fix, implemented but **off by default**: `frame_links_in_fused` adds between
   factors among neighbour frame variables from received, cycle-consistent
   estimates (σ × 2 against double counting).
2. **No correct estimate links `uuv_2` to the rest of the team.** The only
   candidate, `uuv_0`'s 4-inlier alignment of `uuv_2`, is **wrong** (38.6 m
   off; cycle error 51.7 m / 161°), and the cycle check rightly rejects it.
   `uuv_2` surveys the north piers, which overlap too little with the other
   agents. So the start constraint stays free relative to the end constraints,
   and the frame links have nothing correct to add.
3. Uncertainty is modelled honestly in translation: `uuv_1`'s end-anchored
   frame estimates carry σ ≈ 5 m and its pose marginal at k = 500 is 4.1 m
   (actual drift 4.4 m). Yaw is not: the injected yaw bias (0.0015 rad/m,
   ≈ 24° over 275 m, 1σ) is not in the estimator's odometry model (≈ 2°), so
   yaw gates are over-tight for long transits.

*Correction:* an earlier version of this entry (commit `f07863c`) said the
cycle check vetoed a *correct* link. It did not: checked against ground
truth, that link is wrong.

Next (T-X1-02): a scenario in which the start surveyor is itself linked to the
team (e.g. `uuv_2` sharing piles with `uuv_0` or seen from the quay by the
UGV); `frame_links_in_fused` on; estimator odometry σ that includes the yaw-bias
effect. Then re-run the 50-seed cycle-check benchmark (must stay at 0 wrong).

### L22. Both transit ends surveyed by teammates: still no drift correction (negative)

`fleet_transit_3uuv.yaml` (commit `fb84e56`): the 275 m transit of L19, with
`uuv_2` surveying the start (north piers) and `uuv_0` the end (south piers),
both same-medium. Seeds 0–4, M64. `uuv_1` ATE [m]:

| Seed | Alone | Avatar fused | Oracle |
|---|---|---|---|
| 0 | 1.096 | 1.105 | 0.493 |
| 1 | 1.409 | 1.229 | 0.453 |
| 2 | 4.357 | 4.400 | 0.309 |
| 3 | 4.863 | 4.655 | 0.244 |
| 4 | 6.816 | 6.393 | 0.502 |

```
python experiments/trajectory_study.py --presets fleet_transit_3uuv --seeds 0 1 2 3 4
```

- Fused vs. alone: −1 % to +13 %; **no meaningful drift correction** even
  with same-medium coverage at both ends. The team does not merge within
  600 s at 64 bit/s with five agents (four acoustic nodes share the modem).
- **Lesson (method):** an ad-hoc check first suggested 2.9 → 1.0 m. It
  compared the 2-UUV scenario's solo run with the 3-UUV scenario's fused
  run, and the extra vehicle changes every random stream. Only compare runs
  of the same scenario and seed, as the study scripts do.
- **Hazard:** windowed association (`align_window_kf = 120`) produced a wrong
  pairing (`uuv_1` fused 10.6 m, 1 kbit/s, seed 0). Small windows alias. It
  stays off by default, and any use needs a stricter per-window gate.
- T-X1-02 remains open. The next check is whether pairs from both ends reach
  `uuv_1`'s fused graph at all (L20's diagnostic, applied to this preset),
  then the fusion itself.

### L21. Paper data at `0c3cba2` (clean provenance)

`make -C experiments paper-data` at `0c3cba2`; all three CSVs carry the clean
label (the provenance helper ignores `paper/data`). The only change from L18 is
the connectivity fix (L19): at 512 and 1024 bit/s the team connects at 100 s
instead of 110–120 s. Server-vs-Avatar rows and the cycle-check counts are
unchanged.

### L20. Anchored transit: collaboration corrects small drift, not metre-level drift (T-X1-02, partial)

`fleet_transit_anchored.yaml`: `uuv_1`'s one-way transit (≈ 200 m) starts and
ends in the piers that `uuv_0` surveys (same medium, 4-inlier minimum), so the
team knows both ends. Seeds 0–4, M64, commit `5b5e685`. `uuv_1` ATE [m]:

| Seed | Alone | Avatar fused (windows off / 120 kf) | Oracle | Team ATE, Avatar / oracle |
|---|---|---|---|---|
| 0 | 0.340 | 0.263 / 0.263 | 0.300 | 0.272 / 0.183 |
| 1 | 0.640 | 0.248 / 0.234 | 0.183 | 0.222 → 0.196 (windows) / 0.114 |
| 2 | 1.345 | 1.344 / 1.342 | 0.219 | 0.092 (3 agents) → 1.599 (windows, 4 agents) / 0.132 |
| 3 | 2.555 | 2.330 / 2.330 | 0.219 | 1.362 / 0.133 |
| 4 | 3.345 | 3.166 / 3.166 | 0.314 | 1.665 / 0.193 |

```
python experiments/trajectory_study.py --presets fleet_transit_anchored --seeds 0 1 2 3 4 [--align-window 120]
```

**Findings.**
- **First positive H1 evidence:** with small drift (seeds 0–1), the team
  reduces the drifting AUV's error by 23–63 % and reaches about the oracle's
  level. Windows help a little (seed 1: 0.248 → 0.234 m).
- **Metre-level drift is not corrected** (seeds 2–4: ≤ 9 %). The AUV merges
  into the team frame (it connects at 400–460 s), but its map is bent by 1–3 m
  and is placed rigidly, so team ATE degrades to 1.4–1.7 m. In seed 2,
  windows merge the AUV (4 agents) but make team ATE worse (1.6 m): merging a
  bent map without bending it hurts the team metric.
- So H1 holds only in the small-drift regime so far. Next for T-X1-02:
  (1) check that both end clusters get pairs when drift is large (per-window
  pair counts); (2) if they do, the fused solve is the bottleneck (Huber on
  linked points treats metre-level residuals as outliers; try a
  graduated/annealed kernel or a per-cluster frame initialisation); (3) keep
  a drifting agent out of the team frame until its map is consistent.
- **Diagnostic (same commit, windows 120 kf):** `uuv_1`'s pairs with `uuv_0` all
  come from keyframes 237–361 (pipeline leg and the end), plus one pair at
  k = 0 in seed 1. Their whitened residuals are small (median 0.7, none in
  the Huber tail). So **coverage, not fusion, is the bottleneck**: heading
  east from (50, 4), the sonar (30 m, 90°) sees few surveyed piles, and the
  start is never anchored. Item (2) above is ruled out for now. Next: a start
  that faces surveyed structure, or a teammate that maps the start, then
  re-test.

### L19. Feature-poor transit: the H1 test case exists, and Avatar does not exploit it yet

`fleet_transit.yaml`: `uuv_1` leaves the piers at (12, 42), crosses the sparse
outer mooring field, returns along the pipeline and ends among the piles at
(15, 0), about 275 m one way with no self-revisits. `uuv_0` surveys the outer
field. Seeds 0–4, M64, commit `9287911`.

| Seed | `uuv_1` alone [m] | Oracle (`uuv_1`) [m] | Ratio | `uuv_1` Avatar fused [m] |
|---|---|---|---|---|
| 0 | 2.876 | 0.221 | 13.0 | 2.876 |
| 1 | 3.541 | 0.347 | 10.2 | 3.544 |
| 2 | 5.495 | 0.172 | 31.9 | 5.494 |
| 3 | 2.379 | 0.167 | 14.3 | 2.245 |
| 4 | 4.691 | 0.215 | 21.8 | 4.690 |

```
python experiments/trajectory_study.py --presets fleet_transit --seeds 0 1 2 3 4
```

**Findings.**
- **T-S1-04's acceptance is met:** single-agent AUV ATE is 10–32× the oracle's
  (no front-end errors needed). With the team's landmarks, the drift is
  recoverable in principle.
- **Avatar recovers none of it.** The team never merges within 600 s at 64 bit/s,
  and even at 1 kbit/s `uuv_1`'s fused ATE equals its solo ATE. Diagnosis:
  (1) whole-map rigid alignment fails once the map is bent by metres of
  drift; (2) sliding-window alignment (`align_window_kf`, commit `7cf2806`,
  off by default) finds the end-of-transit cluster (13 inliers against the
  UGV), but the sparse middle has 2–4 parts per window and the start window's
  8 below-water parts do not reach the 8-inlier cross-medium minimum. With a
  single anchored cluster, the frame variable absorbs it rigidly and nothing
  bends. Lowering the cross-only minimum to 5 connects the team (380–540 s)
  but still does not correct `uuv_1`.
- Fixed on the way: the anchor now counts a neighbour's estimate of the
  anchor's frame when checking team connectivity (before, `uuv_1`'s alignment
  of the anchor was ignored).
- Next (T-X1-02): anchor at least two drift-separated clusters (same-medium
  teammate coverage at both ends, or a cross-medium window test that is safe
  with fewer inliers under the cycle check), and check that the fused graph
  then bends the trajectory. This is the core H1 experiment for the paper.

### L16. Team frames: fuse every consistent estimate, not one chain (T-X2-04)

The Avatar X150 outlier (seed 7, team ATE 0.53 m, L15) is a frame problem, not a
trajectory problem: every agent's fused ATE is small, but the anchor's frame of
`uuv_1` is off by 1.6 m / 2.8°. The anchor's own alignment of `uuv_1` (15
cross-medium coaxial pairs) is 1.5 m / 2.7° off while claiming σ_yaw = 0.32°
(≈ 8.5σ). `uuv_1`'s estimate of the reverse edge is 0.5 m / 0.4° off. The
least-σ chain used only the direct edge.

`team_frames()` (commit `8a526a4`) now solves a small 4-DoF pose graph over the
agents' frames with every cycle-consistent estimate the agent knows (own,
received, and the neighbours' estimates of its own frame; σ floors 5 cm /
0.3°). Ad-hoc check on 5 runs (not a script; the 10-seed paper data below
includes the change): team ATE drops in all 5, by 2–10 % (seed 7: 0.53 →
0.49 m). The remaining error is the over-confident cross-only alignment
itself. The simulator places above/below parts on the same axis (no pile rake),
so the bias is not a model artifact; the likely cause is a partly shifted
pairing along a pile row (T-S1-07).

### L17. Bandwidth sweep: ordering matters only at low rates, and VoI does not help (negative)

`experiments/bandwidth_sweep.py`, `fleet_default`, acoustic rate 16 → 1024 bit/s
(raw modem rate, shared by the three acoustic nodes), schedulers FIFO (first
observed, first sent), quality (v0), VoI. Seeds 0–9; Table `tab:bandwidth`.
A 5-seed run at `6b705b6` (`results/`) showed the same pattern.

| Acoustic [bit/s] | FIFO merged / at [s] | Quality merged / at [s] | VoI merged / at [s] | UUV frame error [m], FIFO / quality / VoI |
|---|---|---|---|---|
| 16 | 0/10 / – | 0/10 / – | 0/10 / – | – |
| 32 | 4/10 / 540 | 8/10 / 540 | 5/10 / 560 | 0.166 / 0.175 / 0.157 |
| 64 | 10/10 / 280 | 10/10 / 280 | 10/10 / 280 | 0.135 / 0.153 / 0.144 |
| 128 | 10/10 / 180 | 10/10 / 180 | 10/10 / 180 | **0.388** / 0.174 / 0.219 |
| 256 | 10/10 / 120 | 10/10 / 130 | 10/10 / 130 | 0.135 / 0.129 / 0.135 |
| 512 | 10/10 / 120 | 10/10 / 110 | 10/10 / 120 | 0.133 / 0.132 / 0.131 |
| 1024 | 10/10 / 120 | 10/10 / 120 | 10/10 / 120 | 0.134 / 0.131 / 0.132 |

```
make -C experiments bandwidth      # → paper/data/bandwidth_sweep.csv, 10 seeds
```

Provenance: code at `d40b8ed`. The CSV says `d40b8ed-dirty` only because the
preceding `make paper-data` step had rewritten tracked files in `paper/data/`;
the same applies to `association_cycle_check.csv`. Experiment scripts now use
`experiments/_provenance.py`, which ignores output directories.

**Findings.**
- At 16 bit/s no order merges the team within 600 s. At 32 bit/s the quality
  order merges it in 8/10 runs, VoI in 5/10 and FIFO in 4/10. From 64 bit/s up,
  all orders connect at about the same time; the token bucket, not the order,
  sets the schedule (L6).
- At 128 bit/s FIFO's UUV frames are 2× worse on average (0.39 vs. 0.17 m),
  driven by bad alignments on individual seeds (e.g. 2.1 m at seed 4 in the
  5-seed run). The quality order prefers well-observed parts, which are the
  ones that match.
- **VoI never beats the quality order.** At the lowest rates the binding
  constraint is getting enough *matchable* records across to pass the
  association tests (≥ 8 inliers for cross-only matches), not their geometric
  information. A VoI that models the probability of passing those tests
  (records → accepted alignment) is the next idea (T-C2-01 stays open). The
  default is back to `quality` (commit `d40b8ed`).
- H2 as written (≥ 90 % accuracy at ≤ 20 % of the bytes vs. FIFO) is not
  shown. What the sweep does show: this fleet merges down to a 32 bit/s modem
  (8/10 runs with the quality order), and the order matters only at and below
  128 bit/s.

### L18. Paper data at `d40b8ed` (frame fusion + quality order)

`make -C experiments paper-data` regenerated all three tables. Changes from L15
(`c9d61af`) come from team-frame fusion (L16) and the default order (L17):
Avatar team ATE, mean ± 95 % CI over 10 seeds, M64 0.139 ± 0.019 → **0.129 ±
0.019** m, X150 0.192 ± 0.088 → **0.169 ± 0.076** m, 1 kbit/s 0.131 ± 0.020 →
**0.109 ± 0.011** m (server, stride 10: 0.125 ± 0.015 m, merged at 150 s vs.
Avatar's 90 s). The server rows are unchanged, because its joint solve does not
use the team-frame code. Cycle check: 830/1000 accepted (834 at `c9d61af`),
14 → 0 wrong, 0 correct vetoed.

### L14. Front-end errors: a robust kernel saves single agents, not the team (T-S1-04, partly negative)

`FrontEndErrors` (commit `aecaef2`) adds Poisson clutter per keyframe and
sensor, and identity switches to another part of the same medium within 10 m
(the measured position stays that of the true part). It uses its own random
stream, so everything else is identical. The oracle keeps ground-truth
association and is unaffected. Seeds 0–2, 600 s, M64. "low" = 5 % switches +
0.5 clutter/kf; "high" = 15 % + 1.0. k = Huber threshold on landmark
observations (`AvatarParams.point_obs_robust_k`, default off).

| Preset | Errors | k | Alone ugv / uav / uuv_0 / uuv_1 [m] | Oracle (same agents) [m] | Team, Avatar [m] | Team merged |
|---|---|---|---|---|---|---|
| `fleet_default` | none | – | 0.022 / 0.153 / 0.041 / 0.100 | 0.030 / 0.127 / 0.044 / 0.071 | 0.118 | 3/3 |
| `fleet_default` | low | – | 0.286 / 2.081 / 0.635 / 0.829 | same | 1.260 | 2/3 |
| `fleet_default` | low | 3 | 0.024 / 0.557 / 0.053 / 0.112 | same | 0.481 | 3/3 |
| `fleet_default` | high | – | 0.672 / 3.103 / 1.533 / 2.253 | same | 12.3 | 0/3 |
| `fleet_default` | high | 3 | 0.034 / 1.109 / 0.095 / 0.213 | same | 0.956 | 2/3 |
| `fleet_exploration` | none | – | 0.022 / 0.153 / 0.064 / 0.140 | 0.031 / 0.130 / 0.073 / 0.101 | 0.161 | 3/3 |
| `fleet_exploration` | low | 3 | 0.024 / 0.557 / 0.079 / 0.163 | same | 0.516 | 3/3 |
| `fleet_exploration` | high | 3 | 0.034 / 1.109 / 0.104 / 0.248 | same | 0.950 | 3/3 |

```
python experiments/realism_study.py --seeds 0 1 2     # → results/realism_study.csv
```

**Findings.**
- Without a robust kernel, 5 % identity switches wreck every agent (UAV
  2.1 m, AUVs 0.6–0.8 m). The team merges in 3 of 6 low-error runs and in 0 of 6
  high-error runs. Any realistic front-end needs a robust kernel. Huber at k = 3 barely changes the error-free case (solo ATEs
  identical to 3 decimals; team 0.118 vs. 0.119 m).
- With the kernel, the Husky and the BlueROV2s stay near their error-free
  accuracy (AUV alone ≤ 2.5× the oracle). **T-S1-04's acceptance (AUV alone
  ≥ 5× the oracle) is not met.** The harbor is feature-rich and the DVL is good,
  so a robust single agent does not drift much here. Next: feature-poor
  transits (long open-water legs) and compass disturbance near steel.
- The **UAV** suffers most (0.56 m low, 1.11 m high; 4–9× the oracle): its
  D435i sees few parts, so wrong matches dominate. **Avatar's team ATE follows
  the UAV** (0.48–0.96 m), because the fused graphs keep each agent's own
  corrupted local factors and the team metric aligns all agents at once. This
  is where collaboration *should* help and does not yet: the oracle's
  ground-truth association hides the problem. The fair comparator under
  front-end errors is the server with ideal links (estimated association
  everywhere), still to run (T-S1-04 notes).
- Decision for the PI: turn the robust kernel on by default? It is harmless
  without errors and essential with them, but it changes the default code
  path of every earlier run.

### L15. First paper data: 10 seeds for the server comparison, 50 for the cycle check

`make -C experiments paper-data` at commit `c9d61af` (clean) wrote
`paper/data/server_vs_avatar.csv`, `paper/data/association_cycle_check.csv` and the
generated tables `tab_*.tex` that the manuscript inputs (Tables `tab:server`,
`tab:cycle`). Summary (Tier-1 simulation, `fleet_default`, 600 s):

| Acoustic | Method | Team merged | Connected at [s] median | Team ATE [m], mean ± 95 % CI (merged runs) |
|---|---|---|---|---|
| M64 | Avatar | 10/10 | 280 | 0.139 ± 0.019 |
| M64 | server, stride 10 / 30 | 0/10 / 0/10 | – | – |
| X150 | Avatar | 10/10 | 200 | 0.192 ± 0.088 |
| X150 | server, stride 10 | 5/10 | 580 | 2.17 ± 5.54 |
| 1 kbps | Avatar | 10/10 | 90 | 0.131 ± 0.020 |
| 1 kbps | server, stride 10 / 30 | 10/10 / 8/10 | 150 / 480 | 0.125 ± 0.015 / 0.257 ± 0.091 |
| – | oracle | 10/10 | – | 0.085 ± 0.006 |

- The 5-seed picture of L13 holds at 10 seeds. At 64 bit/s only Avatar merges
  the team. At 1 kbit/s the server (stride 10) is as accurate as Avatar
  (0.125 vs. 0.131 m) but merges later (150 vs. 90 s).
- Outliers behind the wide intervals: Avatar X150 seed 7 (0.53 m; not yet
  investigated) and the server's wrong 4-inlier AUV↔AUV alignment at X150 seed 1
  (10.2 m, L13).
- Cycle check, harbor, 50 seeds: 834/1000 alignments accepted, pair precision
  0.994, 14 wrong → 0, 0 correct vetoed (reproduces L11 at the new commit).
- Still missing for the paper: a paired test against the strongest baseline, the
  T-E2-06 fairness sweep, and the investigation of the X150 outlier.

### L9. Trajectory shape and height: turns cost the UAV, altitude blinds it

Presets in `experiments/scenarios/`, seeds 0–2, commit `8cf50e8`. "Alone" is
single-agent ATE; "team" is the team ATE over the agents in the anchor's frame
graph (4-DoF alignment).

| Preset | Alone uuv_0 | Alone uuv_1 | Alone uav_0 | Team, Avatar | Team, oracle | Team connected [s] |
|---|---|---|---|---|---|---|
| `fleet_default` | 0.041 | 0.100 | 0.153 | 0.118 | 0.078 | 280–300 |
| `fleet_complex_turns` | 0.042 | 0.059 | 0.189 | 0.110 | 0.090 | 340–360 |
| `fleet_heights` | 0.038 | 0.131 | 0.193 | 0.135 (3 agents) | 0.116 | never |
| `fleet_surfacing` | 0.037 | 0.082 | 0.153 | 0.117 | 0.077 | 280–320 |
| `fleet_exploration` | 0.064 | 0.140 | 0.153 | 0.161 | 0.092 | 280 |

```
python experiments/trajectory_study.py --seeds 0 1 2              # → results/trajectory_study.csv
python experiments/uav_altitude_sweep.py --heights 3 4.5 6 7      # → results/uav_altitude_sweep.csv
```

**Findings.**
- **Complex turns** (UAV zig-zag, turn-dependent yaw noise) raise the UAV's solo
  ATE from 0.153 to 0.189 m and delay team connection by about 60 s. The
  BlueROV2 on a figure-8 improves (0.100 → 0.059 m) because it re-observes the
  same structures.
- **Height.** In `fleet_heights` the UAV (helix, 2–7 m) never joins the team.
  The altitude sweep (default stand-off path, commit `f5f9e34`, seeds 0–2) shows
  a sharp cut-off set by the D435i model (30° down, ≤ 6 m range):

  | UAV height [m] | Parts seen | Common with UGV (≥ 3 obs) | UAV first aligned [s] | Team connected |
  |---|---|---|---|---|
  | 3.0 | 20–22 | 12–14 | 60 (all seeds) | 3/3, 280–300 s |
  | 4.5 | 20–22 | 11–12 | 120–160 | 3/3, 280–400 s |
  | 6.0 | 4–8 | 1–6 | 260 in 2/3 seeds | 2/3, 440–460 s |
  | 7.0 | 0 | 0 | never | 0/3 |

  With the reference sensor, the Tarot must stay below about 5 m to take part in
  the map. Above that it needs another sensor (new task T-S1-06). This is a
  **sensor-model result, not a field result**. The D435i range and noise model
  should be checked against data before it goes into the paper.
- **Surfacing windows** give the first RF ↔ UUV alignment at 60 s instead of
  200–260 s (L12 table). Team connection is not earlier, because the other UUV
  still depends on the acoustic path.
- **Exploration-only coverage** hurts the team most: Avatar 0.161 m vs. oracle
  0.092 m (ratio 1.75; seed 0 alone is 0.242 m).
- An earlier run of this study at `4d14543-dirty` is superseded by this one.

### L10. Speed: 2.6× faster, and BLAS threads were the hidden cost

Profiling (cProfile) showed half the time in marginal covariances, which
solved a full sparse LU for every requested landmark column, plus a deep copy
of the graph at each fused solve. Changes (commit `8cf50e8`, T-S1-05):
Schur-complement marginals (factorise only the pose block, invert the dense
landmark block), a symmetric minimum-degree ordering (`MMD_AT_PLUS_A`: 4× less
fill-in than COLAMD), cached factor arrays, and a structural graph copy.

| 600 s run, seed 0, 1 thread | Before | After |
|---|---|---|
| `harbor` | 125 s | 49 s (target ≤ 60 s met) |
| `harbor_fleet` | 47 s | 16 s |
| test suite | 68 s | 37 s |

Metrics are identical to 6 decimals. **Negative finding:** multi-threaded
OpenBLAS made the same solves 5–10× *slower* on these small matrices, and far
worse when several runs shared the 4-core container (4 workers × 4 threads).
Every entry point now pins BLAS to one thread before importing NumPy.

### L11. Cycle consistency removes every wrong alignment in 50 seeds

Kruskal-style check over the team frame graph (T-X2-02, commit `434da8a`):
edges are taken strongest first. An edge that closes a cycle is kept only if
the cycle error is inside a σ-scaled gate (≥ 1 m, ≥ 0.1 rad), so a rejected
edge is always the weakest in its cycle.

```
python experiments/association_precision.py --scenario harbor --duration 90 --seeds $(seq 0 49)
```

`harbor`, 90 s, seeds 0–49: **14 wrong alignments before the check, 0 after,
0 correct alignments vetoed** (all wrong ones are 8-pair cross-only matches
between `ugv_0` and `auv_1`). Limitation: an edge with no cycle cannot be
checked. L13 shows such a case when data is sparse.

### L12. VoI scheduler: no measurable gain at 64 bps (negative result)

`quality` (v0) vs. `voi` (expected D-optimal alignment gain per byte, summed
over receiver domains; T-C2-01) on the same data, seeds 0–4, commit `8cf50e8`.

| Preset | Scheduler | Team connected [s] median | First RF ↔ UUV align [s] median | Team ATE [m] mean |
|---|---|---|---|---|
| `fleet_default` | quality / voi | 280 / 280 | 220 / 260 | 0.125 / 0.124 |
| `fleet_exploration` | quality / voi | 280 / 280 | 260 / 220 | 0.177 / 0.151 |
| `fleet_surfacing` | quality / voi | 280 / 280 | 60 / 60 | 0.136 / 0.136 |

```
python experiments/scheduler_comparison.py --seeds 0 1 2 3 4 \
    --presets fleet_default fleet_exploration fleet_surfacing
```

At 64 bps each node earns about one record per tick, and every run spends
exactly the same 2 340 acoustic bytes. The order of a handful of records barely
changes when frames first connect. Differences are within seed noise; VoI is
slightly better in exploration (0.151 vs. 0.177 m, 5 seeds, not significant).
H2 needs the bandwidth sweep (T-C5-01) with a byte-matched FIFO baseline. VoI
stays the default because it is never worse here. Do not claim H2 from this
table.

### L13. A centralized server with every byte counted cannot close the team at 64 bps

*A&B*-style server (T-E2-05, commit `cd5b610`): robots stream keyframes
(odometry + detections, 27 B + 13 B per detection, descriptors as in Avatar)
to the quay gateway over the same links and budgets. The server applies the
same association front-end and cycle check, then one joint solve. The acoustic
stride sends every n-th keyframe with composed odometry. The downlink is free
(favours the baseline). `fleet_default`, seeds 0–4.

| Acoustic | Method | Whole team connected | Connected at [s] median | UUV keyframes at server | Team ATE [m] mean |
|---|---|---|---|---|---|
| M64 | Avatar | 5/5 | 280 | – | 0.124 |
| M64 | server, stride 10 / 30 | 0/5 / 0/5 | – | 9 % / 26 % | (UGV + UAV only) |
| X150 | Avatar | 5/5 | 200 | – | 0.137 |
| X150 | server, stride 10 / 30 | 1/5 / 0/5 | 600 | 14 % / 41 % | see below |
| 1 kbps | Avatar | 5/5 | 80 | – | 0.113 |
| 1 kbps | server, stride 10 / 30 | 5/5 / 4/5 | 140 / 480 | 99 % / 99 % | 0.120 / 0.250 |
| any | oracle | – | – | – | 0.081 |

```
python experiments/server_vs_avatar.py --seeds 0 1 2 3 4 --profiles m64 x150 acoustic_generic
```

**Findings.**
- Raw keyframes do not fit an M64/X150 modem: the server receives 9–41 % of the
  AUV keyframes. It never merges the whole team at 64 bps, while Avatar's
  condensed landmarks do in every run. At 1 kbps the server works, but connects
  later (140 vs. 80 s) with similar accuracy. With ideal links it reaches
  0.083 m vs. the oracle's 0.081 m, so the gap is the bytes, not the estimator.
- **Failure case (X150, seed 1, stride 10):** with 5 % of `uuv_0`'s keyframes,
  the server accepted a 4-inlier `uuv_0` ↔ `uuv_1` alignment that is 25.5 m off.
  No cycle existed to veto it, and the team ATE is 10.2 m. Sparse data makes
  aliasing likely. This is a fairness caveat as much as a result: a stricter
  inlier gate for the server is part of T-E2-06.
- Caveats: 5 seeds; the uplink format and strides are ours, not A&B's (they had
  no acoustic comms). T-E2-06 sweeps a landmark-only uplink and adaptive
  strides before any paper claim.

---

## 2026-09-28: reference fleet (Claude, after PI decisions D1–D3)

### L6. The reference fleet over a 64 bps modem: bandwidth sets *when* the team connects

`harbor_fleet`: Husky anchor (VLP-16 + D435i), Tarot 680 (D435i), 2 × BlueROV2
(Micron Gemini + DVL), and a quay-side gateway (ADR-0006). 600 s, seeds 0–4,
code at `709dbd5`. The CSV says `7404d13-dirty` because the sweep started before
the commit. Re-running m64/seed 0 at `7a5721f` reproduced it bit for bit.

| Acoustic profile | Team connected at [s] (5 seeds) | Team ATE, decentralized [m] mean (median) | Oracle [m] mean | Median ratio | Acoustic bytes sent |
|---|---|---|---|---|---|
| M64, 64 bps | 280 in all 5 | 0.137 (0.114) | 0.091 | 1.38 | 2 340 |
| X150 class, 100 bps | 200–260 (median 220) | 0.160 (0.158) | 0.091 | 1.47 | ≈ 3 650 |
| generic, 1 kbps | 80–140 (median 120) | 0.133 (0.120) | 0.091 | 1.37 | ≈ 15 500 |

```
python experiments/fleet_acoustic_sweep.py --seeds 0 1 2 3 4 --duration 600   # → results/
```

**Findings.**
- The whole team (UGV, UAV, 2 UUVs) shares one frame in **every run, even at
  64 bps**. Team connection time drops monotonically with modem rate
  (280 → 220 → 120 s).
- At 64 bps the timing is identical across seeds (280 s, 2 340 B). The token
  bucket, not the noise, sets the schedule: each node earns 26 B per 20 s tick,
  and a 1-record packet costs 31 B. **At this rate the link budget determines
  the connection time**, which is the regime where a VoI scheduler (T-C2-01)
  and the gateway policy (T-C7-01) should matter most.
- **Final accuracy does not improve with bandwidth** in this scenario: medians
  are 0.114 / 0.158 / 0.120 m, with no significant difference over 5 seeds.
  The error comes mostly from `uuv_1` (team ATE 0.18–0.24 m), which surveys the
  hulls and connects through the frame chain. Next: look into the chain path
  and heading drift (R9) before claiming H1's "≤ 1.5× oracle". The median ratio
  is 1.37–1.47, but the X150 mean ratio is 1.76.
- Negative / caveat: 5 seeds are too few for statistics; the paper needs ≥ 10.

### L7. Fleet association is clean, including direct above ↔ below matches

One-shot all-to-all alignment after 300 s of solo mapping (4 SLAM agents,
unlimited comm), 20 seeds: **237 / 240** directed alignments accepted,
**0 wrong alignments**, pair precision **1.000**.

```
python experiments/association_precision.py --scenario harbor_fleet --seeds $(seq 0 19) --duration 300
```

In the fleet scenario every pair of SLAM agents observes some common
structures, so the no-overlap aliasing case of L4 (`ugv_0` ↔ `auv_1`) does not
arise. T-X2-02 (cycle consistency) is still needed for the
Aliasing scenario.

### L8. Behaviour changes that affect earlier numbers

Acoustic digests now omit descriptors, and budgets carry over between ticks
(ADR-0006). `harbor` runs after `709dbd5` therefore send different acoustic
traffic than L2 reports. L2 remains the record for `f674a52`.

---

## 2026-09-28: foundation session (Claude)

### L1. Literature check reshapes the gap

- *Above and Below* (McConnell, Shariati, Szenher, Li; RA-L 2026 / ICRA 2026) already
  does USV↔AUV C-SLAM with inter-robot loop closures from structures visible above
  and below the surface. It is **centralized**, has **no aerial or ground agents**,
  and did not use real acoustic comms (listed as future work). This is the closest
  prior work: novelty claim N3 is at high risk until we read it in full (T-R1-01).
- Kimera-Multi is fully distributed, not centralized as the initial Discord scoping said.
- DAVE has a ROS 2 Jazzy + Gazebo Harmonic branch. Porting uuv_simulator is not needed (ADR-0003).
- LOTUSim (IROS 2026) is a multi-domain (air/surface/underwater) Gazebo + ROS 2 maritime simulator.
- Details: `docs/research/gap_analysis.md`, `docs/research/related_work.md`.

### L2. v0 pipeline runs end to end, but the Tier-1 harbour is too easy for H1

Harbour, 5 agents (USV anchor, 2 AUVs, UAV, UGV), seed 1, 600 s, code at `f674a52`
(sonar 20 m / p_det 0.6, association v0.1):

| Mode | ATE per agent [cm] (usv, auv0, auv1, uav, ugv) | Team ATE [cm] | Bytes sent (RF / acoustic) | Wall [s] |
|---|---|---|---|---|
| independent | 3.0, 7.1, 8.2, 4.7, 2.7 | – (no common frame) | 0 / 0 | 7 |
| centralized oracle | 2.9, 6.9, 7.4, 4.7, 2.6 | 5.5 | unlimited | 19 |
| decentralized (Avatar v0) | fused: 3.0, 7.1, 8.0, 4.7, 2.7 | 7.0 | 30.9 kB / 22.6 kB | 164 |

```
avatar compare --scenario harbor --duration 600 --seeds 1
```

(Before the association rewrite and the sonar default change, the same seed gave
team ATE 6.8 cm vs. 4.6 cm for the oracle. The conclusion is unchanged.)

**Finding.** The decentralized team reaches a common frame close to the oracle.
But single-agent drift is negligible because Tier 1 gives each agent
**oracle intra-agent data association** in a **landmark-rich** harbour. The
local-ATE benefit of collaboration (H1) cannot be shown under these
conditions. The benefit shown today is the **common team frame**, which the
independent baseline cannot provide at all. Next: T-S1-04 (front-end
association errors, feature-poor transits, exploration-only coverage).
This does not refute H1. It shows the current simulator cannot test H1.

### L3. Frame error is not a clean alignment metric

On a 90 s run, `auv_1`'s local-frame error was 0.80 m / 3.0°, while its team
ATE was 0.36 m and every one of its inter-agent pairs was correct. `auv_1` starts
50 m from the pier. The frame error includes the heading drift it accumulates
before the first overlap, which no one can observe. **Decision:** team ATE is
the primary consistency metric. Frame error is diagnostic only.

### L4. Association: RANSAC under-sampling and perceptual aliasing

One-shot all-to-all alignment after 90 s of solo mapping (includes pairs that
could not physically communicate, e.g. UAV↔AUV):

| Version | Seeds | Accepted / possible directed links | Wrong alignments | Pair precision |
|---|---|---|---|---|
| intermediate: 2-point RANSAC + σ-scaled gates + ambiguity test (harness bug present) | 8 | (not comparable, see harness bug) | 5 | 0.953 |
| v0.1: σ-scaled gates + consistency-graph cliques + MSAC + ambiguity test + cross-only support ≥ 8 | 20 | 329 / 400 | **4** | **0.996** |

```
python experiments/association_precision.py --seeds $(seq 0 19) --duration 90
```

- The first v0 (RANSAC with fixed 1 m gates) reached only 76/89 = 0.85 pair
  precision in the integration test, with whole alignments shifted by one or two
  pile spacings.
- **RANSAC failure mode.** With ~10³ gated candidates, random 2-point sampling
  rarely drew two correct pairs, and wrong shifted hypotheses won. The PCM-style
  consistency graph with greedy cliques fixed this.
- **Aliasing.** Regular pile rows produce shifted hypotheses. Cross-medium pairs
  constrain only x/y, so pile-top height cannot break the tie. They now weigh 2/3,
  and cross-only alignments need ≥ 8 pairs.
- **Remaining errors:** all 4 wrong alignments are `ugv_0 ↔ auv_1`, a pair with
  **no true overlap**. The UGV sees the landward end of pier A above water, and
  auv_1 sees the seaward end below. A shifted match along the row explains 8 pairs,
  and the true hypothesis has zero support, so the ambiguity test cannot fire.
  Fix: team cycle consistency (T-X2-02).
- **Harness bug (fixed).** Digests are broadcasts with per-link "already sent"
  state. The first all-to-all harness built them per receiver, so only the first
  receiver got data. This explains an apparent 5/20 acceptance rate.

### L5. Cross-language contract holds

The Python and C++ codecs produce byte-identical output on
`testdata/wire_v0_vectors.json`, and the CRC-16 check value is `0x29B1`.
Rounding is pinned to `copysign(floor(|v|+0.5), v)` in binary64 (spec §3),
because `std::round` and `floor(x + 0.5)` differ at 0.49999999999999994.
