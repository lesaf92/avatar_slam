# Task board

How to use this board: AGENTS.md §4. Claim a `todo` task whose dependencies are
`done` by setting it to `in-progress`, with your handle and branch, in your first
commit. One task per branch. Priority: **P0** blocks the next milestone,
**P1** is needed for Paper A, **P2** is for Paper B or nice-to-have.

Status values: `todo` · `in-progress` · `review` · `done` · `blocked`.

## Milestone M0: foundation (target 2026-10-05)

| ID | Task | Pri | Deps | Status | Owner / branch | Acceptance |
|---|---|---|---|---|---|---|
| T-I1-01 | Repo skeleton, rules (AGENTS.md), plan, ADRs, conventions, wire spec | P0 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Docs in `docs/` |
| T-S1-01 | Tier-1 simulator v0 (harbour, 4 domains, sensors, odometry) | P0 | – | done | Claude · same | `pytest` green |
| T-C1-01 | Channel models, network, wire codec v0 (Py + C++ golden vectors) | P0 | – | done | Claude · same | Byte-identical vectors |
| T-B1-01 | 4-DoF factor graph + LM + robust + marginals (Python) | P0 | – | done | Claude · same | Jacobians vs. finite differences |
| T-B1-02 | Agent runtime: local/fused graphs, condensed sharing (ADR-0004) | P0 | T-B1-01 | done | Claude · same | Integration tests |
| T-X1-01 | Association v0.1: gating, consistency-graph cliques, ambiguity test | P0 | – | done | Claude · same | 99.6 % pair precision, 20 seeds (LOG) |
| T-E1-01 | Metrics: ATE (4-DoF), team ATE, frame error, comm accounting | P0 | – | done | Claude · same | – |
| T-V1-01 | Scenario viewer (`viz/scenario_viewer.html`) | P1 | T-E1-01 | done | Claude · same | Opens sample run |
| T-P1-01 | Manuscript skeleton + bib with status lines | P0 | – | done | Claude · same | `make -C paper` |
| T-I1-02 | Get CI green on GitHub (ROS 2 job never run before; fix if red) | P0 | T-I1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | All jobs green on `main` (CI run 5) |
| T-I1-04 | pre-commit hooks (ruff, clang-format, citation check) | P2 | T-I1-01 | todo | | `pre-commit run -a` passes |

## Milestone M1: backbone v1 in the fast sim (target 2026-10-31)

| ID | Task | Pri | Deps | Status | Owner / branch | Acceptance |
|---|---|---|---|---|---|---|
| T-R1-01 | **Deep-read *Above and Below*** (method, association, data, code). Update ledger N3 and the Paper A positioning | P0 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Notes in `docs/research/above_and_below.md`; N3 reworded (not "first"), N2 re-scoped |
| T-R1-02 | Deep-read DRACo-SLAM2 + SlideSLAM (comm numbers, association) | P1 | – | todo | | Notes in `docs/research/` |
| T-R1-04 | Verify every bib entry by DOI (`verified-web` / `UNVERIFIED` → `verified`) | P1 | – | todo | | `make -C paper check` shows no unverified cited entries |
| T-S1-04 | **Realism for H1**: front-end association errors (missed/false detections, id switches), feature-poor transits, exploration-only coverage; show non-trivial single-agent drift | P0 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | LOG entry: independent AUV ATE ≥ 5× the centralized oracle on at least one scenario: **10–32×** on `fleet_transit` (LOG L19); front-end errors in L14 |
| T-S1-03 | Occlusion model (footprint ray casting) | P1 | – | todo | | Test: pile behind hull not detected |
| T-S1-05 | Profile + speed up decentralized runs (incremental solves, cached marginals) | P1 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | 600 s harbour ≤ 60 s wall: 49 s (LOG L10) |
| T-X2-02 | **Team frame-graph cycle consistency**: reject `T_ij` inconsistent with `T_ik ∘ T_kj` (evidence: all 4 wrong alignments in 20 seeds were ugv_0↔auv_1, pairs with no true overlap) | P0 | T-X1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | 0 wrong alignments over 50 seeds (`experiments/association_precision.py`): 14 → 0, no correct one vetoed (LOG L11) |
| T-X2-01 | Aliasing-grid scenario + association benchmark → `paper/data/` | P1 | T-S4-02 | todo | | CSV with precision/recall per seed |
| T-X2-04 | Team frame fusion: pose graph over agent frames from all cycle-consistent estimates (`team_frames`) | P1 | T-X2-02 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Beats the least-σ chain on seeds with a biased edge (LOG L16) |
| T-S1-07 | Sim realism: pile rake/sway (above/below parts offset), so `cross_medium_model_sigma_m` models something real; investigate over-confident cross-only alignments (X150 seed 7, LOG L16) | P1 | – | todo | | Test + LOG entry |
| T-X1-02 | **Drift-tolerant association (core H1 result)**: anchor ≥ 2 drift-separated clusters of a drifting AUV's map so that the fused graph bends its trajectory; windowed alignment exists (`align_window_kf`) | P0 | T-S1-04 | in-progress | Claude · `claude/relaxed-ramanujan-bzrqyd` | `fleet_transit`: Avatar fused `uuv_1` ATE ≤ 2× oracle (LOG L19). Partial: small drift corrected on `fleet_transit_anchored`, metre-level not (L20), not even with both ends surveyed (`fleet_transit_3uuv`, L22); windowed association can alias. **Cause (L23):** free per-neighbour frames, and the start surveyor (`uuv_2`) has no correct link to the team; the yaw-bias effect is missing from the odometry σ |
| T-X2-03 | GNC/PCM-style robust inter-agent factors in the fused graph | P1 | T-X1-01 | todo | | Injected outliers do not move frames > 0.2 m |
| T-C2-01 | **VoI-per-byte digest scheduler** (replace heuristic in `build_digests`) | P0 | T-B1-02 | in-progress | Claude · `claude/relaxed-ramanujan-bzrqyd` | H2 curve: ≥ 90 % accuracy at ≤ 20 % bytes vs. FIFO. Implemented (`avatar/comm/scheduler.py`); no measurable gain at M64 yet (LOG L12); needs the T-C5-01 sweep |
| T-C3-01 | Surface gateway: store-and-forward relay RF ↔ acoustic with dedup (v0 policy, ADR-0006) | P0 | T-C1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | UAV↔UUV alignment through the relay in `harbor_fleet` (test_fleet.py) |
| T-S4-03 | `harbor_fleet` scenario: the PI's reference fleet (Husky, Tarot 680, BlueROV2, gateway, optional BlueBoat) with device-specific sensor, odometry, and channel profiles | P0 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Team connects over M64 (LOG L6) |
| T-C1-02 | Token-bucket budgets per (node, link); frame alignments inside the budget; no descriptors on acoustic | P0 | T-C1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Acoustic bytes ≤ modem capacity (test) |
| T-C4-01 | Surfacing windows (AUV RF bursts when z > −0.3 m) | P1 | T-C1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | `fleet_surfacing.yaml` + test (RF only while surfaced) |
| T-S4-04 | Trajectory library (10 kinds + CSV replay), per-agent start/height/heading/speed overrides, YAML presets, turn-dependent odometry error (PI request) | P0 | – | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | `test_trajectories.py`; LOG L9 |
| T-C6-01 | Modem-M64 driver + link adapter: fragment ≤ 64 B wire packets into modem frames, reassemble, drop incomplete (verify the M64 frame size) | P0 | – | todo | | Loopback test with the recorded frame format |
| T-C7-01 | Gateway policy v1: VoI-based RF→acoustic selection (shares code with T-C2-01) | P1 | T-C2-01 | todo | | Beats the class-priority v0 on time-to-team-connection |
| T-C5-01 | Bandwidth (100 bps–10 kbps) and loss sweep experiment → `paper/data/` | P1 | T-C2-01 | todo | | CSV + figure script |
| T-S1-06 | UAV sensing at altitude: a longer-range or nadir sensor option (e.g. downward mapping camera) so the UAV can join the team above ~5 m | P1 | – | todo | | `fleet_heights.yaml` connects (LOG L9) |
| T-E2-06 | Server baseline fairness: landmark-only uplink and adaptive acoustic stride, swept | P1 | T-E2-05 | todo | | CSV over strides/policies in `paper/data/` |
| T-S4-01 | Scenario YAML (world, team, links) shared by Tier 1 and Tier 2 | P0 | – | todo | | `harbor.yaml` reproduces current harbour |
| T-S4-02 | Dam-face, offshore-jacket, aliasing-grid scenarios | P1 | T-S4-01 | todo | | Registered in `SCENARIOS` |

## Milestone M2: ROS 2 + Gazebo end-to-end (target 2026-12-20)

| ID | Task | Pri | Deps | Status | Owner / branch | Acceptance |
|---|---|---|---|---|---|---|
| T-I1-03 | Docker image: `ros:jazzy` + Gazebo Harmonic + DAVE (ros2 branch) + PX4 SITL | P0 | – | todo | | `docker compose up` spawns the empty world |
| T-S2-01 | Gazebo Harmonic harbour world (piles, hulls, quay, buoys) from scenario YAML | P0 | T-S4-01, T-I1-03 | todo | | World loads; GT exported |
| T-S2-02 | Spawn the reference fleet in `unified.launch.py`: Husky (`clearpath_simulator`), PX4 SITL hexacopter (Tarot-like), BlueROV2 (DAVE), static gateway, optional BlueBoat | P0 | T-S2-01 | todo | | All move on scripted paths |
| T-S2-03 | Sensor bridges: VLP-16 (`gpu_lidar`), D435i (`rgbd_camera`), Micron Gemini (DAVE multibeam: 90°, 128 beams, 50 m), DVL A50, Bar30, IMU (`ros_gz_bridge`), NED→ENU at the boundary | P0 | T-S2-02 | todo | | Topics per `conventions.md` §7 |
| T-S2-04 | Evaluate LOTUSim as the Tier-2 host (vs. DAVE) | P2 | – | todo | | ADR if adopted |
| T-S3-01 | ROS 2 comm emulator (`EncodedPacket` gateway enforcing `avatar.comm` models) | P0 | T-C1-01 | todo | | Same stats as Tier 1 on a replay |
| T-F1-01 | Odometry adapters → 4-DoF increments with covariance | P0 | T-S2-03 | todo | | – |
| T-F2-01 | Open-vocabulary camera detector + CLIP-family embeddings node | P1 | T-S2-03 | todo | | – |
| T-F2-02 | LiDAR object clustering → landmark parts | P0 | T-S2-03 | todo | | – |
| T-F2-03 | Imaging-sonar object extraction (DRACo2-style) | P0 | T-S2-03 | todo | | – |
| T-F3-01 | Landmark-part tracker (medium tagging, intra-agent coaxial links) | P0 | T-F2-* | todo | | – |
| T-F4-01 | Descriptor compression (int8/PQ); D = 0 packets for non-semantic landmarks | P1 | T-F2-01 | todo | | ≤ 32 B/landmark |
| T-B2-01 | C++ back-end on GTSAM (iSAM2), matched to Python on shared graph vectors | P1 | T-B1-01 | todo | | Match to 1 mm / 0.01° |
| T-B4-01 | Acoustic range factors (latency-aware, cross-frame) | P1 | T-B1-02 | todo | | – |
| T-G1 | **Gate G1:** cross-medium association on Gazebo sonar data | P0 | T-F3-01 | todo | | Frame error < 1 m in Gazebo harbour |

## Milestone M3–M4: Paper A (target submission ≈ 2027-03-01)

| ID | Task | Pri | Deps | Status | Owner / branch | Acceptance |
|---|---|---|---|---|---|---|
| T-E2-01 | Swarm-SLAM baseline adapter (ROS 2) | P1 | M2 | todo | | Runs on air/ground subset |
| T-E2-02 | Kimera-Multi baseline adapter | P1 | M2 | todo | | – |
| T-E2-03 | SlideSLAM baseline adapter | P1 | M2 | todo | | – |
| T-E2-04 | DRACo-SLAM2 baseline adapter | P1 | M2 | todo | | Runs on underwater subset |
| T-E2-05 | *A&B*-style centralized server baseline (estimated association, all bytes counted) | P0 | T-X1-01 | done | Claude · `claude/relaxed-ramanujan-bzrqyd` | Mode `server` in `runner.py` (`avatar/baselines/centralized_server.py`); LOG L13 |
| T-E3-01 | Experiment matrix + `make -C experiments paper-data` | P0 | T-E2-* | todo | | Regenerates every paper number |
| T-E4-01 | Real-data validation (A&B / DRACo2 data, ARACATI, PI's data) | P0 | T-R1-05 | todo | | At least one real sequence |
| T-R1-05 | Obtain real datasets (ask authors; licences) | P0 | – | todo | | Data + licence noted |
| T-H1-01 | Hardware bring-up, ROS 2 Jazzy drivers per platform: `velodyne`, `realsense2_camera`, PX4 uXRCE-DDS, BlueROV2 (ArduSub + MAVROS/BlueOS), Water Linked DVL, Tritech Gemini SDK | P0 | D6 | todo | | Each sensor publishes per `conventions.md` §7 |
| T-H1-02 | Comm bring-up: 5 GHz mesh + `rmw_zenoh` (only `/avatar/*` shared), M64 modems through T-C6-01, tether ACL (no Avatar traffic) | P0 | T-C6-01 | todo | | Bench test: packets over real M64 |
| T-H1-03 | Ground truth: RTK for UGV/UAV/gateway, UGPS G2 for UUVs, survey of pile tops and quay | P0 | D6 | todo | | GT logs with uncertainty |
| T-H1-04 | Verify every UNVERIFIED spec in `docs/hardware.md` (Gemini vertical aperture, M64 frame size, X150 payload rate) | P1 | – | todo | | Table updated with datasheet sources |
| T-P2-01 | Introduction + related work from the ledgers | P1 | T-R1-01 | todo | | – |
| T-P3-01 | Method sections (formal statement of the no-double-counting property) | P1 | – | in-progress | Claude · `claude/relaxed-ramanujan-bzrqyd` | – |
| T-P4-01 | Experiments section from `paper/data` | P0 | T-E3-01 | in-progress | Claude · `claude/relaxed-ramanujan-bzrqyd` | – |
| T-P5-01 | Figures (teaser, coaxial parts, bandwidth curves) via scripts | P1 | – | todo | | – |
| T-P6-01 | Internal Reviewer-2 pass (`paper/README.md` checklist) | P0 | T-P4-01 | todo | | Checklist all ticked |

## Paper B / later

| ID | Task | Pri | Deps | Status | Owner / branch |
|---|---|---|---|---|---|
| T-N1-01 | Per-agent 3DGS submaps anchored to keyframes (camera, then LiDAR depth) | P2 | M2 | todo | |
| T-N2-01 | Imaging-sonar rasterizer (range–azimuth) | P2 | T-N1-01 | todo | |
| T-N3-01 | Decentralized submap exchange + uncertainty-weighted merge | P2 | T-N1-01 | todo | |
| T-N4-01 | Open-vocabulary feature field | P2 | T-N1-01 | todo | |
| T-B1-05 | 6-DoF extension | P2 | – | todo | |
| T-B3-01 | DPGO inside RF clusters (study) | P2 | – | todo | |
| T-B3-02 | Consistent multi-hop information sharing | P2 | – | todo | |
| T-V2-01 | Live ROS 2 web viewer (foxglove_bridge/rosbridge + three.js) | P2 | M2 | todo | |
| T-V3-01 | Figure pipeline (matplotlib, shared palette) | P1 | – | todo | |
| T-R1-03 | Monthly novelty re-search (next: 2026-10-28) | P1 | – | todo (recurring) | |

## Notes / hand-offs

- *2026-09-28 (Claude, second session):* **T-S1-04 hand-off.** Works:
  `FrontEndErrors` (clutter, identity switches; own RNG stream, off by default),
  optional Huber on landmark observations, `experiments/realism_study.py`.
  Doesn't: with the robust kernel on, AUVs drift ≤ 2.5× the oracle, so H1's
  acceptance is not met in this harbor. Next: (1) feature-poor transits
  (long open-water legs), compass disturbance near steel; (2) run the server
  with ideal links as the estimated-association comparator; (3) find out why
  Avatar's team ATE follows the corrupted UAV (L14). **PI decision pending:**
  robust kernel on by default? **T-C2-01:** VoI implemented but no gain at
  64 bps (L12); acceptance needs T-C5-01. **Paper:** `make -C experiments
  paper-data` regenerates `paper/data/` (L15); method text for budgets, VoI,
  cycle check, and team frame is in `paper/sections/`. **H1:** `fleet_transit`
  shows 10–32× single-AUV drift, but Avatar does not yet correct it (L19,
  T-X1-02 is the next core task).

- *2026-09-28 (Claude):* the PI fixed the fleet, venue, and authorship. The
  comm stack was delegated and is decided in ADR-0006. The default fleet has no
  surface vessel, so a sensorless gateway relays. Its v0 policy is simple
  (waterline-crossing classes first) and is the obvious place for T-C7-01.

- *2026-09-28 (Claude, foundation session):* everything marked `done` above is on
  branch `claude/relaxed-ramanujan-bzrqyd`. The ROS 2 packages have **not been
  built locally** (packages.ros.org is blocked in the dev container). CI's `ros2`
  job is their first build, which is why T-I1-02 is P0.
- *2026-09-28 (Claude):* T-I1-02 done. The first CI run built `avatar_msgs` and
  `avatar_core` with colcon on ROS 2 Jazzy without changes. The only failure was
  Python formatting outside `avatar_py/`, fixed by a single root `ruff.toml`.
  Merged to `main` via PR #1; all jobs green.
