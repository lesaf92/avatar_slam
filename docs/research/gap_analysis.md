# Gap analysis: where Avatar SLAM sits

*Last literature sweep: 2026-09-28 (web search; publisher pages spot-checked).
Re-run monthly (task `T-R1-03`). Cells marked `?` still need to be checked
against the paper itself.*

## 1. The gap in one paragraph

Collaborative SLAM (C-SLAM) is mature **within a medium**. Decentralized
air/ground teams exist (Kimera-Multi, Swarm-SLAM, SlideSLAM), and so do
decentralized underwater teams on acoustic links (DRACo-SLAM, DRACo-SLAM2).
Multi-agent neural and Gaussian mapping works, but it runs on a server or on
homogeneous RGB-D teams (MAGiC-SLAM, GRAND-SLAM, HAMMER, MNE-SLAM, RAMEN, UDON).
The first **cross-medium** C-SLAM, *Above and Below* (RA-L 2026), links one USV
and several AUVs. It uses structures visible both above and below the
waterline, but runs on a **centralized** server, has **no aerial or ground
agents**, and did not use real acoustic links. **No published system builds a
single, consistent map with a decentralized team that spans air, ground,
surface, and underwater.** Such a team has two properties current systems do
not handle together:

1. Link bandwidths differ by 3–4 orders of magnitude (RF in the Mbps range,
   acoustic at 10²–10⁴ bps), and connectivity is intermittent: an AUV gets RF
   only when it surfaces.
2. Agents in different media see **disjoint parts of the same objects**. The
   camera or LiDAR sees a pile above the water, and the sonar sees it below.

Avatar SLAM targets exactly this intersection.

## 2. Corrections to the initial (Discord) scoping

| Earlier claim | Finding | Consequence |
|---|---|---|
| Kimera-Multi is "centralized" | Kimera-Multi is **fully distributed**, peer-to-peer (T-RO 2022, King-Sun Fu best paper) | It is a decentralized baseline, not a centralized one |
| "No work fuses above/below water in C-SLAM" | *Above and Below* (McConnell et al., RA-L 2026 / ICRA 2026) does USV↔AUV loop closures from structures seen above and below water, centralized | Our novelty must be **decentralized + air/ground + medium-aware comm**, not "first cross-medium" |
| Critical path: port `uuv_simulator` to ROS 2 | Project **DAVE** already has a ROS 2 Jazzy + Gazebo Harmonic branch, including a GPU multibeam sonar migrated in GSoC 2025. **LOTUSim** (IROS 2026) is a Gazebo + ROS 2 multi-domain (air/surface/underwater) maritime simulator | Do not port uuv_simulator. Build on DAVE (ADR-0003) and evaluate LOTUSim |
| ROMAN is "HERCULES collaborative SLAM" | ROMAN is MIT ACL's open-set object map alignment for view-invariant global localization (RSS 2025) | It is a baseline for inter-agent association (open-set objects), not a full C-SLAM benchmark |

## 3. Capability matrix

Legend: ✓ yes · ✗ no · ~ partial · ? unverified

| Work (venue) | Air | Ground | Surface | Underwater | Decentral. | Acoustic / heterogeneous-link aware | Open-vocab semantics | Dense / neural map | Cross-medium assoc. | Code |
|---|---|---|---|---|---|---|---|---|---|---|
| Kimera-Multi (T-RO'22) | ✗? | ✓ | ✗ | ✗ | ✓ | ✗ | ✗ (closed-set) | ~ mesh | ✗ | ✓ |
| Swarm-SLAM (RA-L'24) | ? | ✓ | ✗ | ✗ | ✓ | ~ (RF budget, LC prioritisation) | ✗ | ✗ | ✗ | ✓ |
| SlideSLAM (T-RO'25) | ✓ | ✓ | ✗ | ✗ | ✓ | ~ (sparse objects, 2–3 MB/km) | ✗ (closed-set) | ✗ | ✗ | ✓ |
| Hydra-Multi (IROS'23?) | ? | ✓ | ✗ | ✗ | ✗ (centralized) | ✗ | ✗ | ~ scene graph | ✗ | ? |
| ROMAN (RSS'25) | ? | ✓ | ✗ | ✗ | n/a (alignment) | ✗ | ✓ (open-set objects) | ✗ | ✗ | ✓ |
| MAGiC-SLAM (CVPR'25) | ✗ | ~ RGB-D | ✗ | ✗ | ✗ (server) | ✗ | ✗ | ✓ 3DGS | ✗ | ✓ |
| MNE-SLAM (CVPR'25) | ✗ | ~ RGB-D | ✗ | ✗ | ✓ (p2p) | ✗ | ✗ | ✓ neural field | ✗ | ? |
| GRAND-SLAM (RA-L'25) | ✗ | ✓ | ✗ | ✗ | ✗ (centralized) | ✗ | ✗ | ✓ 3DGS | ✗ | ? |
| HAMMER (RA-L'25?) | ? | ✓ | ✗ | ✗ | ✗ (server) | ✗ | ✓ CLIP | ✓ 3DGS | ✗ | ✓? |
| DiNNO / Di-NeRF / MACIM | – | ✓ | ✗ | ✗ | ✓ (C-ADMM) | ✗ (synchronous) | ✗ | ✓ implicit | ✗ | ~ |
| RAMEN (RSS'25) / UDON ('25) | – | ✓ | ✗ | ✗ | ✓ | ✓ (async, lossy; RF) | ✗ | ✓ implicit | ✗ | ✓ / ? |
| DRACo-SLAM2 (IROS'25) | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ (acoustic) | ✗ | ✗ | ✗ | ✓ |
| Above and Below (RA-L'26) | ✗ | ✗ | ✓ | ✓ | ✗ (centralized) | ✗ (future work) | ✗ | ✗ | ✓ | ? |
| AONeuS / Z-Splat / SonarSplat | ✗ | ✗ | ✗ | ✓ (single agent) | – | – | ✗ | ✓ sonar(+cam) | ✗ | ~ |
| **Avatar SLAM (target)** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ (layer 2) | ✓ | ✓ |

## 4. Novelty-claims ledger

A claim may appear in the paper only once its row is `confirmed`.

| ID | Claim (draft wording) | Closest prior work | Why prior does not do it | Risk | Status |
|---|---|---|---|---|---|
| N1 | First **decentralized** C-SLAM whose team spans **aerial, ground, surface and underwater** agents | Above and Below; SlideSLAM; DRACo-SLAM2 | A&B is centralized and has no air/ground agents; SlideSLAM is air/ground only; DRACo2 is underwater only | Medium: A&B's authors list acoustic comm as future work | draft |
| N2 | **Medium-aware map sharing**: one scheduler allocates map content across RF and acoustic links (≥10³× bandwidth ratio, intermittent), with the USV as gateway | DRACo(2), Swarm-SLAM, RAMEN/UDON, Choudhary et al. IJRR'17 | Each handles one link class; none schedules across heterogeneous media | Medium | draft |
| N3 | **Cross-medium landmark association** (above-water LiDAR/camera ↔ below-water sonar) via a coaxial landmark-part model, decentralized | Above and Below | Must read A&B's association method. If it matches ours, drop "first" and claim only the decentralized, gateway-free variant | **High** | needs-reading (`T-R1-01`) |
| N4 | **Decentralized heterogeneous neural map layer**: per-agent Gaussian submaps with modality-specific rasterizers (pinhole, LiDAR, imaging sonar) fused through the backbone | HAMMER; MNE-SLAM; SonarSplat; Z-Splat | HAMMER is server/camera; MNE is RGB-D; sonar splatting is single-agent | Medium–high (effort) | draft (Paper B) |
| N5 | First **open benchmark** for air-ground-surface-underwater C-SLAM with ground truth and communication traces | LOTUSim, HoloOcean 2.0, DAVE; C-SLAM datasets | Simulators are not SLAM benchmarks; datasets are single-medium | Low | draft |

## 5. Groups to monitor (novelty threats and potential collaborators)

- **McConnell, Englot et al.** (Stevens RFAL; *Above and Below*, DRACo-SLAM/2): closest thread.
- **MIT ACL, How** (ROMAN, GRAND-SLAM) and **MIT SPARK, Carlone** (Kimera-Multi, Hydra-Multi).
- **Stanford MSL, Schwager** (DiNNO, HAMMER); **Zhao, Ivanovic, Mehr** (RAMEN, UDON).
- **UPenn Kumar Lab** (SlideSLAM); **MIST Lab, Beltrame** (Swarm-SLAM).
- **Kaess (CMU) / Metzler / Pediredla** (AONeuS, Z-Splat); **Skinner (UMich)** (SonarSplat).
- **DAVE / LOTUSim / HoloOcean** maintainers (simulation stack).

## 6. Positioning statement (draft for the abstract)

> Teams that mix aerial, ground, surface, and underwater robots could map
> harbours, dams, and offshore assets end to end. Yet no SLAM system lets them
> build one consistent map without a central server. The obstacles are
> physical, not just algorithmic: the media split both what agents *see*
> (optics above water, acoustics below) and how they *talk* (Mbps RF versus
> kbps acoustic links that are intermittent and slow). Avatar SLAM is a
> decentralized metric-semantic SLAM system that (i) models waterline-crossing
> structures as coaxial landmark parts, which connect above- and below-water
> maps; (ii) schedules map content across heterogeneous links by information
> value per byte; and (iii) layers per-agent neural submaps on top of a
> lightweight object-level backbone.
