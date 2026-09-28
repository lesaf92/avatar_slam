# Related work ledger

This ledger feeds `paper/references.bib` and the related-work section.
**Status:**
- `verified-web`: authors, venue, and year were confirmed on 2026-09-28 by search results that point to publisher or proceedings pages.
- `memory`: well known, but still needs a DOI check.
- `lead`: not yet confirmed to exist in the form stated.

Rule (AGENTS.md §5): only `verified` entries can be cited in a submitted
manuscript. Upgrade `verified-web` to `verified` by opening the DOI or
publisher page and recording it in the bib comment.

## A. Collaborative SLAM, air/ground (RF links)

| Key | Reference | Status | Relevance |
|---|---|---|---|
| `tian2022kimeramulti` | Y. Tian, Y. Chang, F. Herrera Arias, C. Nieto-Granda, J. P. How, L. Carlone, "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems," *IEEE T-RO* 38(4):2022–2038, 2022 | verified-web | Distributed baseline (p2p, robust DPGO, metric-semantic mesh) |
| `lajoie2024swarmslam` | P.-Y. Lajoie, G. Beltrame, "Swarm-SLAM: Sparse Decentralized Collaborative SLAM Framework for Multi-Robot Systems," *IEEE RA-L* 9(1):475–482, 2024 | verified-web | Decentralized baseline, ROS 2, loop-closure prioritisation under budget |
| `liu2025slideslam` | X. Liu, J. Lei, A. Prabhu, Y. Tao, I. Spasojevic, P. Chaudhari, N. Atanasov, V. Kumar, "SlideSLAM: Sparse, Lightweight, Decentralized Metric-Semantic SLAM for Multi-Robot Navigation," *IEEE T-RO*, 2025 (IEEE Xplore 11230622; arXiv 2406.17249) | verified-web (title/venue); authors from memory | Decentralized heterogeneous air/ground object-level baseline |
| `chang2023hydramulti` | Y. Chang, N. Hughes, A. Ray, L. Carlone, "Hydra-Multi: Collaborative Online Construction of 3D Scene Graphs with Multi-Robot Teams," *IROS* 2023 | verified-web (venue inferred from Xplore ID, check) | Heterogeneous scene graphs, centralized |
| `peterson2025roman` | M. B. Peterson, Y. X. Jia, Y. Tian, A. Thomas, J. P. How, "ROMAN: Open-Set Object Map Alignment for Robust View-Invariant Global Localization," *RSS* 2025 | verified-web | Open-set object alignment baseline for inter-agent association |
| `lajoie2022survey` | P.-Y. Lajoie, B. Ramtoula, F. Wu, G. Beltrame, "Towards Collaborative Simultaneous Localization and Mapping: a Survey of the Current Research Landscape," *Field Robotics*, 2022 | memory | Survey framing |
| `choudhary2017dgs` | S. Choudhary, L. Carlone, C. Nieto, J. Rogers, H. I. Christensen, F. Dellaert, "Distributed mapping with privacy and communication constraints: Lightweight algorithms and object-based models," *IJRR* 36(12), 2017 | memory | Object-based, low-comm distributed mapping (DGS) |
| `tian2021dpgo` | Y. Tian, K. Khosoussi, D. M. Rosen, J. P. How, "Distributed Certifiably Correct Pose-Graph Optimization," *IEEE T-RO* 37(6), 2021 | memory | Distributed back-end option |
| `cunningham2010ddfsam` | A. Cunningham, M. Paluri, F. Dellaert, "DDF-SAM: Fully Distributed SLAM using Constrained Factor Graphs," *IROS* 2010 | memory | Our v0 back-end follows its condensed-landmark sharing idea |
| `cunningham2013ddfsam2` | A. Cunningham, V. Indelman, F. Dellaert, "DDF-SAM 2.0: Consistent Distributed Smoothing and Mapping," *ICRA* 2013 | memory | Double-counting avoidance |
| `mangelson2018pcm` | J. G. Mangelson, D. Dominic, R. M. Eustice, R. Vasudevan, "Pairwise Consistent Measurement Set Maximization for Robust Multi-Robot Map Merging," *ICRA* 2018 | memory | Outlier rejection for inter-agent loop closures |
| `yang2020gnc` | H. Yang, P. Antonante, V. Tzoumas, L. Carlone, "Graduated Non-Convexity for Robust Spatial Perception," *IEEE RA-L* 5(2), 2020 | memory | Robust back-end |

## B. Multi-agent neural / Gaussian mapping

| Key | Reference | Status | Relevance |
|---|---|---|---|
| `yugay2025magic` | V. Yugay, T. Gevers, M. R. Oswald, "MAGiC-SLAM: Multi-Agent Gaussian Globally Consistent SLAM," *CVPR* 2025 | verified-web (first author, venue); co-authors from memory | Centralized multi-agent 3DGS baseline |
| `deng2025mneslam` | T. Deng et al., "MNE-SLAM: Multi-Agent Neural SLAM for Mobile Robots," *CVPR* 2025 | verified-web (title, venue, first-author surname) | Distributed neural SLAM (triplane+MLP, p2p) |
| `thomas2025grandslam` | A. Thomas, A. Sonawalla, A. Rose, J. P. How, "GRAND-SLAM: Local Optimization for Globally Consistent Large-Scale Multi-Agent Gaussian SLAM," *IEEE RA-L*, 2025 | verified-web | Large-scale multi-agent 3DGS (centralized) |
| `yu2025hammer` | "HAMMER: Heterogeneous, Multi-Robot Semantic Gaussian Splatting," IEEE journal (Xplore 11018364), 2025 | verified-web (title); authors lead: J. Yu, T. Chen, M. Schwager | Heterogeneous, server-based semantic 3DGS with CLIP |
| `yu2022dinno` | J. Yu, J. A. Vincent, M. Schwager, "DiNNO: Distributed Neural Network Optimization for Multi-Robot Collaborative Learning," *IEEE RA-L*, 2022 | verified-web (title); venue from memory | C-ADMM consensus for neural maps |
| `asadi2024dinerf` | M. Asadi, K. Zareinia, S. Saeedi, "Di-NeRF: Distributed NeRF for Collaborative Learning with Relative Pose Refinement," *IEEE RA-L*/*IROS* 2024 | lead | Distributed NeRF with pose refinement |
| `macim` | "MACIM: Multi-Agent Collaborative Implicit Mapping," *IEEE RA-L*, 2024 | lead (authors?) | Distributed implicit ESDF |
| `zhao2025ramen` | H. Zhao, B. Ivanovic, N. Mehr, "RAMEN: Real-time Asynchronous Multi-agent Neural Implicit Mapping," *RSS* 2025 | verified-web | Uncertainty-weighted async C-ADMM; closest neural-consensus method |
| `udon2025` | "UDON: Uncertainty-weighted Distributed Optimization for Multi-Robot Neural Implicit Mapping under Extreme Communication Constraints," arXiv 2509.12702, 2025 | verified-web (title); authors? | Extreme-comm neural consensus |
| `kerbl2023gs` | B. Kerbl, G. Kopanas, T. Leimkühler, G. Drettakis, "3D Gaussian Splatting for Real-Time Radiance Field Rendering," *ACM TOG* 42(4), 2023 | memory | Representation |

## C. Underwater and cross-medium multi-robot SLAM

| Key | Reference | Status | Relevance |
|---|---|---|---|
| `mcconnell2026aboveandbelow` | J. McConnell, A. Shariati, P. Szenher, Y. Li, "Above and Below: Heterogeneous Multi-robot SLAM Across Surface and Underwater Domains," *IEEE RA-L*, 2026 (Xplore 11248870; ICRA 2026; arXiv 2605.09811) | verified-web | **Closest prior work.** Centralized USV+AUV; scan-level cross-surface loop closures (LiDAR waterline slice ↔ in-plane sonar, Go-ICP + PCM); AUV data recorded from the surface vessel; simulated comms at ~1–3 kbit/s. Deep reading: `above_and_below.md` |
| `huang2025draco2` | Y. Huang, J. McConnell, X. Lin, B. Englot, "DRACo-SLAM2: Distributed Robust Acoustic Communication-efficient SLAM for Imaging Sonar Equipped Underwater Robot Teams with Object Graph Matching," *IROS* 2025 | verified-web | Decentralized underwater baseline (acoustic) |
| `mcconnell2022draco` | J. McConnell, Y. Huang, P. Szenher, I. Collado-Gonzalez, B. Englot, "DRACo-SLAM: Distributed Robust Acoustic Communication-efficient SLAM for Imaging Sonar Equipped Underwater Robot Teams," *IROS* 2022 | verified-web (title, arXiv 2210.00867); authors from memory | Predecessor |
| `bahr2009cl` | A. Bahr, J. J. Leonard, M. F. Fallon, "Cooperative Localization for Autonomous Underwater Vehicles," *IJRR* 28(6), 2009 | memory | Acoustic range-based cooperative localization |
| `electroslam2026` | Q. Yang, J. Zheng, C. Wang, M. Xiong, G. Xie, "Electro-SLAM: Distributed underwater multi-robot SLAM via bio-inspired active and passive electro-sensing," *IJRR*, 2026 | verified-web | Distributed underwater, non-acoustic sensing |
| `bindusbl2026` | "BIND-USBL: Bounding IMU Navigation Drift using USBL in Heterogeneous ASV-AUV Teams," arXiv 2604.11861, 2026 | lead | ASV–AUV USBL aiding |

## D. Sonar and opti-acoustic neural reconstruction

| Key | Reference | Status | Relevance |
|---|---|---|---|
| `qadri2024aoneus` | M. Qadri, K. Zhang, A. Hinduja, M. Kaess, A. Pediredla, C. A. Metzler, "AONeuS: A Neural Rendering Framework for Acoustic-Optical Sensor Fusion," *SIGGRAPH* 2024 | memory | Camera+sonar neural surfaces |
| `qu2025zsplat` | Z. Qu et al., "Z-Splat: Z-Axis Gaussian Splatting for Camera-Sonar Fusion," *IEEE TPAMI* 47(9), 2025 | verified-web (venue) | Camera–sonar Gaussian splatting |
| `sethuraman2025sonarsplat` | A. V. Sethuraman et al., "SonarSplat: Novel View Synthesis of Imaging Sonar via Gaussian Splatting," 2025 (arXiv 2504.00159) | verified-web (title); venue? | Sonar rasterizer for the neural layer |
| `nasgs2026` | "NAS-GS: Noise-Aware Sonar Gaussian Splatting," arXiv 2601.06285, 2026 | lead | Sonar GS |

## E. Simulation and datasets

| Key | Reference | Status | Relevance |
|---|---|---|---|
| `zhang2022dave` | M. M. Zhang et al., "DAVE Aquatic Virtual Environment: Toward a General Underwater Robotics Simulator," *IEEE/OES AUV Symposium* 2022 | verified-web (title); venue from memory | Underwater sim; ROS 2 Jazzy + Harmonic branch (GSoC 2024/2025) |
| `potokar2022holoocean` | E. Potokar, S. Ashford, M. Kaess, J. G. Mangelson, "HoloOcean: An Underwater Robotics Simulator," *ICRA* 2022 | memory | Alternative sim (UE5, ROS 2 bridge in 2.0) |
| `holoocean2` | "A Preview of HoloOcean 2.0," arXiv 2510.06160, 2025 | verified-web (title) | UE5.3, ROS 2 bridge |
| `lotusim2026` | "LOTUSim: Multi-Domain Simulator for Marine Robotics," *IROS* 2026 (arXiv 2607.03072; github.com/naval-group/LOTUSim, EPL-2.0) | verified-web | **Multi-domain** Gazebo + ROS 2 + Unity + xdyn; candidate host for our benchmark |
| `cieslak2019stonefish` | P. Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface," *OCEANS* 2019 | memory | Alternative sim |
| `aracati` | ARACATI 2014/2017 sonar datasets (FURG, Brazil): marina with imaging sonar + GPS | lead | Possible real cross-medium data (sonar ↔ aerial imagery). Brazilian PI advantage |

## F. Foundations used in the method

| Key | Reference | Status |
|---|---|---|
| `qin2018vins` | T. Qin, P. Li, S. Shen, "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," *IEEE T-RO* 34(4), 2018 | memory (4-DoF pose graph) |
| `dellaert2017factor` | F. Dellaert, M. Kaess, "Factor Graphs for Robot Perception," *Foundations and Trends in Robotics* 6(1–2), 2017 | memory |
| `umeyama1991` | S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," *IEEE TPAMI* 13(4), 1991 | memory |
| `sturm2012tum` | J. Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems," *IROS* 2012 | memory (ATE) |
| `radford2021clip` | A. Radford et al., "Learning Transferable Visual Models From Natural Language Supervision," *ICML* 2021 | memory |
