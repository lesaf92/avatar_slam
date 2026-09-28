# Deep reading: *Above and Below* (T-R1-01)

J. McConnell, A. Shariati, P. Szenher, Y. Li, "Above and Below: Heterogeneous
Multi-robot SLAM Across Surface and Underwater Domains", arXiv 2605.09811v1
(10 May 2026); bib key `mcconnell2026aboveandbelow`. Read in full from the
arXiv v1 PDF (9 pages), supplied by the PI on 2026-09-28. All text below is our paraphrase.

## 1. What the paper does

| Aspect | *Above and Below* |
|---|---|
| Team | 1 USV + 2 "AUVs", 3-DoF planar poses (x, y, θ) |
| Architecture | **Centralized.** The USV is the central node and solves one pose graph (GTSAM) with every robot's odometry chain and the inter-robot loop closures |
| USV front-end | 3D LiDAR, cropped near the waterline using height, roll and pitch; KISS-ICP odometry; no loop closures |
| AUV front-end | DVL + IMU EKF dead reckoning, down-sampled to keyframes; no loop closures on the AUV |
| Cross-surface association | Scan level, not landmark level. A LiDAR slice near the waterline (2D, points below a height cap) and in-plane sonar points (elevation assumed zero) give range-histogram descriptors; a KD-tree proposes candidates; **Go-ICP** (no initial guess) then ICP registers a sonar submap (window w_s keyframes) to a LiDAR submap; an overlap score gates each match; **PCM** keeps the largest pairwise-consistent set |
| Communication | One way, AUV → USV: pose estimate + sonar contacts (SOCA-CFAR) compressed as rectangles. Average rate 0.95 / 1.86 / 3.38 kbit/s in the three sites; mean message 5.6 kbit. **Simulated comms**; the text cites COTS modems up to 62.5 kbit/s. Real acoustic comms and inter-robot ranges are future work |
| Data | Real perceptual data from one modified Kingfisher (VLP-32C above, Oculus M750d sonar on a pole, Nortek DVL-1000, VN100 IMU), with RTK-GPS ground truth. **The "AUV" datasets were recorded by the surface vessel** carrying the sonar, then replayed concurrently |
| Environments | Bridge, waterfront, harbor. All three have strong magnetic interference (steel hulls, power lines) |
| Results | MAE/RMSE per AUV against RTK-GPS. Large sonar windows help; PCM is sometimes too strict; small windows often fail ("Fail" in 7 of 12 small-window rows). The best cases improve over single-robot SLAM by up to several metres (waterfront, harbor) and by under 1 m at the bridge |
| Code | Announced as open source (link in the paper) |

## 2. Consequences for our novelty claims (ledger, `gap_analysis.md` §4)

- **N3 (cross-medium association): not "first".** *Above and Below* already
  finds loop closures between above-water LiDAR and below-water sonar through
  structures that cross the surface. What differs in ours: (i) **landmark-level**
  coaxial parts (pile top ↔ pile bottom, horizontal-only factor) instead of
  scan registration; (ii) run **on every agent** (decentralized), on condensed
  records that fit a 64 bit/s modem; (iii) the same model links **UAV and
  UGV** maps to the AUVs (direct above ↔ below matches, LOG L7). The claim is
  re-worded accordingly (ledger N3).
- **N1 (decentralized, air-ground-surface-underwater):** still open. *Above and
  Below* is centralized and has no aerial or ground agents.
- **N2 (medium-aware communication):** *Above and Below* needs about
  1–3.4 kbit/s on average, with simulated links. Our result that the team
  merges over a real 64 bit/s modem profile (LOG L15–L18, L21), while a
  keyframe-streaming server does not, directly addresses their future work.
  Our VoI order does **not** beat a simple quality order (LOG L17), so N2 must
  claim the link budgets, gateway and condensed records, not the VoI criterion.
- **Evaluation:** their ground truth is RTK-GPS on a surface vessel. Our
  Tier-1 results are simulated. A fair comparison on their data (T-E2-05,
  E4) needs their sonar/LiDAR streams and cannot use our landmark front-end
  as is. Contact the authors about data (decision D3).

## 3. Ideas worth borrowing

- **Heading near steel.** They report substantial magnetic interference in all
  three sites. This supports modelling a gyro-integrated heading with bias for
  the BlueROV2 in harbours (task on heading source, LOG L24).
- **Robust back-end.** They discuss PCM, DCS and switchable constraints and
  note that robust back-ends tend to defer to the odometry chain when
  inter-robot measurements are sparse. That is exactly our sparse-link regime
  (graduated non-convexity study, T-X2-03).
- **Window size** matters for sonar registration, as for our windowed
  association (LOG L19–L22): small windows alias.
