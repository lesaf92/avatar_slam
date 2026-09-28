# Lab notebook

Dated, append-only record of findings, results, and negative results (AGENTS.md §5).
Newest entries first. Every result gives the command that reproduces it.

---

## 2026-09-28: trajectories, VoI scheduler, cycle check, speed, server baseline (Claude)

All runs below: `harbor_fleet`, 600 s, M64 unless stated, **simulation (Tier 1)**.
The commit is recorded in every CSV under `results/` (git-ignored; re-run the
command to regenerate).

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
