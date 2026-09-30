# experiments

Reproducible experiment scripts. Scripts that produce paper numbers write to
`paper/data/` and record the commit hash and seeds (AGENTS.md §5). Scratch output
goes to `results/` (git-ignored).

```bash
make -C experiments paper-data   # every Tier-1 table and CSV in paper/data/, then the LaTeX tables
make -C experiments tables       # LaTeX tables only, from the CSVs already in paper/data/
make -C experiments tier2-record tier2 tables   # Tier 2 (needs Gazebo and a GPU, gazebo/README.md)
```

| Script | Purpose | Task |
|---|---|---|
| `association_precision.py` | One-shot all-to-all alignment: acceptance, wrong alignments, pair precision over seeds | T-X1-01 / T-X2-02 |
| `server_vs_avatar.py` | Avatar vs. an *A&B*-style centralized server with every byte counted | T-E2-05 |
| `bandwidth_sweep.py` | Acoustic rate × digest order | T-C5-01 / T-C2-01 |
| `scheduler_comparison.py` | Digest scheduler: quality order vs. VoI per byte | T-C2-01 |
| `fleet_acoustic_sweep.py` | Reference fleet across acoustic modem profiles | T-S4-03 |
| `trajectory_study.py` | Trajectory shape and height vs. single-agent drift and team consistency; the drift table (H1, D8) | T-S4-04 / T-X1-02 |
| `uav_altitude_sweep.py` | UAV altitude vs. its ability to join the team | T-S1-06 |
| `realism_study.py` | Front-end errors and robust kernels | T-S1-04 |
| `tier2_ekf_sweep.py` | One robot's ATE and association statistics over EKF-tracker parameter grids | T-F3-02 |
| `tier2_tracker_diagnostics.py` | Dead-reckoning drift, duplicate tracks, wrong-track events and gate sweeps of one recording | T-F3-01 / T-F3-02 |
| `tier2_study.py` | Tier 1 vs. Tier 2 (Gazebo) parity with GT, NN, registration and EKF tracking; gate G1 | T-S2-01..03, T-G1 |
| `gazebo/` | Tier-2 world, recorder (C++, gz-transport), geometry check | T-S2-01..03 |
| `make_paper_tables.py` | LaTeX tables and macros from `paper/data/*.csv` | T-E3-01 |
| `_provenance.py` | Commit label written into every CSV | AGENTS.md §5 |

`paper-data` does not include Tier 2, which needs Gazebo; run `tier2-record` and
`tier2` explicitly. Run from a clean tree: every CSV records `git describe --dirty`.
