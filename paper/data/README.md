# paper/data

Machine-generated results that the manuscript reads (CSV/JSON). Each file must
be produced by a script in `experiments/` and must record the commit hash and
seeds in its header or a sidecar `.meta.json`. Do not edit these files by hand.

| File | Written by | Used in |
|---|---|---|
| `server_vs_avatar.csv` | `experiments/server_vs_avatar.py` | `tab_server_vs_avatar.tex` → Table `tab:server` |
| `association_cycle_check.csv` | `experiments/association_precision.py --out` | `tab_cycle_check.tex` → Table `tab:cycle` |
| `bandwidth_sweep.csv` | `experiments/bandwidth_sweep.py` | `tab_bandwidth.tex` → Table `tab:bandwidth` |
| `drift_anchored.csv` | `experiments/trajectory_study.py` | `tab_drift.tex`, `drift_summary.tex` → Table `tab:drift` |
| `realism.csv` | `experiments/realism_study.py` | `tab_realism.tex` → Table `tab:realism` |
| `tier2.csv` | `experiments/tier2_study.py --seeds 0-9` (needs Gazebo recordings; `make -C experiments tier2`) | `tab_tier2.tex`, `tab_tier2_agents.tex`, `tier2_summary.tex` (`\tier...`) → Table `tab:tier2` |
| `tier2_heldout.csv` | `experiments/tier2_study.py --seeds 10-19` (recorded after the design was fixed) | `tab_tier2_heldout.tex`, `tab_tier2_heldout_agents.tex`, `tier2_heldout_summary.tex` (`\held...`) → Table `tab:tier2held` |
| `tab_*.tex` | `experiments/make_paper_tables.py` (never edit by hand) | `paper/sections/*.tex` |

Regenerate everything with `make -C experiments paper-data` from a clean tree.
