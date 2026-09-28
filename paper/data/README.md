# paper/data

Machine-generated results that the manuscript reads (CSV/JSON). Each file must
be produced by a script in `experiments/` and must record the commit hash and
seeds in its header or a sidecar `.meta.json`. Do not edit these files by hand.

| File | Written by | Used in |
|---|---|---|
| `server_vs_avatar.csv` | `experiments/server_vs_avatar.py` | `tab_server_vs_avatar.tex` → Table `tab:server` |
| `association_cycle_check.csv` | `experiments/association_precision.py --out` | `tab_cycle_check.tex` → Table `tab:cycle` |
| `tab_*.tex` | `experiments/make_paper_tables.py` (never edit by hand) | `paper/sections/*.tex` |

Regenerate everything with `make -C experiments paper-data` from a clean tree.
