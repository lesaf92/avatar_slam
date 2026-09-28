# experiments

Reproducible experiment scripts. Scripts that produce paper numbers write to
`paper/data/` and record the commit hash and seeds (AGENTS.md §5). Scratch output
goes to `results/` (git-ignored).

| Script | Purpose | Task |
|---|---|---|
| `association_precision.py` | One-shot all-to-all alignment: acceptance, wrong alignments, pair precision over seeds | T-X1-01 / T-X2-02 |

Planned: `make -C experiments paper-data` regenerates every table and figure (T-E3-01).
