# Regenerate every number in the manuscript (AGENTS.md §5, docs/PLAN.md WP-E).
#   make -C experiments paper-data     # runs the experiments, then the tables
#   make -C experiments tables         # tables only, from existing paper/data CSVs
# Run from a clean tree: each CSV records `git describe --dirty`.

PY     ?= python3
DATA   := ../paper/data
SEEDS  ?= 0 1 2 3 4 5 6 7 8 9

.PHONY: paper-data tables server cycle bandwidth

paper-data: server cycle bandwidth tables

server:
	$(PY) server_vs_avatar.py --seeds $(SEEDS) --profiles m64 x150 acoustic_generic \
		--out $(DATA)/server_vs_avatar.csv

cycle:
	$(PY) association_precision.py --scenario harbor --duration 90 --seeds $$(seq 0 49) \
		--out $(DATA)/association_cycle_check.csv

bandwidth:
	$(PY) bandwidth_sweep.py --seeds $(SEEDS) --rates 16 32 64 128 256 512 1024 \
		--out $(DATA)/bandwidth_sweep.csv

tables:
	$(PY) make_paper_tables.py
