# Regenerate every number in the manuscript (AGENTS.md §5, docs/PLAN.md WP-E).
#   make -C experiments paper-data     # runs the experiments, then the tables
#   make -C experiments tables         # tables only, from existing paper/data CSVs
# Run from a clean tree: each CSV records `git describe --dirty`.

PY     ?= python3
DATA   := ../paper/data
SEEDS  ?= 0 1 2 3 4 5 6 7 8 9

.PHONY: paper-data tables server cycle bandwidth drift realism tier2-record tier2

paper-data: server cycle bandwidth drift realism tables

server:
	$(PY) server_vs_avatar.py --seeds $(SEEDS) --profiles m64 x150 acoustic_generic \
		--out $(DATA)/server_vs_avatar.csv

cycle:
	$(PY) association_precision.py --scenario harbor --duration 90 --seeds $$(seq 0 49) \
		--out $(DATA)/association_cycle_check.csv

bandwidth:
	$(PY) bandwidth_sweep.py --seeds $(SEEDS) --rates 16 32 64 128 256 512 1024 \
		--out $(DATA)/bandwidth_sweep.csv

drift:
	$(PY) trajectory_study.py --presets fleet_transit_anchored --seeds $(SEEDS) \
		--out $(DATA)/drift_anchored.csv

realism:
	$(PY) realism_study.py --seeds 0 1 2 --out $(DATA)/realism.csv

# Tier 2 (Gazebo Harmonic): needs a GPU, Gazebo and the recorder (gazebo/README.md).
# Not part of `paper-data`, which must run without Gazebo; run it explicitly.
# Seeds 0-9 are the development set (all decisions were made on them); seeds 10-19
# were recorded afterwards and only used for validation (LOG L29).
T2RUNS  ?= ../results/tier2
JOBS    ?= 4
HELDOUT ?= 10 11 12 13 14 15 16 17 18 19
tier2-record:
	$(MAKE) -C gazebo
	for s in $(SEEDS) $(HELDOUT); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --out $(T2RUNS)/harbor_fleet_seed$$s; \
		$(PY) gazebo/check_geometry.py $(T2RUNS)/harbor_fleet_seed$$s --every 30; \
	done

tier2:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(SEEDS) --jobs $(JOBS) \
		--out $(DATA)/tier2.csv
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(HELDOUT) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2ekf --out $(DATA)/tier2_heldout.csv

tables:
	$(PY) make_paper_tables.py
