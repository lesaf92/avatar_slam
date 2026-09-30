# Regenerate every number in the manuscript (AGENTS.md §5, docs/PLAN.md WP-E).
#   make -C experiments paper-data     # runs the experiments, then the tables
#   make -C experiments tables         # tables only, from existing paper/data CSVs
# Run from a clean tree: each CSV records `git describe --dirty`.

PY     ?= python3
DATA   := ../paper/data
SEEDS  ?= 0 1 2 3 4 5 6 7 8 9

.PHONY: paper-data tables server cycle bandwidth drift realism tier2-record tier2-record-lidar tier2 tier2-heldout tier2-lidar

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
# Seeds 0-19: development set (every design decision was made on them, LOG L29-L31).
# Seeds 20-39: held-out set, recorded afterwards (20-29 were analysed after their
# run, 30-39 only ran once). LIDAR: the fleet with a LiDAR on the UAV (LOG L31),
# recorded into T2LIDARRUNS, on seeds 0-19 and 30-39; the tracker was not tuned for it.
T2RUNS      ?= ../results/tier2
T2LIDARRUNS ?= ../results/tier2_uavlidar
JOBS        ?= 4
T2DEV       ?= $(shell seq 0 19)
HELDOUT     ?= $(shell seq 20 39)
T2LIDAR     ?= $(shell seq 0 19) $(shell seq 30 39)

tier2-record:
	$(MAKE) -C gazebo
	for s in $(T2DEV) $(HELDOUT); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --out $(T2RUNS)/harbor_fleet_seed$$s; \
		$(PY) gazebo/check_geometry.py $(T2RUNS)/harbor_fleet_seed$$s --every 30; \
	done

tier2-record-lidar:
	for s in $(T2LIDAR); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --scenario-arg uav_lidar=true \
			--out $(T2LIDARRUNS)/harbor_fleet_seed$$s; \
		$(PY) gazebo/check_geometry.py $(T2LIDARRUNS)/harbor_fleet_seed$$s --every 30; \
	done

tier2:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(T2DEV) --jobs $(JOBS) \
		--out $(DATA)/tier2.csv

tier2-heldout:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(HELDOUT) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2ekf --out $(DATA)/tier2_heldout.csv

tier2-lidar:
	$(PY) tier2_study.py --runs $(T2LIDARRUNS) --seeds $(T2LIDAR) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2ekf --out $(DATA)/tier2_uavlidar.csv

tables:
	$(PY) make_paper_tables.py
