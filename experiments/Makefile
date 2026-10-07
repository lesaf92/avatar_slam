# Regenerate every number in the manuscript (AGENTS.md §5, docs/PLAN.md WP-E).
#   make -C experiments paper-data     # runs the experiments, then the tables
#   make -C experiments tables         # tables only, from existing paper/data CSVs
#   make -C experiments all-data       # paper-data, then the Tier-2 studies (on the recordings
#                                      # in results/), then the tables last
# Run from a clean tree: each CSV records `git describe --dirty`.

PY     ?= python3
DATA   := ../paper/data
SEEDS  ?= 0 1 2 3 4 5 6 7 8 9

.PHONY: paper-data all-data tables loss loss-confirm tier2-sonar-heldout tier2-record-ping360 server cycle bandwidth drift realism tier2-record tier2-record-lidar tier2 tier2-heldout tier2-lidar tier2-record-sonar tier2-sonar sonar-diagnostics

paper-data: server cycle bandwidth loss drift realism tables

# `paper-data` builds the tables from whatever Tier-2 CSVs exist, so the Tier-2 studies come
# first here and the tables are rebuilt in a second make (a goal that is already made is skipped).
all-data: paper-data loss-confirm tier2 tier2-heldout tier2-lidar tier2-sonar tier2-sonar-heldout
	$(MAKE) tables

server:
	$(PY) server_vs_avatar.py --seeds $(SEEDS) --profiles m64 x150 acoustic_generic \
		--workers $(JOBS) --out $(DATA)/server_vs_avatar.csv

cycle:
	$(PY) association_precision.py --scenario harbor --duration 90 --seeds $$(seq 0 49) \
		--out $(DATA)/association_cycle_check.csv

bandwidth:
	$(PY) bandwidth_sweep.py --seeds $(SEEDS) --rates 16 32 64 128 256 512 1024 10000 \
		--workers $(JOBS) --out $(DATA)/bandwidth_sweep.csv

# Packet loss (T-C5-01): flat loss at every range, at the M64's rate and at 1 kbit/s.
loss:
	$(PY) bandwidth_sweep.py --seeds $(SEEDS) --rates 64 1024 --losses 0 0.2 0.4 0.6 0.8 \
		--workers $(JOBS) --out $(DATA)/loss_sweep.csv

# The pre-registered paired test of the digest orders under heavy loss (LOG L47): 200 fresh
# seeds, 1200 runs.
loss-confirm:
	$(PY) bandwidth_sweep.py --seeds $$(seq 50 249) --rates 64 --losses 0.5 0.6 \
		--workers $(JOBS) --out $(DATA)/loss_confirm.csv

drift:
	$(PY) trajectory_study.py --presets fleet_transit_anchored --seeds $(SEEDS) \
		--out $(DATA)/drift_anchored.csv

realism:
	$(PY) realism_study.py --seeds 0 1 2 --workers $(JOBS) --out $(DATA)/realism.csv

# Tier 2 (Gazebo Harmonic): needs a GPU, Gazebo and the recorder (gazebo/README.md).
# Not part of `paper-data`, which must run without Gazebo; run it explicitly.
# Seeds 0-19: development set (every design decision was made on them, LOG L29-L31).
# Seeds 20-79: held-out set (D13, LOG L39): 20-39 recorded after the tracker's design (20-29
# were analysed after their run), 40-79 recorded for T-F3-05 (22 of them re-recorded after L39).
# LIDAR: the fleet with a LiDAR on the UAV (LOG L31), recorded into T2LIDARRUNS on seeds 0-79;
# the tracker was not tuned for it. SONAR: DAVE's sonar images, a second pass over the runs.
T2RUNS      ?= ../results/tier2
T2LIDARRUNS ?= ../results/tier2_uavlidar
JOBS        ?= 4
T2DEV       ?= $(shell seq 0 19)
HELDOUT     ?= $(shell seq 20 79)
T2LIDAR     ?= $(shell seq 0 79)
# PING360: the reference fleet with a Ping360 on each BlueROV2 (T-S1-11, D14), development seeds.
T2PING360RUNS ?= ../results/tier2_ping360
T2PING360     ?= $(T2DEV)

tier2-record:
	$(MAKE) -C gazebo
	for s in $(T2DEV) $(HELDOUT); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --out $(T2RUNS)/harbor_fleet_seed$$s || exit 1; \
	done

tier2-record-lidar:
	for s in $(T2LIDAR); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --scenario-arg uav_lidar=true \
			--out $(T2LIDARRUNS)/harbor_fleet_seed$$s || exit 1; \
	done

# Range data with the Ping360 proxy; the Gemini's DAVE images are those of the reference fleet's
# run (hard links: same world, poses and Gemini). Needs `tier2-record-sonar` on $(T2RUNS) first.
tier2-record-ping360:
	for s in $(T2PING360); do \
		$(PY) gazebo/record.py --seed $$s --duration 600 --scenario-arg uuv_ping360=true \
			--out $(T2PING360RUNS)/harbor_fleet_seed$$s || exit 1; \
		ln -f $(T2RUNS)/harbor_fleet_seed$$s/sonar.npz $(T2PING360RUNS)/harbor_fleet_seed$$s/sonar.npz || exit 1; \
	done

tier2:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(T2DEV) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2reg T2ekf T2nn-uuv T2nn-land --out $(DATA)/tier2.csv

tier2-heldout:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(HELDOUT) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2ekf --out $(DATA)/tier2_heldout.csv

tier2-lidar:
	$(PY) tier2_study.py --runs $(T2LIDARRUNS) --seeds $(T2LIDAR) --jobs $(JOBS) \
		--tiers T1 T2 T2nn T2ekf --out $(DATA)/tier2_uavlidar.csv

# DAVE's multibeam sonar (T-S2-05, ADR-0008, LOG L34). The recording
# is a second pass over `tier2-record`'s runs in the avatar-dave image (docker/, gazebo/README.md);
# the study needs no Gazebo. Diagnostics: why the team does not merge on the sonar images.
tier2-record-sonar:
	for s in $(T2DEV) $(HELDOUT); do \
		AVATAR_TIER2_IMAGE=avatar-dave ../docker/tier2.sh python experiments/gazebo/record_sonar.py \
			--run-dir results/tier2/harbor_fleet_seed$$s; \
	done

tier2-sonar:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(T2DEV) --jobs $(JOBS) \
		--tiers T2 T2s T2snn T2sekf --out $(DATA)/tier2_sonar.csv

tier2-sonar-heldout:
	$(PY) tier2_study.py --runs $(T2RUNS) --seeds $(HELDOUT) --jobs $(JOBS) \
		--tiers T2 T2s T2snn T2sekf --out $(DATA)/tier2_sonar_heldout.csv

sonar-diagnostics:
	for st in reasons ambiguity noise snap rules; do \
		$(PY) sonar_diagnostics.py $$st --runs $(T2RUNS) --seeds $(T2DEV) --jobs $(JOBS) \
			--out ../results/sonar_diag_$$st.jsonl; \
	done

tables:
	$(PY) make_paper_tables.py
