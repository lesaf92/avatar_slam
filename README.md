# Avatar SLAM

**Heterogeneous, decentralized, neural SLAM for teams of aerial, ground, surface,
and underwater robots.**

In *Avatar* only the Avatar masters every element. Avatar SLAM lets robots in the
**air**, on **earth**, and in the **water** build one consistent map, without a
central server and over links that range from Mbps radio to kbps acoustic modems.

> Status: **M0 foundation** (2026-09-28). The rules, plan, and contracts are in
> place. The Python reference, C++ core, and ROS 2 interfaces are v0. All results
> so far come from the Tier-1 simulator. See [`docs/LOG.md`](docs/LOG.md).

## Why

Harbours, dams, and offshore assets must be mapped above *and* below the
waterline. Existing collaborative SLAM works within one medium: air/ground teams
over RF, or underwater teams over acoustic links. The closest cross-medium system
is centralized and pairs only surface vessels with AUVs. Avatar SLAM targets the
open intersection:

- **Perception split.** LiDAR and cameras see a pile's above-water part, while
  sonar sees the below-water part. Avatar models them as **coaxial landmark parts**.
- **Communication split.** RF carries ≥ 10³× more bits than acoustic links, and an
  AUV reaches RF only when it surfaces. Avatar shares **condensed landmarks
  (16 B each)** under a **medium-aware** policy.
- **No server.** Each agent keeps its own 4-DoF factor graph. Shared information
  comes only from an agent's own measurements, so it is never double counted.

Details: [`docs/PLAN.md`](docs/PLAN.md) (research plan) ·
[`docs/research/gap_analysis.md`](docs/research/gap_analysis.md) (novelty) ·
[`docs/architecture.md`](docs/architecture.md) (system).

## Repository map

| Path | What | Language |
|---|---|---|
| [`AGENTS.md`](AGENTS.md) | **Rules for every contributor (human or AI)** | Markdown |
| [`docs/`](docs/) | Plan, task board, architecture, conventions, specs, ADRs, research, lab log | Markdown |
| [`paper/`](paper/) | Manuscript (IEEEtran journal) + verified bibliography | LaTeX |
| [`avatar_py/`](avatar_py/) | Python reference: simulator, comm, codec, back-end, association, evaluation | Python |
| [`avatar_core/`](avatar_core/) | Real-time core (ROS-agnostic): frames, wire codec | C++17 |
| [`ros2/avatar_msgs/`](ros2/avatar_msgs/) | ROS 2 Jazzy interfaces | ROS IDL |
| [`experiments/`](experiments/) | Reproducible experiment scripts | Python |
| [`viz/`](viz/) | Harbour chart viewer (plan + elevation, playback) | HTML/JS |
| [`testdata/`](testdata/) | Cross-language golden vectors | JSON |

## Quick start

```bash
# Python reference (Python ≥ 3.10)
pip install -e "avatar_py[dev]"
pytest avatar_py/tests -q
avatar compare --scenario harbor --duration 300 --seeds 0      # independent vs. decentralized vs. oracle
avatar export-viz --duration 300 --out viz/data/harbor_seed0.json
python -m http.server -d viz 8000                              # open http://localhost:8000/scenario_viewer.html

# C++ core
cmake -S avatar_core -B build/avatar_core && cmake --build build/avatar_core -j
ctest --test-dir build/avatar_core --output-on-failure

# ROS 2 Jazzy (Ubuntu 24.04)
colcon build --base-paths avatar_core ros2

# Manuscript
make -C paper
```

## Working on the project

1. Read [`AGENTS.md`](AGENTS.md).
2. Pick a task in [`docs/TASKS.md`](docs/TASKS.md) and claim it.
3. Respect the contracts: frames and units ([`docs/conventions.md`](docs/conventions.md)),
   the wire format ([`docs/spec/wire_format_v0.md`](docs/spec/wire_format_v0.md)), and
   ROS messages. Changing a contract requires an ADR.
4. Record findings, including negative ones, in [`docs/LOG.md`](docs/LOG.md).

## License

MIT, see [`LICENSE`](LICENSE).
