# AGENTS.md: Rules of the Avatar SLAM project

This file is the rulebook for **every contributor**: human, Claude Code,
Hermes, or any other agent. Read it fully before touching the repository.
If a rule here conflicts with a tool default, the rule here wins. If two rules
here conflict, stop and ask the PI (repository owner).

> **Mission.** Produce a paper that a top robotics journal (IEEE T-RO, IJRR, RA-L, or
> T-FR) will accept, together with open, reproducible code. The paper's system is
> Avatar SLAM, a heterogeneous, decentralized, neural SLAM system for mixed teams
> of aerial, ground, surface, and underwater robots.
> See [`docs/PLAN.md`](docs/PLAN.md).

---

## 1. Read before you work

| Order | File | Why |
|---|---|---|
| 1 | `AGENTS.md` (this file) | Rules |
| 2 | [`docs/PLAN.md`](docs/PLAN.md) | Research thesis, contributions, work packages, milestones |
| 3 | [`docs/TASKS.md`](docs/TASKS.md) | Task board. Pick your task here. |
| 4 | [`docs/architecture.md`](docs/architecture.md) | System layers, data flow, module map |
| 5 | [`docs/conventions.md`](docs/conventions.md) | Frames, units, naming, IDs |
| 6 | [`docs/spec/wire_format_v0.md`](docs/spec/wire_format_v0.md) | Inter-agent byte protocol (contract) |
| 7 | [`docs/decisions/`](docs/decisions/) | Architecture Decision Records (ADRs) |
| 8 | [`docs/research/`](docs/research/) | Gap analysis and verified related work |
| 9 | [`docs/LOG.md`](docs/LOG.md) | Lab notebook: dated findings and results |

## 2. Language policy (hard rule)

| Artifact | Language / format | Location |
|---|---|---|
| Documentation, plans, specs, notes | **Markdown** (GitHub-flavoured; diagrams in Mermaid) | `docs/`, `README.md`, package READMEs |
| Paper manuscript | **LaTeX** (`IEEEtran` journal class) + BibTeX | `paper/` |
| Real-time / onboard code, ROS 2 nodes | **C++17** (Eigen; `rclcpp` for ROS) | `avatar_core/`, `ros2/` |
| Research prototypes, simulation, evaluation, learning, tooling | **Python ≥ 3.10** (NumPy/SciPy; PyTorch when learning is needed) | `avatar_py/`, `experiments/`, `tools/` |
| High-rate / high-quality visualization | **HTML + JS** (ES modules, no build step, pinned CDN versions such as three.js and uPlot) | `viz/` |
| Glue | CMake, YAML, shell (POSIX `sh`/`bash`), GitHub Actions YAML | anywhere needed |

Do not add any other language (Rust, Julia, MATLAB, etc.) without an ADR.

**Python ↔ C++ rule.** New algorithms are prototyped in Python (`avatar_py`),
which acts as the **reference implementation / test oracle**. When an algorithm
is stable and needed onboard or at real-time rates, it is ported to C++
(`avatar_core`), and the port must match the Python reference on shared test
vectors in `testdata/`.

## 3. Repository layout

```
avatar_slam/
├── AGENTS.md / CLAUDE.md / README.md
├── docs/                 # Markdown only: plan, tasks, architecture, specs, ADRs, research
├── paper/                # LaTeX manuscript (IEEEtran), references.bib, figures/, data/
├── avatar_py/            # Python package `avatar` (sim, comm, backend, frontend, eval) + tests
├── avatar_core/          # C++17 library (ROS-agnostic): frames, codec, ... + gtest
├── ros2/                 # ROS 2 Jazzy packages (thin wrappers around the cores)
│   └── avatar_msgs/      # Interface definitions (contract, see ADR-0005)
├── experiments/          # Reproducible experiment configs + scripts (produce paper numbers)
├── viz/                  # HTML visualizers
├── testdata/             # Cross-language golden vectors (contract)
├── tools/                # Dev scripts
└── .github/workflows/    # CI
```

The core libraries (`avatar_py`, `avatar_core`) **must not depend on ROS**.
ROS 2 packages depend on the cores, never the reverse (ADR-0002).

## 4. How to pick up and finish work (multi-agent protocol)

1. **Pick a task** from [`docs/TASKS.md`](docs/TASKS.md) with status `todo` whose
   dependencies are all `done`. Prefer the lowest milestone first.
2. **Claim it.** In your first commit, set its status to `in-progress` and put your
   agent/handle and branch in the `Owner` column. One task per branch.
   Branch name: `wp/<task-id>-<slug>` (e.g. `wp/T-C2-03-voi-scheduler`),
   unless your harness assigns a branch; if it does, use that branch and record it.
3. **Stay in scope.** Touch only what the task needs. If you discover new work,
   add it to `TASKS.md` as a new `todo` row. Do not silently widen your task.
4. **Contracts are frozen without an ADR.** Changing any of the following
   requires a new ADR in `docs/decisions/` and a version bump:
   - `docs/spec/wire_format_*.md` and `testdata/wire_*` vectors
   - `ros2/avatar_msgs` message definitions
   - `docs/conventions.md` (frames, units, IDs)
5. **Definition of done** (all must hold):
   - Code builds with zero warnings; `pytest` / `ctest` pass locally (commands in §6).
   - New behaviour has tests. Numerical code has at least one test against an
     analytic or finite-difference oracle.
   - Public functions and classes have docstrings or Doxygen comments stating
     units and frames.
   - Docs updated: package README, `docs/architecture.md` if structure changed,
     and a dated entry in `docs/LOG.md` for any result or finding.
   - `TASKS.md` row set to `review` (or `done` if the PI allows self-merge).
6. **Hand-off notes.** If you stop before done, write a `Notes` entry in the task
   row (what works, what doesn't, next step). Never leave a broken build on a
   shared branch.

## 5. Scientific integrity (non-negotiable)

- **No fabricated numbers.** Every number in the paper comes from a script in
  `experiments/` that writes to `paper/data/` (CSV/JSON), which LaTeX reads or
  which is copied verbatim. Record the commit hash and seed next to each result.
- **Simulation is labelled as simulation.** Never describe simulated results as
  field results.
- **No unverified citations.** A reference may enter `paper/references.bib` only
  after its authors, title, venue, and year are checked against the publisher,
  DOI, or official proceedings page. Mark each bib entry with a comment
  `% status: verified <date> <source>` or `% status: UNVERIFIED`. The manuscript
  must not compile with an UNVERIFIED entry cited in the final submission
  (see `paper/README.md`).
- **Novelty claims are tracked.** Every "first" / "novel" claim must be listed in
  [`docs/research/gap_analysis.md`](docs/research/gap_analysis.md) with the closest
  prior work and why it does not already do it. Re-check before submission.
- **Baselines run fairly.** Same inputs, same compute budget, tuned per their
  authors' recommendations, with any adaptation documented.
- **Report negative results** in `docs/LOG.md`. They shape the paper.

## 6. Build and test commands

```bash
# Python reference package
pip install -e "avatar_py[dev]"
pytest avatar_py/tests -q
ruff check avatar_py tools experiments && ruff format --check avatar_py tools experiments

# C++ core (plain CMake, no ROS needed)
cmake -S avatar_core -B build/avatar_core -DCMAKE_BUILD_TYPE=Release
cmake --build build/avatar_core -j
ctest --test-dir build/avatar_core --output-on-failure

# ROS 2 Jazzy workspace (Ubuntu 24.04, needs ROS installed)
colcon build --base-paths avatar_core ros2 --packages-up-to avatar_msgs avatar_core
colcon test && colcon test-result --verbose

# Paper
make -C paper            # latexmk -> paper/build/main.pdf

# Run a simulated experiment
python -m avatar.cli compare --scenario harbor --duration 300 --seed 0
```

CI (`.github/workflows/ci.yml`) runs all of the above. A red CI is the owner's
top priority.

## 7. Code style

**Python.** `ruff` (lint + format, config in the root `ruff.toml`), full
type hints, NumPy-style docstrings, `numpy.random.Generator` passed explicitly
(no global RNG state), no hidden I/O in library code, SI units.

**C++.** C++17, `.clang-format` at repository root (Google-based, 100 columns),
`-Wall -Wextra -Wpedantic -Werror`, namespaces `avatar::<module>`, headers in
`include/avatar/`, no raw `new`/`delete`, Eigen fixed-size types for small math,
GoogleTest for tests.

**Both.** Every quantity carries its unit in the name or docstring (`range_m`,
`yaw_rad`, `bandwidth_bps`). Frames are explicit in names (`p_world`,
`T_local_from_remote`); see `docs/conventions.md`.

**Commits.** Imperative subject ≤ 72 chars, prefixed by area:
`docs:`, `paper:`, `py:`, `core:`, `ros2:`, `viz:`, `ci:`, `exp:`.
Reference the task ID in the body (`Task: T-B1-02`).

## 8. What not to do

- Don't commit large binaries (rosbags, datasets, model weights, PDFs of papers).
  Use `.gitignore`d `data/` and `results/` directories and document download scripts.
- Don't commit generated LaTeX build files or `__pycache__`.
- Don't paste text from papers into docs or the manuscript; paraphrase and cite.
- Don't push to `main` directly; open a PR.
- Don't change a contract (see §4.4) as a side effect of another task.
