# viz

HTML visualizers (AGENTS.md §2: HTML + JS, no build step).

## Harbour chart (`scenario_viewer.html`)

Plan view and elevation section of a Tier-1 run, drawn with chart conventions:
magenta for structures that cross the waterline (coaxial landmark parts), and
dotted outlines for submerged-only features. It shows ground truth and team-frame
estimates, RF and acoustic packets in flight, per-agent ATE, and the matrix of
inter-agent alignments.

```bash
avatar export-viz --duration 300 --seed 0 --out viz/data/harbor_seed0.json
python -m http.server -d viz 8000   # then open http://localhost:8000/scenario_viewer.html
```

The **Run** selector switches between the bundled runs: the reference fleet
(`harbor_fleet`, M64 64 bps, 600 s) with default paths, complex turns, height
changes, and surfacing windows (presets in `experiments/scenarios/`), plus the
v0 generic team. Regenerate one with, e.g.,
`avatar export-viz --scenario-file experiments/scenarios/fleet_heights.yaml --subsample 3 --out viz/data/fleet_heights_seed0.json`. To view any other
run, use **Load run JSON**. The page draws with Canvas 2D and
loads no libraries. Planned: a live ROS 2 viewer (T-V2-01).
