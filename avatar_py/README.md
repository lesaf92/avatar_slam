# avatar_py: Python reference implementation

The package `avatar` is the **reference implementation and test oracle** of Avatar
SLAM (ADR-0002). It has no ROS dependency; it needs only NumPy and SciPy.

```bash
pip install -e "avatar_py[dev]"
pytest avatar_py/tests -q
```

| Module | Content |
|---|---|
| `avatar.types` | Stable enums: `Domain`, `Medium`, `LandmarkFlags`, `LinkType` |
| `avatar.geometry` | 4-DoF pose algebra, 4-DoF Umeyama alignment, NED/ENU and FRD/FLU conversions |
| `avatar.semantics` | Class vocabulary (wire contract) and simulated open-vocabulary descriptors |
| `avatar.sim` | Tier-1 simulator: world and landmark parts, agents and trajectories, sensors, measurement generation, scenarios |
| `avatar.comm` | Channel models (RF, acoustic), broadcast network, wire codec v0 |
| `avatar.backend.graph` | Sparse 4-DoF factor graph: LM, Huber, marginal covariances |
| `avatar.frontend.association` | Cross-medium association and robust 4-DoF alignment |
| `avatar.agent` | `AvatarAgent`: local and fused graphs, digests, inbox, alignments |
| `avatar.eval.metrics` | ATE, team ATE, frame chaining, frame error |
| `avatar.runner` | `independent` / `decentralized` / `centralized` modes on shared measurements |
| `avatar.cli` | `avatar run`, `avatar compare`, `avatar export-viz` |

Every run is deterministic given `--seed`. The measurements are generated once
and shared by all modes, so comparisons between modes are paired.
