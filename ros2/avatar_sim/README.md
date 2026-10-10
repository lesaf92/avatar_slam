# avatar_sim

Simulation-only ROS 2 nodes of Avatar SLAM: thin `rclpy` wrappers around `avatar_py`
(ADR-0009). Onboard code stays C++ (`avatar_core`). Time runs free (ADR-0010).

| Node | In | Out | What it does |
|---|---|---|---|
| `comm_emulator` (T-S3-01) | `/avatar/comm/tx`, `/clock` | `/avatar/<agent>/rx` | The scenario's links (`avatar.runner.make_network`): medium gating, range, loss, latency, airtime with TDMA shares; a delivery leaves once `/clock` passes its arrival |
| `frontend_replay` (T-S3-02) | `/clock` | `/avatar/<agent>/keyframe` | An offline run's keyframes (Tier 1, or a Tier-2 recording's front-end output) as `avatar_msgs/Keyframe` |
| `agent_node` | its keyframes, its `rx`, `/clock` | `/avatar/comm/tx` | `AvatarAgent`: every 20 s of simulated time, solve, align and send within the budget (`avatar.runner.agent_packets`); at the end, its estimate as JSON |
| `gateway_node` | its `rx`, `/clock` | `/avatar/comm/tx` | The quay-side relay (`avatar.runner.gateway_packets`) |
| `sim_clock` | — | `/clock` | Free-running clock (simulated seconds per wall second); starts once every node listens |

```bash
# the whole team on a Tier-2 recording, its estimates in out_dir
ros2 launch avatar_sim replay.launch.py run_dir:=results/tier2/harbor_fleet_seed0 \
    tracking:=ekf rate:=5.0 out_dir:=results/ros2/seed0
# a Tier-1 scenario
ros2 launch avatar_sim replay.launch.py seed:=1 duration_s:=150.0 \
    scenario_args:="{acoustic: acoustic_generic}"
```

`avatar_py` must be importable (`pip install -e avatar_py`, or `PYTHONPATH=avatar_py`). Tests
(`colcon test --packages-select avatar_sim`): the emulator replays a Tier-1 run's transmissions
with identical statistics; a keyframe message round trip is exact; the team merges in one
process. `experiments/ros2_parity.py` compares the launched team with the offline pipeline seed by
seed (LOG L56).
