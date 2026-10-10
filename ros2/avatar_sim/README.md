# avatar_sim

Simulation-only ROS 2 nodes of Avatar SLAM: thin `rclpy` wrappers around `avatar_py`
(ADR-0009). Onboard code stays C++ (`avatar_core`).

| Node | Topics | What it does |
|---|---|---|
| `comm_emulator` (T-S3-01) | in: `/avatar/comm/tx`, `/clock`; out: `/avatar/<agent>/rx` (`avatar_msgs/EncodedPacket`) | The scenario's inter-agent links (`avatar.runner.make_network`): medium gating, range, loss, latency, airtime with TDMA shares. A delivery is published once `/clock` passes its arrival time |

```bash
ros2 run avatar_sim comm_emulator --ros-args -p scenario:=harbor_fleet -p seed:=0 \
    -p scenario_args:="{acoustic: m64}" -p use_sim_time:=true
```

`avatar_py` must be importable (`pip install -e avatar_py`, or `PYTHONPATH=avatar_py`). The test
replays a Tier-1 run's transmissions through the node and requires its link statistics to equal
the run's (`colcon test --packages-select avatar_sim`).
