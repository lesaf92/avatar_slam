"""The team in ROS 2 on a replay of an offline run (T-S3-02, ADR-0010).

One comm emulator, one free-running clock, a replay front-end and an agent node per SLAM agent,
a gateway node per gateway. Ends when the clock does. Tier 1 (``scenario``, ``scenario_args``,
``seed``, ``duration_s``) or a Tier-2 recording (``run_dir``, ``tracking``, ``sonar``); with
``out_dir`` each agent writes its estimate to ``<out_dir>/<agent>.json``. ``live`` names agents
(comma-separated) whose front-end runs live on the recording's sensor frames (``sensor_replay`` ->
``frontend_live``, T-S3-03) instead of replaying the offline front-end's output.

    ros2 launch avatar_sim replay.launch.py run_dir:=results/tier2/harbor_fleet_seed0 \\
        tracking:=ekf rate:=5.0 out_dir:=results/ros2/seed0
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    "scenario": "harbor_fleet",
    "scenario_args": "{}",
    "seed": "0",
    "duration_s": "600.0",
    "run_dir": "",
    "tracking": "oracle",
    "sonar": "",
    "rate": "5.0",
    "out_dir": "",
    "live": "",
}


def _nodes(context):
    from avatar.agent import AvatarParams
    from avatar.runner import make_sim
    from avatar.tier2.dataset import load_meta

    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    if a["run_dir"]:
        meta = load_meta(a["run_dir"])
        name, seed, dur, kw = (
            meta["scenario"],
            int(meta["seed"]),
            float(meta["duration_s"]),
            meta["scenario_args"],
        )
    else:
        name, seed, dur = a["scenario"], int(a["seed"]), float(a["duration_s"])
        kw = yaml.safe_load(a["scenario_args"]) or {}
    scenario, sim = make_sim(name, seed, dur, AvatarParams(), **kw)
    end_s = float(next(iter(sim.agents.values())).times[-1])
    common = {
        "scenario": a["scenario"], "scenario_args": a["scenario_args"], "seed": int(a["seed"]),
        "duration_s": float(a["duration_s"]), "run_dir": a["run_dir"],
        "tracking": a["tracking"], "sonar": a["sonar"],
    }  # fmt: skip
    live = {s for s in a["live"].split(",") if s}
    if live and not a["run_dir"]:
        raise RuntimeError("live front-ends need a Tier-2 recording (run_dir)")
    # every node but the clock and the live front-ends listens to /clock: the emulator, a
    # front-end (or a sensor replay) and an agent per SLAM agent, a gateway node per gateway
    n_listen = 1 + sum(2 if g.role == "slam" else 1 for g in scenario.agents)
    clock_params = {"end_s": end_s, "rate": float(a["rate"]), "subscribers": n_listen}
    clock = Node(package="avatar_sim", executable="sim_clock", name="sim_clock",
                 parameters=[clock_params])  # fmt: skip
    nodes = [
        Node(package="avatar_sim", executable="comm_emulator", name="comm_emulator",
             parameters=[common]),
        clock,
        RegisterEventHandler(
            OnProcessExit(target_action=clock, on_exit=[EmitEvent(event=Shutdown())])
        ),
    ]  # fmt: skip
    for ag in scenario.agents:
        if ag.role == "slam":
            out = f"{a['out_dir']}/{ag.name}.json" if a["out_dir"] else ""
            if ag.name in live:
                nodes += [
                    Node(package="avatar_sim", executable="sensor_replay",
                         name=f"sensors_{ag.name}", parameters=[common, {"agent": ag.name}]),
                    Node(package="avatar_sim", executable="frontend_live",
                         name=f"frontend_{ag.name}", parameters=[common, {"agent": ag.name}]),
                ]  # fmt: skip
            else:
                nodes.append(
                    Node(package="avatar_sim", executable="frontend_replay",
                         name=f"frontend_{ag.name}", parameters=[common, {"agent": ag.name}])
                )  # fmt: skip
            nodes += [
                # the last solve of a large map takes seconds: let it end before SIGTERM
                Node(package="avatar_sim", executable="agent_node", name=f"agent_{ag.name}",
                     parameters=[common, {"agent": ag.name, "out": out, "end_s": end_s}],
                     sigterm_timeout="60"),
            ]  # fmt: skip
        elif ag.role == "gateway":
            gw = Node(package="avatar_sim", executable="gateway_node", name=f"gateway_{ag.name}",
                      parameters=[common, {"agent": ag.name}])  # fmt: skip
            nodes.append(gw)
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
        + [OpaqueFunction(function=_nodes)]
    )
