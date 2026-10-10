from setuptools import setup

setup(
    name="avatar_sim",
    version="0.1.0",
    packages=["avatar_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/avatar_sim"]),
        ("share/avatar_sim", ["package.xml"]),
        ("share/avatar_sim/launch", ["launch/replay.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "comm_emulator = avatar_sim.comm_emulator:main",
            "agent_node = avatar_sim.agent_node:main",
            "gateway_node = avatar_sim.gateway_node:main",
            "frontend_replay = avatar_sim.frontend_replay:main",
            "sim_clock = avatar_sim.sim_clock:main",
            "sensor_replay = avatar_sim.sensor_replay:main",
            "frontend_live = avatar_sim.frontend_live:main",
        ]
    },
)
