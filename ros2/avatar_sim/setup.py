from setuptools import setup

setup(
    name="avatar_sim",
    version="0.1.0",
    packages=["avatar_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/avatar_sim"]),
        ("share/avatar_sim", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={"console_scripts": ["comm_emulator = avatar_sim.comm_emulator:main"]},
)
