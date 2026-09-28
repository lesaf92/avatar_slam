# ADR-0002: ROS-agnostic cores, Python reference + C++ port

- **Status:** Accepted
- **Date:** 2026-09-28

## Context
Many agents will develop in parallel, often in containers without ROS or a
GPU. Research iteration is fastest in Python, while onboard real-time
components need C++. Coupling algorithms to ROS makes unit testing and CI slow.

## Decision
- `avatar_py` (Python package `avatar`) is the **reference implementation** of
  every algorithm and the home of the fast simulator and evaluation.
- `avatar_core` (C++17, Eigen; GTSAM later) holds real-time ports. It is a plain
  CMake project with a `package.xml` (`build_type: cmake`), so colcon builds it.
- `ros2/*` packages are **thin wrappers** (I/O, parameters, TF) around the cores.
- Cross-language equivalence is enforced by golden vectors in `testdata/`.

## Consequences
- The cores never `import rclpy` or include `rclcpp`.
- A port is not "done" until it passes the shared vectors.
- Some duplication (Python + C++) is accepted in exchange for testability.

## Alternatives considered
- *pybind11 bindings of C++ everywhere*: slows research iteration and couples builds.
- *ROS-first nodes*: hard to test and slow to iterate; rejected.
