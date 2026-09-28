# avatar_core: C++17 real-time core

ROS-agnostic library (Eigen). It builds with plain CMake or colcon
(`package.xml` build type `cmake`). It mirrors the Python reference and is
checked against the shared golden vectors in `testdata/`.

```bash
cmake -S avatar_core -B build/avatar_core -DCMAKE_BUILD_TYPE=Release
cmake --build build/avatar_core -j
ctest --test-dir build/avatar_core --output-on-failure
```

| Header | Content |
|---|---|
| `avatar/types.hpp` | `Domain`, `LinkType`, landmark flag bits |
| `avatar/frames.hpp` | `Pose4` (4-DoF compose/inverse/between), `wrapAngle`, NED/ENU, FRD/FLU |
| `avatar/comm/codec.hpp` | Wire format v0: `LandmarkDigest`, `FrameAlignment`, `encode`, `decode`, CRC-16 |

Downstream (CMake):

```cmake
find_package(avatar_core REQUIRED)
target_link_libraries(my_node avatar::avatar_core)
```

Tests need GoogleTest and nlohmann-json (`libgtest-dev nlohmann-json3-dev`). Disable
them with `-DBUILD_TESTING=OFF`.
