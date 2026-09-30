"""Make the speckle seed of DAVE's multibeam sonar reproducible (ADR-0008, docs/LOG.md L34).

Upstream seeds the speckle of a "blazing" image with ``time(NULL)``: a recording cannot be
reproduced, and frames within one wall-clock second share their speckle. The seed becomes
the simulation time in milliseconds plus a hash of the sensor name, so a frame's speckle
depends on when and by which sensor it was taken, not on the frames computed before it.

    python3 patch_dave_seed.py <dave>/gazebo/dave_gz_multibeam_sonar/multibeam_sonar
"""

import sys
from pathlib import Path

root = Path(sys.argv[1])


def sub(name: str, old: str, new: str) -> None:
    path = root / name
    text = path.read_text()
    assert text.count(old) == 1, (name, old[:60], text.count(old))
    path.write_text(text.replace(old, new))


sub(
    "sonar_calculation_cuda.cuh",
    "bool _debugFlag, bool _blazingFlag);",
    "bool _debugFlag, unsigned long long _blazingFlag);",
)
sub(
    "sonar_calculation_cuda.cu",
    "float beamCorrectorSum, bool debugFlag, bool blazingFlag)",
    "float beamCorrectorSum, bool debugFlag, unsigned long long blazingFlag)",
)
sub(
    "sonar_calculation_cuda.cu",
    """  unsigned long long seed;
  if (blazingFlag)
  {
    seed = static_cast<unsigned long long>(time(NULL));
  }
  else
  {
    seed = 1234;
  }
""",
    """  // blazingFlag is the speckle seed of this frame (0: the fixed seed 1234)
  const unsigned long long seed = blazingFlag != 0ULL ? blazingFlag : 1234ULL;
""",
)
sub(
    "MultibeamSonarSensor.cc",
    "    this->blazingFlag             // _blazingFlag",
    "    this->blazingFlag ? this->frameSeed : 0ULL  // _blazingFlag (a seed)",
)
sub(
    "MultibeamSonarSensor.hh",
    "    bool blazingFlag;",
    "    bool blazingFlag;\n\n"
    "    /// \\brief Speckle seed of the frame being computed (simulation time, sensor name)\n"
    "    unsigned long long frameSeed = 0;",
)
sub(
    "MultibeamSonarSensor.cc",
    "  // Generate sensor data\n  this->Render();",
    """  // Speckle seed from the simulation time and the sensor name: recordings are reproducible
  {
    unsigned long long h = 1469598103934665603ULL;  // FNV-1a of the sensor name
    for (unsigned char c : this->Name())
    {
      h ^= c;
      h *= 1099511628211ULL;
    }
    this->dataPtr->frameSeed =
      h + static_cast<unsigned long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(_now).count());
  }

  // Generate sensor data
  this->Render();""",
)
print("patched", root)
