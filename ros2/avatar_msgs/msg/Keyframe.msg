# Everything one agent measures at one keyframe (ADR-0010): front-end -> agent node.
# Semantic mirror of avatar.sim.measurements.KeyframeData.

std_msgs/Header header            # stamp = keyframe time
uint32 index                      # keyframe number, from 0
bool has_odom                     # false at keyframe 0
float64[4] odom                   # increment [dx, dy, dz, dyaw] from the previous keyframe
float64[4] odom_sigma             # its reported 1-sigma
bool has_abs_z                    # an absolute depth or height (pressure, barometer)
float64 abs_z                     # [m]
float64 abs_z_sigma               # [m]
Detection[] detections
