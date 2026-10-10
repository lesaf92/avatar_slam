# One detection of a landmark part at a keyframe, as a front-end hands it to the agent
# (ADR-0010). Semantic mirror of avatar.sim.sensors.Detection without the ground truth.

uint8 MEDIUM_ABOVE=0
uint8 MEDIUM_BELOW=1

int64 track_id                    # the front-end's track id (agent-private)
int64 object_key                  # the front-end's key of the physical structure (links the
                                  # parts of one structure across the waterline); -1: none
uint8 medium                      # MEDIUM_*
uint8 modality                    # LandmarkRecord.FLAG_CAMERA, FLAG_LIDAR or FLAG_SONAR
geometry_msgs/Point position      # [m] gravity-aligned body frame
geometry_msgs/Vector3 sigma       # 1-sigma along x, y, z [m]
geometry_msgs/Vector3 extent      # footprint x, footprint y, part height [m]; 0 = not measured
uint8 class_id                    # index into avatar.semantics.CLASS_NAMES (0 = unknown)
float64[] descriptor              # open-vocabulary descriptor; may be empty
