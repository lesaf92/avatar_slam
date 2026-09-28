# One condensed landmark part, in the sender's local map frame.
# Semantic mirror of a LANDMARK_DIGEST record (docs/spec/wire_format_v0.md §3).

uint8 FLAG_ABOVE=1
uint8 FLAG_BELOW=2
uint8 FLAG_CAMERA=4
uint8 FLAG_LIDAR=8
uint8 FLAG_SONAR=16

uint16 landmark_id               # sender-private id (never a ground-truth id)
geometry_msgs/Point position     # [m] sender local map frame (ENU, z = 0 at the waterline)
float32 sigma_xy                 # isotropic horizontal 1-sigma [m]
float32 sigma_z                  # vertical 1-sigma [m]
geometry_msgs/Vector3 extent     # footprint x, footprint y, observed-part height [m]
uint8 class_id                   # index into avatar.semantics.CLASS_NAMES (0 = unknown)
uint8 flags                      # FLAG_* bits; exactly one of ABOVE/BELOW is set
uint16 n_obs                     # number of observations fused into this landmark
float32[] descriptor             # open-vocabulary descriptor in [-1, 1]; may be empty
