# Condensed landmark map of one agent, computed from its own measurements only
# (ADR-0004). Semantic mirror of the LANDMARK_DIGEST wire message.

uint8 DOMAIN_AERIAL=0
uint8 DOMAIN_GROUND=1
uint8 DOMAIN_SURFACE=2
uint8 DOMAIN_UNDERWATER=3

std_msgs/Header header           # frame_id = <agent_name>/map
uint8 sender_id
uint8 domain                     # DOMAIN_* constant
uint16 seq
LandmarkRecord[] records
