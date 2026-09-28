# Raw wire-format bytes crossing a (simulated or real) inter-agent link.
# payload follows docs/spec/wire_format_v0.md (magic 0xA7, CRC-16/CCITT-FALSE).

uint8 LINK_RF=0
uint8 LINK_ACOUSTIC=1

std_msgs/Header header           # stamp = transmission start time
uint8 sender_id
uint8 link_type                  # LINK_* constant
uint8[] payload
