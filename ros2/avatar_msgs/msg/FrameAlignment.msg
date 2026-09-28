# Sender's estimate of T_sender_other: maps points in the other agent's local map
# frame into the sender's local map frame (4-DoF, gravity-aligned, waterline datum).

std_msgs/Header header           # frame_id = <sender_name>/map
uint8 sender_id
uint8 other_id
uint16 n_inliers
float64 x                        # [m]
float64 y                        # [m]
float64 z                        # [m]
float64 yaw                      # [rad], (-pi, pi]
float64 sigma_xy                 # [m]
float64 sigma_z                  # [m]
float64 sigma_yaw                # [rad]
