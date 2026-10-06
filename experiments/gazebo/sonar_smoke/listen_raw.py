import sys
import time

import numpy as np
import rclpy
from marine_acoustic_msgs.msg import ProjectedSonarImage
from rclpy.node import Node

DT = {
    0: np.uint8,
    1: np.int8,
    2: np.uint16,
    3: np.int16,
    4: np.uint32,
    5: np.int32,
    8: np.float32,
    9: np.float64,
}


class L(Node):
    def __init__(self):
        super().__init__("sonar_listen_raw")
        self.msgs = []
        self.create_subscription(
            ProjectedSonarImage, "/sensor/multibeam_sonar/sonar_image_raw", self.msgs.append, 10
        )


rclpy.init()
n = L()
t0 = time.time()
NEED = int(sys.argv[3]) if len(sys.argv) > 3 else 3
while time.time() - t0 < float(sys.argv[1]) and len(n.msgs) < NEED:
    rclpy.spin_once(n, timeout_sec=0.5)
print("messages:", len(n.msgs))
m = n.msgs[-1]
d = m.image
a = np.frombuffer(bytes(d.data), dtype=DT[d.dtype])
nb = d.beam_count
nr = len(m.ranges)
print(
    "stamp",
    m.header.stamp.sec,
    m.header.stamp.nanosec,
    "frame",
    m.header.frame_id,
    "dtype",
    d.dtype,
    "size",
    a.size,
    "beams",
    nb,
    "ranges",
    nr,
)
print(
    "freq",
    m.ping_info.frequency,
    "c",
    m.ping_info.sound_speed,
    "n rx_beamwidths",
    len(m.ping_info.rx_beamwidths),
    "n dirs",
    len(m.beam_directions),
)
r = np.asarray(m.ranges)
print("range min/max", r.min(), r.max(), "step", np.diff(r)[:3])
dirs = np.array([[v.x, v.y, v.z] for v in m.beam_directions])
print("beam dir 0", dirs[0], "last", dirs[-1])
img = a.reshape(nr, nb) if a.size == nr * nb else a.reshape(-1, nb)
print("image", img.shape, "min/max/mean", img.min(), img.max(), float(img.mean()))
out = sys.argv[2] if len(sys.argv) > 2 else "/work/results/sonar_smoke/raw.npz"
imgs = np.stack(
    [np.frombuffer(bytes(q.image.data), dtype=DT[q.image.dtype]).reshape(nr, nb) for q in n.msgs]
)
np.savez(out, img=img, imgs=imgs, ranges=r, dirs=dirs)
