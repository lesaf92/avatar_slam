"""Check the sonar smoke test: targets at the right range and bearing, runs bit-identical.

Ground truth of world.sdf, in the sonar frame (sonar at (0, 0, -3), looking along +x): a box
whose front face is 5.5 m ahead, and a cylinder of radius 0.3 m at (8, -2.5), whose nearest
point is 8.08 m away at -17.4 degrees.
"""

import sys

import numpy as np

CHECKS = {
    "box": dict(win=(5.0, 7.0, -8.0, 8.0), rng=5.5, az=0.0, rng_tol=0.3, az_tol=6.0),
    "cylinder": dict(win=(7.6, 8.8, -22.0, -12.0), rng=8.08, az=-17.4, rng_tol=0.3, az_tol=2.0),
}


def main(a_path: str, b_path: str) -> int:
    a, b = np.load(a_path), np.load(b_path)
    imgs, r, d = a["imgs"], a["ranges"], a["dirs"]
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    img = imgs[-1]
    p99 = float(np.percentile(img, 99))
    ok = True
    print(f"frames {imgs.shape}, median {np.median(img):.1f} dB, 99th percentile {p99:.1f} dB")
    for name, c in CHECKS.items():
        r0, r1, a0, a1 = c["win"]
        rr, aa = (r >= r0) & (r <= r1), (az >= a0) & (az <= a1)
        w = img[np.ix_(rr, aa)]
        i, j = np.unravel_index(np.argmax(w), w.shape)
        pk, pr, pa = float(w.max()), float(r[rr][i]), float(az[aa][j])
        good = pk > p99 and abs(pr - c["rng"]) < c["rng_tol"] and abs(pa - c["az"]) < c["az_tol"]
        ok &= good
        print(
            f"{name:9s} peak {pk:5.1f} dB at {pr:5.2f} m, {pa:6.1f} deg "
            f"(expected {c['rng']:.2f} m, {c['az']:.1f} deg): {'ok' if good else 'FAIL'}"
        )
    same = bool(np.array_equal(a["imgs"], b["imgs"]))
    fresh = not np.array_equal(a["imgs"][0], a["imgs"][1])
    print(f"two runs bit-identical: {same}; consecutive frames differ: {fresh}")
    ok &= same and fresh
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
