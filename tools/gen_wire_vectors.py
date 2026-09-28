#!/usr/bin/env python3
"""Generate ``testdata/wire_v0_vectors.json`` from the Python reference codec.

Run only when the wire spec changes (ADR-0005): the vectors are a contract
checked by both the Python and C++ test suites.

    python tools/gen_wire_vectors.py > testdata/wire_v0_vectors.json
"""

from __future__ import annotations

import json
import sys

from avatar.comm import codec


def record_dict(r: codec.LandmarkRecord) -> dict:
    return {
        "landmark_id": r.landmark_id,
        "position": list(r.position),
        "sigma_xy": r.sigma_xy,
        "sigma_z": r.sigma_z,
        "extent": list(r.extent),
        "class_id": r.class_id,
        "flags": r.flags,
        "n_obs": r.n_obs,
        "descriptor": list(r.descriptor),
    }


def main() -> None:
    vectors = []

    empty = codec.LandmarkDigest(
        sender_id=3, seq=42, stamp_ms=1000, domain=2, descriptor_dim=0, records=()
    )
    vectors.append(
        {
            "name": "digest_empty",
            "type": "LANDMARK_DIGEST",
            "sender_id": 3,
            "seq": 42,
            "stamp_ms": 1000,
            "domain": 2,
            "descriptor_dim": 0,
            "records": [],
            "hex": codec.encode(empty).hex(),
        }
    )

    recs = (
        codec.LandmarkRecord(
            17,
            (12.34, -4.2, -3.0),
            0.1,
            0.83,
            (0.6, 0.6, 12.1),
            1,
            0x02 | 0x10,
            7,
            (0.5, -0.25, 1.0, -1.0),
        ),
        codec.LandmarkRecord(
            65535,
            (-1638.35, 1638.35, 0.024),
            9.0,
            0.0,
            (100.0, 0.1, 0.0),
            3,
            0x01 | 0x04 | 0x08,
            300,
            (0.0, 0.004, -0.996, 0.33),
        ),
    )
    dig = codec.LandmarkDigest(
        sender_id=0,
        seq=65535,
        stamp_ms=4_000_000_000,
        domain=3,
        descriptor_dim=4,
        records=recs,
    )
    vectors.append(
        {
            "name": "digest_two_records",
            "type": "LANDMARK_DIGEST",
            "sender_id": 0,
            "seq": 65535,
            "stamp_ms": 4_000_000_000,
            "domain": 3,
            "descriptor_dim": 4,
            "records": [record_dict(r) for r in recs],
            "hex": codec.encode(dig).hex(),
        }
    )

    fa = codec.FrameAlignment(
        sender_id=4,
        seq=7,
        stamp_ms=123456,
        other_id=1,
        n_inliers=12,
        x=1.5,
        y=-2.25,
        z=0.125,
        yaw=0.5,
        sigma_xy=0.05,
        sigma_z=0.1,
        sigma_yaw=0.01,
    )
    vectors.append(
        {
            "name": "frame_alignment",
            "type": "FRAME_ALIGNMENT",
            "sender_id": 4,
            "seq": 7,
            "stamp_ms": 123456,
            "other_id": 1,
            "n_inliers": 12,
            "x": 1.5,
            "y": -2.25,
            "z": 0.125,
            "yaw": 0.5,
            "sigma_xy": 0.05,
            "sigma_z": 0.1,
            "sigma_yaw": 0.01,
            "hex": codec.encode(fa).hex(),
        }
    )

    out = {
        "spec": "docs/spec/wire_format_v0.md",
        "version": 0,
        "crc16_check": {
            "ascii": "123456789",
            "crc": codec.crc16_ccitt_false(b"123456789"),
        },
        "vectors": vectors,
    }
    json.dump(out, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
