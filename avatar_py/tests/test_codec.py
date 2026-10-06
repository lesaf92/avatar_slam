import json

import numpy as np
import pytest

from avatar.comm import codec


def test_crc_check_value():
    assert codec.crc16_ccitt_false(b"123456789") == 0x29B1


def test_empty_digest_bytes_match_spec_example():
    msg = codec.LandmarkDigest(3, 42, 1000, 2, 0, ())
    assert codec.encode(msg).hex() == "a70103002a00e8030000020000e962"


def _record(rng, dim):
    return codec.LandmarkRecord(
        landmark_id=int(rng.integers(0, 65536)),
        position=tuple(rng.uniform(-1000, 1000, 3)),
        sigma_xy=float(rng.uniform(0, 3)),
        sigma_z=float(rng.uniform(0, 3)),
        extent=tuple(rng.uniform(0, 30, 3)),
        class_id=int(rng.integers(0, 13)),
        flags=int(rng.choice([1, 2])) | int(rng.choice([0, 4, 8, 16])),
        n_obs=int(rng.integers(1, 400)),
        descriptor=tuple(rng.uniform(-1, 1, dim)),
    )


@pytest.mark.parametrize("dim", [0, 8, 64])
def test_digest_round_trip_within_quantization(rng, dim):
    recs = tuple(_record(rng, dim) for _ in range(5))
    msg = codec.LandmarkDigest(7, 300, 123_456, 3, dim, recs)
    data = codec.encode(msg)
    assert len(data) == codec.digest_size(5, dim)
    out = codec.decode(data)
    assert out.sender_id == 7 and out.seq == 300 and out.stamp_ms == 123_456
    for a, b in zip(recs, out.records, strict=True):
        assert a.landmark_id == b.landmark_id and a.class_id == b.class_id
        assert np.all(np.abs(np.subtract(a.position, b.position)) <= codec.POS_LSB_M / 2 + 1e-9)
        assert abs(a.sigma_xy - b.sigma_xy) <= codec.SIGMA_LSB_M / 2 + 1e-9
        assert min(a.n_obs, 255) == b.n_obs
        assert np.all(np.abs(np.subtract(a.descriptor, b.descriptor)) <= 0.5 / 127 + 1e-9)


def test_frame_alignment_round_trip():
    msg = codec.FrameAlignment(1, 2, 3, 4, 5, 1.5, -2.25, 0.125, 0.5, 0.05, 0.1, 0.01)
    out = codec.decode(codec.encode(msg))
    assert out.other_id == 4 and out.n_inliers == 5
    assert out.x == 1.5 and out.y == -2.25 and out.yaw == pytest.approx(0.5)
    assert len(codec.encode(msg)) == codec.PACKET_OVERHEAD + codec.FRAME_BODY_SIZE


def test_corruption_and_bad_inputs_rejected(rng):
    data = bytearray(codec.encode(codec.LandmarkDigest(1, 1, 1, 0, 0, (_record(rng, 0),))))
    data[12] ^= 0x01
    with pytest.raises(codec.CodecError, match="CRC"):
        codec.decode(bytes(data))
    with pytest.raises(codec.CodecError):
        codec.decode(b"\xa7\x01")
    too_far = codec.LandmarkRecord(1, (2000.0, 0, 0), 0.1, 0.1, (1, 1, 1), 0, 1, 1)
    with pytest.raises(codec.CodecError, match="range"):
        codec.encode(codec.LandmarkDigest(1, 1, 1, 0, 0, (too_far,)))
    bad_sigma = codec.LandmarkRecord(1, (0, 0, 0), -0.1, 0.1, (1, 1, 1), 0, 1, 1)
    with pytest.raises(codec.CodecError):
        codec.encode(codec.LandmarkDigest(1, 1, 1, 0, 0, (bad_sigma,)))


def test_max_records_per_packet_fits_mtu():
    for mtu in (64, 256, 1400):
        for dim in (0, 8, 32):
            n = codec.max_records_per_packet(mtu, dim)
            assert codec.digest_size(n, dim) <= mtu
            if n < 255:
                assert codec.digest_size(n + 1, dim) > mtu


def test_golden_vectors(repo_root):
    doc = json.loads((repo_root / "testdata" / "wire_v0_vectors.json").read_text())
    assert doc["crc16_check"]["crc"] == codec.crc16_ccitt_false(b"123456789")
    for v in doc["vectors"]:
        if v["type"] == "LANDMARK_DIGEST":
            recs = tuple(
                codec.LandmarkRecord(
                    r["landmark_id"],
                    tuple(r["position"]),
                    r["sigma_xy"],
                    r["sigma_z"],
                    tuple(r["extent"]),
                    r["class_id"],
                    r["flags"],
                    r["n_obs"],
                    tuple(r["descriptor"]),
                )
                for r in v["records"]
            )
            msg = codec.LandmarkDigest(
                v["sender_id"], v["seq"], v["stamp_ms"], v["domain"], v["descriptor_dim"], recs
            )
        else:
            msg = codec.FrameAlignment(
                v["sender_id"],
                v["seq"],
                v["stamp_ms"],
                v["other_id"],
                v["n_inliers"],
                v["x"],
                v["y"],
                v["z"],
                v["yaw"],
                v["sigma_xy"],
                v["sigma_z"],
                v["sigma_yaw"],
            )
        assert codec.encode(msg).hex() == v["hex"], v["name"]
        codec.decode(bytes.fromhex(v["hex"]))  # must decode cleanly


def test_measured_footprint_is_never_sent_as_unmeasured():
    """0 x 0 means "not measured" (spec §3); a tiny measured footprint is sent as one LSB."""

    def roundtrip(ext):
        rec = codec.LandmarkRecord(1, (0.0, 0.0, 0.0), 0.1, 0.1, ext, 0, 1, 1, ())
        msg = codec.LandmarkDigest(1, 0, 0, 0, 0, (rec,))
        return codec.decode(codec.encode(msg)).records[0].extent[:2]

    assert roundtrip((0.05, 0.02, 1.0)) == (0.25, 0.0)
    assert roundtrip((0.02, 0.1, 1.0)) == (0.0, 0.25)
    assert roundtrip((0.0, 0.0, 1.0)) == (0.0, 0.0)
    assert roundtrip((0.6, 0.1, 1.0)) == (0.5, 0.0)
