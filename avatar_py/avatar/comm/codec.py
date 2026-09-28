"""Wire format v0 encoder/decoder (reference implementation).

Normative specification: ``docs/spec/wire_format_v0.md`` (ADR-0005). The C++
implementation in ``avatar_core`` must produce byte-identical output; both are
checked against ``testdata/wire_v0_vectors.json``.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

MAGIC = 0xA7
VERSION = 0
HEADER_SIZE = 10
CRC_SIZE = 2
PACKET_OVERHEAD = HEADER_SIZE + CRC_SIZE
DIGEST_PREFIX_SIZE = 3
RECORD_BASE_SIZE = 16
FRAME_BODY_SIZE = 30
MAX_DESCRIPTOR_DIM = 64

TYPE_LANDMARK_DIGEST = 0x1
TYPE_FRAME_ALIGNMENT = 0x2

POS_LSB_M = 0.05
SIGMA_LSB_M = 0.02
EXTENT_LSB_M = 0.25
DESC_SCALE = 127.0

POSITION_QUANT_VAR_M2 = POS_LSB_M**2 / 12.0
"""Variance [m²] a receiver must add per axis to decoded positions (spec §3)."""


class CodecError(ValueError):
    """Raised for malformed, corrupted, or unencodable packets."""


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _round_half_away(x: float) -> int:
    """Round half away from zero (matches C++ ``std::round``)."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def _quant_signed16(value_m: float, lsb: float, field: str) -> int:
    q = _round_half_away(value_m / lsb)
    if not -32768 <= q <= 32767:
        raise CodecError(f"{field}={value_m} m is outside the representable range")
    return q


def _quant_unsigned8(value: float, lsb: float, field: str) -> int:
    if not value >= 0.0:  # also rejects NaN
        raise CodecError(f"{field}={value} must be non-negative")
    return min(255, _round_half_away(value / lsb))


@dataclass(frozen=True)
class LandmarkRecord:
    """One condensed landmark part (sender-local frame)."""

    landmark_id: int
    position: tuple[float, float, float]
    sigma_xy: float
    sigma_z: float
    extent: tuple[float, float, float]
    class_id: int
    flags: int
    n_obs: int
    descriptor: tuple[float, ...] = ()


@dataclass(frozen=True)
class LandmarkDigest:
    """``LANDMARK_DIGEST`` packet."""

    sender_id: int
    seq: int
    stamp_ms: int
    domain: int
    descriptor_dim: int
    records: tuple[LandmarkRecord, ...]


@dataclass(frozen=True)
class FrameAlignment:
    """``FRAME_ALIGNMENT`` packet: sender's estimate of ``T_sender_from_other``."""

    sender_id: int
    seq: int
    stamp_ms: int
    other_id: int
    n_inliers: int
    x: float
    y: float
    z: float
    yaw: float
    sigma_xy: float
    sigma_z: float
    sigma_yaw: float


Message = LandmarkDigest | FrameAlignment


def record_size(descriptor_dim: int) -> int:
    """Encoded size [B] of one landmark record."""
    return RECORD_BASE_SIZE + descriptor_dim


def digest_size(n_records: int, descriptor_dim: int) -> int:
    """Encoded size [B] of a digest packet with ``n_records`` records."""
    return PACKET_OVERHEAD + DIGEST_PREFIX_SIZE + n_records * record_size(descriptor_dim)


def max_records_per_packet(mtu_B: int, descriptor_dim: int) -> int:
    """Largest record count whose digest packet fits in ``mtu_B`` bytes."""
    n = (mtu_B - PACKET_OVERHEAD - DIGEST_PREFIX_SIZE) // record_size(descriptor_dim)
    return max(0, min(255, n))


def _header(msg_type: int, sender_id: int, seq: int, stamp_ms: int) -> bytes:
    if not 0 <= sender_id <= 254:
        raise CodecError("sender_id must be in [0, 254]")
    if not 0 <= stamp_ms <= 0xFFFFFFFF:
        raise CodecError("stamp_ms out of range")
    return struct.pack(
        "<BBBBHI", MAGIC, (VERSION << 4) | msg_type, sender_id, 0, seq & 0xFFFF, stamp_ms
    )


def _encode_record(r: LandmarkRecord, dim: int) -> bytes:
    if len(r.descriptor) != dim:
        raise CodecError(f"record {r.landmark_id}: descriptor has {len(r.descriptor)} != {dim}")
    if not 0 <= r.landmark_id <= 0xFFFF:
        raise CodecError("landmark_id must fit in uint16")
    if not 0 <= r.class_id <= 255 or not 0 <= r.flags <= 0x1F:
        raise CodecError("class_id or flags out of range")
    x, y, z = (_quant_signed16(v, POS_LSB_M, "position") for v in r.position)
    ex, ey, ez = (_quant_unsigned8(v, EXTENT_LSB_M, "extent") for v in r.extent)
    head = struct.pack(
        "<HhhhBBBBBBBB",
        r.landmark_id,
        x,
        y,
        z,
        _quant_unsigned8(r.sigma_xy, SIGMA_LSB_M, "sigma_xy"),
        _quant_unsigned8(r.sigma_z, SIGMA_LSB_M, "sigma_z"),
        ex,
        ey,
        ez,
        r.class_id,
        r.flags,
        min(255, max(0, r.n_obs)),
    )
    desc = bytes(
        (_round_half_away(max(-1.0, min(1.0, v)) * DESC_SCALE)) & 0xFF for v in r.descriptor
    )
    return head + desc


def encode(msg: Message) -> bytes:
    """Encode a message to wire bytes (including CRC)."""
    if isinstance(msg, LandmarkDigest):
        if not 0 <= msg.descriptor_dim <= MAX_DESCRIPTOR_DIM:
            raise CodecError("descriptor_dim must be in [0, 64]")
        if len(msg.records) > 255:
            raise CodecError("at most 255 records per packet")
        body = struct.pack("<BBB", msg.domain, msg.descriptor_dim, len(msg.records))
        body += b"".join(_encode_record(r, msg.descriptor_dim) for r in msg.records)
        pkt = _header(TYPE_LANDMARK_DIGEST, msg.sender_id, msg.seq, msg.stamp_ms) + body
    elif isinstance(msg, FrameAlignment):
        body = struct.pack(
            "<BBfffffff",
            msg.other_id,
            min(255, max(0, msg.n_inliers)),
            msg.x,
            msg.y,
            msg.z,
            msg.yaw,
            msg.sigma_xy,
            msg.sigma_z,
            msg.sigma_yaw,
        )
        pkt = _header(TYPE_FRAME_ALIGNMENT, msg.sender_id, msg.seq, msg.stamp_ms) + body
    else:  # pragma: no cover - type guard
        raise CodecError(f"cannot encode {type(msg).__name__}")
    return pkt + struct.pack("<H", crc16_ccitt_false(pkt))


def decode(data: bytes) -> Message:
    """Decode wire bytes; raises :class:`CodecError` on any inconsistency."""
    if len(data) < PACKET_OVERHEAD:
        raise CodecError("packet too short")
    (crc,) = struct.unpack_from("<H", data, len(data) - CRC_SIZE)
    if crc != crc16_ccitt_false(data[:-CRC_SIZE]):
        raise CodecError("CRC mismatch")
    magic, vt, sender, flags, seq, stamp = struct.unpack_from("<BBBBHI", data, 0)
    if magic != MAGIC:
        raise CodecError("bad magic")
    if vt >> 4 != VERSION:
        raise CodecError(f"unsupported version {vt >> 4}")
    if flags != 0:
        raise CodecError("reserved header flags must be zero")
    body = data[HEADER_SIZE:-CRC_SIZE]
    msg_type = vt & 0x0F
    if msg_type == TYPE_LANDMARK_DIGEST:
        if len(body) < DIGEST_PREFIX_SIZE:
            raise CodecError("digest body too short")
        domain, dim, count = struct.unpack_from("<BBB", body, 0)
        if dim > MAX_DESCRIPTOR_DIM:
            raise CodecError("descriptor_dim > 64")
        rs = record_size(dim)
        if len(body) != DIGEST_PREFIX_SIZE + count * rs:
            raise CodecError("digest length does not match record count")
        records = []
        for i in range(count):
            off = DIGEST_PREFIX_SIZE + i * rs
            lid, x, y, z, sxy, sz, ex, ey, ez, cls, fl, nobs = struct.unpack_from(
                "<HhhhBBBBBBBB", body, off
            )
            if fl & ~0x1F:
                raise CodecError("reserved landmark flag bits set")
            desc_raw = struct.unpack_from(f"<{dim}b", body, off + RECORD_BASE_SIZE) if dim else ()
            records.append(
                LandmarkRecord(
                    landmark_id=lid,
                    position=(x * POS_LSB_M, y * POS_LSB_M, z * POS_LSB_M),
                    sigma_xy=sxy * SIGMA_LSB_M,
                    sigma_z=sz * SIGMA_LSB_M,
                    extent=(ex * EXTENT_LSB_M, ey * EXTENT_LSB_M, ez * EXTENT_LSB_M),
                    class_id=cls,
                    flags=fl,
                    n_obs=nobs,
                    descriptor=tuple(v / DESC_SCALE for v in desc_raw),
                )
            )
        return LandmarkDigest(sender, seq, stamp, domain, dim, tuple(records))
    if msg_type == TYPE_FRAME_ALIGNMENT:
        if len(body) != FRAME_BODY_SIZE:
            raise CodecError("frame-alignment body has wrong length")
        other, n_in, *vals = struct.unpack("<BBfffffff", body)
        return FrameAlignment(sender, seq, stamp, other, n_in, *vals)
    raise CodecError(f"unknown message type {msg_type:#x}")
