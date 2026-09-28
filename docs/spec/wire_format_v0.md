# Avatar wire format, version 0

**Status:** contract (ADR-0005). Implementations:
`avatar_py/avatar/comm/codec.py`, `avatar_core/src/codec.cpp`.
Golden vectors: `testdata/wire_v0_vectors.json`.

The format is designed for the **worst link in the team**, the acoustic modem
(10²–10⁴ bps, packets of tens to hundreds of bytes, frequent loss). Every
packet is self-contained and independently decodable, so losing one packet
loses only its own records.

## 1. General rules

- Byte order: **little-endian** for all multi-byte integers and floats.
- Floats: IEEE-754 binary32.
- Integers are unsigned unless stated otherwise.
- Quantized fields **saturate** (clamp) at their representable range, except
  landmark positions: an out-of-range position is an **encoding error**. The
  sender must not transmit it and should re-centre or skip.
- Receivers must reject packets with a bad magic, an unknown version, a CRC
  mismatch, or a length that does not match the declared counts.

## 2. Packet framing

| Offset | Size | Type | Field | Notes |
|---:|---:|---|---|---|
| 0 | 1 | u8 | `magic` | `0xA7` |
| 1 | 1 | u8 | `version_type` | high nibble = version (`0`), low nibble = message type |
| 2 | 1 | u8 | `sender_id` | **originating** agent id (0–254); relays keep it (ADR-0006) |
| 3 | 1 | u8 | `flags` | reserved, must be `0` in v0 |
| 4 | 2 | u16 | `seq` | per-sender sequence number, wraps |
| 6 | 4 | u32 | `stamp_ms` | milliseconds since mission epoch |
| 10 | N | – | `body` | depends on type |
| 10+N | 2 | u16 | `crc16` | CRC-16/CCITT-FALSE over bytes `[0, 10+N)` |

Overhead: **12 bytes** per packet.

CRC-16/CCITT-FALSE: polynomial `0x1021`, init `0xFFFF`, no reflection,
xor-out `0x0000`. Check value: `crc16("123456789") = 0x29B1`.

### Message types

| Code | Name | Body |
|---:|---|---|
| `0x1` | `LANDMARK_DIGEST` | §3 |
| `0x2` | `FRAME_ALIGNMENT` | §4 |
| `0x3`–`0xF` | reserved | – |

## 3. `LANDMARK_DIGEST` body

| Size | Type | Field | Notes |
|---:|---|---|---|
| 1 | u8 | `domain` | sender `Domain` code (conventions §5) |
| 1 | u8 | `descriptor_dim` (D) | 0–64 |
| 1 | u8 | `count` (K) | number of records, 0–255 |
| K × (16 + D) | – | records | below |

Landmark record (16 + D bytes):

| Size | Type | Field | Quantization |
|---:|---|---|---|
| 2 | u16 | `landmark_id` | sender-private id |
| 2 | i16 | `x` | 0.05 m / LSB (range ±1638.35 m), sender local frame |
| 2 | i16 | `y` | 0.05 m / LSB |
| 2 | i16 | `z` | 0.05 m / LSB |
| 1 | u8 | `sigma_xy` | 0.02 m / LSB, saturates at 5.10 m (isotropic horizontal std) |
| 1 | u8 | `sigma_z` | 0.02 m / LSB, saturates at 5.10 m |
| 1 | u8 | `extent_x` | 0.25 m / LSB, saturates at 63.75 m |
| 1 | u8 | `extent_y` | 0.25 m / LSB |
| 1 | u8 | `extent_z` | 0.25 m / LSB (extent of the *observed part*) |
| 1 | u8 | `class_id` | `avatar.semantics.CLASS_NAMES` index |
| 1 | u8 | `flags` | bit0 `ABOVE`, bit1 `BELOW`, bit2 `CAMERA`, bit3 `LIDAR`, bit4 `SONAR`, bits 5–7 reserved (0) |
| 1 | u8 | `n_obs` | observation count, saturates at 255 |
| D | i8 | `descriptor[D]` | `round(clip(v, -1, 1) * 127)` |

Rounding (normative, so that every implementation produces identical bytes):
compute `v = value / lsb` in IEEE-754 binary64, then
`q = copysign(floor(|v| + 0.5), v)` (half away from zero). Use this exact
formula, not a library `round`, because `floor(|v| + 0.5)` and `std::round`
differ for `|v| = 0.49999999999999994`. Unsigned fields then saturate:
`q = min(255, q)`. Descriptor values use `v = clip(value, -1, 1) * 127`.
Negative σ or extents are invalid.

Example: an empty digest from agent 3 (domain `SURFACE`), `seq = 42`,
`stamp_ms = 1000` encodes as `a7 01 03 00 2a 00 e8 03 00 00 02 00 00 e9 62`.

**Receiver-side covariance.** A receiver must inflate the decoded σ by the
quantization noise: `σ² ← σ² + (0.05 m)² / 12` on each position axis.

Size example: with K = 20 and D = 0, the packet is 12 + 3 + 20 × 16 = 335 bytes.
On a 1000 bps acoustic link that takes ≈ 2.7 s of channel time.

## 4. `FRAME_ALIGNMENT` body

The sender's estimate of `T_sender_from_other` (maps points in the other
agent's local frame into the sender's local frame).

| Size | Type | Field |
|---:|---|---|
| 1 | u8 | `other_id` |
| 1 | u8 | `n_inliers` (saturating) |
| 4 | f32 | `x` [m] |
| 4 | f32 | `y` [m] |
| 4 | f32 | `z` [m] |
| 4 | f32 | `yaw` [rad], wrapped to (-π, π] |
| 4 | f32 | `sigma_xy` [m] |
| 4 | f32 | `sigma_z` [m] |
| 4 | f32 | `sigma_yaw` [rad] |

Body = 30 bytes; packet = 42 bytes.

## 5. Relays (ADR-0006)

A relay (e.g. the surface gateway) may re-encode an originator's
`LANDMARK_DIGEST` records into new packets. It must keep the originator's
`sender_id`, it may set `descriptor_dim = 0`, and it must not change any other
record field. Receivers must keep a record's earlier descriptor when a copy
without one arrives. This clarifies §2. The bytes of v0 are unchanged.

## 6. Versioning

A future v1 must use version nibble `1`. Receivers must ignore packets with an
unknown version rather than guess. The golden-vector file holds one section
per version.
