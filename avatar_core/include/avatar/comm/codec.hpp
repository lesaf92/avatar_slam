// Avatar wire format v0 encoder/decoder.
//
// Normative spec: docs/spec/wire_format_v0.md (ADR-0005). Must stay byte-identical to the
// Python reference (avatar_py/avatar/comm/codec.py); both are tested against
// testdata/wire_v0_vectors.json.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace avatar::comm {

constexpr std::uint8_t kMagic = 0xA7;
constexpr std::uint8_t kVersion = 0;
constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kPacketOverhead = kHeaderSize + kCrcSize;
constexpr std::size_t kDigestPrefixSize = 3;
constexpr std::size_t kRecordBaseSize = 16;
constexpr std::size_t kFrameBodySize = 30;
constexpr std::size_t kMaxDescriptorDim = 64;

constexpr std::uint8_t kTypeLandmarkDigest = 0x1;
constexpr std::uint8_t kTypeFrameAlignment = 0x2;

constexpr double kPosLsbM = 0.05;
constexpr double kSigmaLsbM = 0.02;
constexpr double kExtentLsbM = 0.25;
constexpr double kDescScale = 127.0;
/// Variance [m²] a receiver must add per axis to decoded positions (spec §3).
constexpr double kPositionQuantVarM2 = kPosLsbM * kPosLsbM / 12.0;

/// Raised for malformed, corrupted, or unencodable packets.
class CodecError : public std::runtime_error {
 public:
  explicit CodecError(const std::string& what) : std::runtime_error(what) {}
};

/// One condensed landmark part, in the sender's local frame.
struct LandmarkRecord {
  std::uint16_t landmark_id{0};
  std::array<double, 3> position{};  ///< [m]
  double sigma_xy{0.0};              ///< isotropic horizontal 1-σ [m]
  double sigma_z{0.0};               ///< vertical 1-σ [m]
  std::array<double, 3> extent{};    ///< footprint x, y and part height [m]
  std::uint8_t class_id{0};
  std::uint8_t flags{0};           ///< avatar::landmark_flags bits
  int n_obs{0};                    ///< saturates at 255 on encode
  std::vector<double> descriptor;  ///< length == descriptor_dim, values in [-1, 1]
};

/// LANDMARK_DIGEST packet.
struct LandmarkDigest {
  std::uint8_t sender_id{0};
  std::uint16_t seq{0};
  std::uint32_t stamp_ms{0};
  std::uint8_t domain{0};
  std::uint8_t descriptor_dim{0};
  std::vector<LandmarkRecord> records;
};

/// FRAME_ALIGNMENT packet: sender's estimate of T_sender_other.
struct FrameAlignment {
  std::uint8_t sender_id{0};
  std::uint16_t seq{0};
  std::uint32_t stamp_ms{0};
  std::uint8_t other_id{0};
  int n_inliers{0};  ///< saturates at 255 on encode
  float x{0.F}, y{0.F}, z{0.F}, yaw{0.F};
  float sigma_xy{0.F}, sigma_z{0.F}, sigma_yaw{0.F};
};

using Message = std::variant<LandmarkDigest, FrameAlignment>;
using Bytes = std::vector<std::uint8_t>;

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
std::uint16_t crc16CcittFalse(const std::uint8_t* data, std::size_t size);

std::size_t recordSize(std::size_t descriptor_dim);
std::size_t digestSize(std::size_t n_records, std::size_t descriptor_dim);
/// Largest record count whose digest packet fits in `mtu_bytes`.
std::size_t maxRecordsPerPacket(std::size_t mtu_bytes, std::size_t descriptor_dim);

Bytes encode(const LandmarkDigest& msg);
Bytes encode(const FrameAlignment& msg);
Bytes encode(const Message& msg);

/// Decode wire bytes; throws CodecError on any inconsistency.
Message decode(const std::uint8_t* data, std::size_t size);
Message decode(const Bytes& data);

}  // namespace avatar::comm
