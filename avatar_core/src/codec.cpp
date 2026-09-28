#include "avatar/comm/codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>

#include "avatar/types.hpp"

namespace avatar::comm {
namespace {

// Normative rounding (spec §3): q = copysign(floor(|v| + 0.5), v) in binary64.
double roundHalfAway(double v) { return std::copysign(std::floor(std::fabs(v) + 0.5), v); }

std::int16_t quantSigned16(double value_m, double lsb, const char* field) {
  const double q = roundHalfAway(value_m / lsb);
  if (!(q >= -32768.0 && q <= 32767.0)) {  // also rejects NaN
    throw CodecError(std::string(field) + " is outside the representable range");
  }
  return static_cast<std::int16_t>(q);
}

std::uint8_t quantUnsigned8(double value, double lsb, const char* field) {
  if (!(value >= 0.0)) {  // also rejects NaN
    throw CodecError(std::string(field) + " must be non-negative");
  }
  return static_cast<std::uint8_t>(std::min(255.0, roundHalfAway(value / lsb)));
}

std::uint8_t saturateU8(int v) { return static_cast<std::uint8_t>(std::clamp(v, 0, 255)); }

class Writer {
 public:
  void u8(std::uint8_t v) { buf_.push_back(v); }
  void i8(std::int8_t v) { buf_.push_back(static_cast<std::uint8_t>(v)); }
  void u16(std::uint16_t v) {
    buf_.push_back(static_cast<std::uint8_t>(v & 0xFFU));
    buf_.push_back(static_cast<std::uint8_t>((v >> 8U) & 0xFFU));
  }
  void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
  void u32(std::uint32_t v) {
    for (unsigned s = 0; s < 32; s += 8)
      buf_.push_back(static_cast<std::uint8_t>((v >> s) & 0xFFU));
  }
  void f32(float v) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof bits);
    u32(bits);
  }
  Bytes& bytes() { return buf_; }

 private:
  Bytes buf_;
};

class Reader {
 public:
  Reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}
  std::uint8_t u8() {
    need(1);
    return data_[pos_++];
  }
  std::int8_t i8() { return static_cast<std::int8_t>(u8()); }
  std::uint16_t u16() {
    need(2);
    const auto v = static_cast<std::uint16_t>(data_[pos_] | (data_[pos_ + 1] << 8U));
    pos_ += 2;
    return v;
  }
  std::int16_t i16() { return static_cast<std::int16_t>(u16()); }
  std::uint32_t u32() {
    need(4);
    std::uint32_t v = 0;
    for (unsigned i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(data_[pos_ + i]) << (8U * i);
    pos_ += 4;
    return v;
  }
  float f32() {
    const std::uint32_t bits = u32();
    float v = 0.F;
    std::memcpy(&v, &bits, sizeof v);
    return v;
  }
  std::size_t remaining() const { return size_ - pos_; }

 private:
  void need(std::size_t n) const {
    if (pos_ + n > size_) throw CodecError("unexpected end of packet");
  }
  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t pos_{0};
};

void writeHeader(Writer& w, std::uint8_t type, std::uint8_t sender, std::uint16_t seq,
                 std::uint32_t stamp_ms) {
  if (sender > 254) throw CodecError("sender_id must be in [0, 254]");
  w.u8(kMagic);
  w.u8(static_cast<std::uint8_t>((kVersion << 4U) | type));
  w.u8(sender);
  w.u8(0);  // reserved flags
  w.u16(seq);
  w.u32(stamp_ms);
}

Bytes finish(Writer& w) {
  Bytes& b = w.bytes();
  const std::uint16_t crc = crc16CcittFalse(b.data(), b.size());
  w.u16(crc);
  return b;
}

}  // namespace

std::uint16_t crc16CcittFalse(const std::uint8_t* data, std::size_t size) {
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(data[i]) << 8U));
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                            : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::size_t recordSize(std::size_t descriptor_dim) { return kRecordBaseSize + descriptor_dim; }

std::size_t digestSize(std::size_t n_records, std::size_t descriptor_dim) {
  return kPacketOverhead + kDigestPrefixSize + n_records * recordSize(descriptor_dim);
}

std::size_t maxRecordsPerPacket(std::size_t mtu_bytes, std::size_t descriptor_dim) {
  if (mtu_bytes < kPacketOverhead + kDigestPrefixSize) return 0;
  const std::size_t n =
      (mtu_bytes - kPacketOverhead - kDigestPrefixSize) / recordSize(descriptor_dim);
  return std::min<std::size_t>(n, 255);
}

Bytes encode(const LandmarkDigest& msg) {
  if (msg.descriptor_dim > kMaxDescriptorDim) throw CodecError("descriptor_dim must be <= 64");
  if (msg.records.size() > 255) throw CodecError("at most 255 records per packet");
  Writer w;
  writeHeader(w, kTypeLandmarkDigest, msg.sender_id, msg.seq, msg.stamp_ms);
  w.u8(msg.domain);
  w.u8(msg.descriptor_dim);
  w.u8(static_cast<std::uint8_t>(msg.records.size()));
  for (const auto& r : msg.records) {
    if (r.descriptor.size() != msg.descriptor_dim) {
      throw CodecError("record descriptor length does not match descriptor_dim");
    }
    if ((r.flags & ~landmark_flags::kValidMask) != 0) throw CodecError("invalid landmark flags");
    w.u16(r.landmark_id);
    for (double v : r.position) w.i16(quantSigned16(v, kPosLsbM, "position"));
    w.u8(quantUnsigned8(r.sigma_xy, kSigmaLsbM, "sigma_xy"));
    w.u8(quantUnsigned8(r.sigma_z, kSigmaLsbM, "sigma_z"));
    for (double v : r.extent) w.u8(quantUnsigned8(v, kExtentLsbM, "extent"));
    w.u8(r.class_id);
    w.u8(r.flags);
    w.u8(saturateU8(r.n_obs));
    for (double v : r.descriptor) {
      const double c = std::clamp(v, -1.0, 1.0);
      w.i8(static_cast<std::int8_t>(roundHalfAway(c * kDescScale)));
    }
  }
  return finish(w);
}

Bytes encode(const FrameAlignment& msg) {
  Writer w;
  writeHeader(w, kTypeFrameAlignment, msg.sender_id, msg.seq, msg.stamp_ms);
  w.u8(msg.other_id);
  w.u8(saturateU8(msg.n_inliers));
  for (float v : {msg.x, msg.y, msg.z, msg.yaw, msg.sigma_xy, msg.sigma_z, msg.sigma_yaw}) {
    w.f32(v);
  }
  return finish(w);
}

Bytes encode(const Message& msg) {
  return std::visit([](const auto& m) { return encode(m); }, msg);
}

Message decode(const std::uint8_t* data, std::size_t size) {
  if (size < kPacketOverhead) throw CodecError("packet too short");
  const std::uint16_t crc = static_cast<std::uint16_t>(data[size - 2] | (data[size - 1] << 8U));
  if (crc != crc16CcittFalse(data, size - kCrcSize)) throw CodecError("CRC mismatch");
  Reader r(data, size - kCrcSize);
  if (r.u8() != kMagic) throw CodecError("bad magic");
  const std::uint8_t vt = r.u8();
  if ((vt >> 4U) != kVersion) throw CodecError("unsupported version");
  const std::uint8_t sender = r.u8();
  if (r.u8() != 0) throw CodecError("reserved header flags must be zero");
  const std::uint16_t seq = r.u16();
  const std::uint32_t stamp = r.u32();
  const std::uint8_t type = vt & 0x0FU;

  if (type == kTypeLandmarkDigest) {
    LandmarkDigest d;
    d.sender_id = sender;
    d.seq = seq;
    d.stamp_ms = stamp;
    d.domain = r.u8();
    d.descriptor_dim = r.u8();
    const std::uint8_t count = r.u8();
    if (d.descriptor_dim > kMaxDescriptorDim) throw CodecError("descriptor_dim > 64");
    if (r.remaining() != count * recordSize(d.descriptor_dim)) {
      throw CodecError("digest length does not match record count");
    }
    d.records.reserve(count);
    for (std::uint8_t i = 0; i < count; ++i) {
      LandmarkRecord rec;
      rec.landmark_id = r.u16();
      for (double& v : rec.position) v = r.i16() * kPosLsbM;
      rec.sigma_xy = r.u8() * kSigmaLsbM;
      rec.sigma_z = r.u8() * kSigmaLsbM;
      for (double& v : rec.extent) v = r.u8() * kExtentLsbM;
      rec.class_id = r.u8();
      rec.flags = r.u8();
      if ((rec.flags & ~landmark_flags::kValidMask) != 0) {
        throw CodecError("reserved landmark flag bits set");
      }
      rec.n_obs = r.u8();
      rec.descriptor.resize(d.descriptor_dim);
      for (double& v : rec.descriptor) v = r.i8() / kDescScale;
      d.records.push_back(std::move(rec));
    }
    return d;
  }
  if (type == kTypeFrameAlignment) {
    if (r.remaining() != kFrameBodySize) throw CodecError("frame-alignment body has wrong length");
    FrameAlignment f;
    f.sender_id = sender;
    f.seq = seq;
    f.stamp_ms = stamp;
    f.other_id = r.u8();
    f.n_inliers = r.u8();
    f.x = r.f32();
    f.y = r.f32();
    f.z = r.f32();
    f.yaw = r.f32();
    f.sigma_xy = r.f32();
    f.sigma_z = r.f32();
    f.sigma_yaw = r.f32();
    return f;
  }
  throw CodecError("unknown message type");
}

Message decode(const Bytes& data) { return decode(data.data(), data.size()); }

}  // namespace avatar::comm
