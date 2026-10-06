#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

#include "avatar/comm/codec.hpp"

using avatar::comm::Bytes;
using avatar::comm::CodecError;
using avatar::comm::FrameAlignment;
using avatar::comm::LandmarkDigest;
using avatar::comm::LandmarkRecord;

namespace {

std::string toHex(const Bytes& b) {
  std::string s;
  char buf[3];
  for (auto v : b) {
    std::snprintf(buf, sizeof buf, "%02x", v);
    s += buf;
  }
  return s;
}

Bytes fromHex(const std::string& h) {
  Bytes b;
  for (std::size_t i = 0; i + 1 < h.size(); i += 2) {
    b.push_back(static_cast<std::uint8_t>(std::stoul(h.substr(i, 2), nullptr, 16)));
  }
  return b;
}

nlohmann::json loadVectors() {
  std::ifstream f(std::string(AVATAR_TESTDATA_DIR) + "/wire_v0_vectors.json");
  EXPECT_TRUE(f.good()) << "missing testdata/wire_v0_vectors.json";
  return nlohmann::json::parse(f);
}

}  // namespace

TEST(Codec, CrcCheckValue) {
  const std::string s = "123456789";
  EXPECT_EQ(
      avatar::comm::crc16CcittFalse(reinterpret_cast<const std::uint8_t*>(s.data()), s.size()),
      0x29B1);
}

TEST(Codec, EmptyDigestMatchesSpecExample) {
  LandmarkDigest d;
  d.sender_id = 3;
  d.seq = 42;
  d.stamp_ms = 1000;
  d.domain = 2;
  EXPECT_EQ(toHex(avatar::comm::encode(d)), "a70103002a00e8030000020000e962");
}

TEST(Codec, GoldenVectorsMatchPythonReference) {
  const auto doc = loadVectors();
  ASSERT_EQ(doc.at("version").get<int>(), 0);
  for (const auto& v : doc.at("vectors")) {
    SCOPED_TRACE(v.at("name").get<std::string>());
    Bytes encoded;
    if (v.at("type") == "LANDMARK_DIGEST") {
      LandmarkDigest d;
      d.sender_id = v.at("sender_id").get<std::uint8_t>();
      d.seq = v.at("seq").get<std::uint16_t>();
      d.stamp_ms = v.at("stamp_ms").get<std::uint32_t>();
      d.domain = v.at("domain").get<std::uint8_t>();
      d.descriptor_dim = v.at("descriptor_dim").get<std::uint8_t>();
      for (const auto& jr : v.at("records")) {
        LandmarkRecord r;
        r.landmark_id = jr.at("landmark_id").get<std::uint16_t>();
        for (int i = 0; i < 3; ++i) {
          r.position[i] = jr.at("position")[i].get<double>();
          r.extent[i] = jr.at("extent")[i].get<double>();
        }
        r.sigma_xy = jr.at("sigma_xy").get<double>();
        r.sigma_z = jr.at("sigma_z").get<double>();
        r.class_id = jr.at("class_id").get<std::uint8_t>();
        r.flags = jr.at("flags").get<std::uint8_t>();
        r.n_obs = jr.at("n_obs").get<int>();
        r.descriptor = jr.at("descriptor").get<std::vector<double>>();
        d.records.push_back(r);
      }
      encoded = avatar::comm::encode(d);
    } else {
      FrameAlignment f;
      f.sender_id = v.at("sender_id").get<std::uint8_t>();
      f.seq = v.at("seq").get<std::uint16_t>();
      f.stamp_ms = v.at("stamp_ms").get<std::uint32_t>();
      f.other_id = v.at("other_id").get<std::uint8_t>();
      f.n_inliers = v.at("n_inliers").get<int>();
      f.x = static_cast<float>(v.at("x").get<double>());
      f.y = static_cast<float>(v.at("y").get<double>());
      f.z = static_cast<float>(v.at("z").get<double>());
      f.yaw = static_cast<float>(v.at("yaw").get<double>());
      f.sigma_xy = static_cast<float>(v.at("sigma_xy").get<double>());
      f.sigma_z = static_cast<float>(v.at("sigma_z").get<double>());
      f.sigma_yaw = static_cast<float>(v.at("sigma_yaw").get<double>());
      encoded = avatar::comm::encode(f);
    }
    EXPECT_EQ(toHex(encoded), v.at("hex").get<std::string>());
    // Decode + re-encode is the identity on valid packets.
    const auto msg = avatar::comm::decode(fromHex(v.at("hex").get<std::string>()));
    EXPECT_EQ(toHex(avatar::comm::encode(msg)), v.at("hex").get<std::string>());
  }
}

TEST(Codec, RejectsCorruptionAndBadInput) {
  LandmarkDigest d;
  d.records.push_back(LandmarkRecord{1, {1.0, 2.0, 3.0}, 0.1, 0.2, {1.0, 1.0, 1.0}, 1, 1, 3, {}});
  Bytes b = avatar::comm::encode(d);
  b[12] ^= 0x01U;
  EXPECT_THROW(avatar::comm::decode(b), CodecError);
  EXPECT_THROW(avatar::comm::decode(Bytes{0xA7, 0x01}), CodecError);
  d.records[0].position[0] = 2000.0;
  EXPECT_THROW(avatar::comm::encode(d), CodecError);
  d.records[0].position[0] = 0.0;
  d.records[0].sigma_xy = -1.0;
  EXPECT_THROW(avatar::comm::encode(d), CodecError);
}

TEST(Codec, MaxRecordsFitsMtu) {
  for (std::size_t mtu : {64U, 256U, 1400U}) {
    for (std::size_t dim : {0U, 8U, 32U}) {
      const auto n = avatar::comm::maxRecordsPerPacket(mtu, dim);
      EXPECT_LE(avatar::comm::digestSize(n, dim), mtu);
      if (n < 255) {
        EXPECT_GT(avatar::comm::digestSize(n + 1, dim), mtu);
      }
    }
  }
}

TEST(Codec, MeasuredFootprintIsNeverSentAsUnmeasured) {
  // 0 x 0 means "not measured" (spec §3); a tiny measured footprint is sent as one LSB.
  auto roundtrip = [](double x, double y) {
    LandmarkDigest d;
    LandmarkRecord r;
    r.extent = {x, y, 1.0};
    r.flags = 1;
    r.n_obs = 1;
    d.records.push_back(r);
    const auto out = std::get<LandmarkDigest>(avatar::comm::decode(avatar::comm::encode(d)));
    return std::array<double, 2>{out.records[0].extent[0], out.records[0].extent[1]};
  };
  EXPECT_EQ(roundtrip(0.05, 0.02), (std::array<double, 2>{0.25, 0.0}));
  EXPECT_EQ(roundtrip(0.02, 0.1), (std::array<double, 2>{0.0, 0.25}));
  EXPECT_EQ(roundtrip(0.0, 0.0), (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(roundtrip(0.6, 0.1), (std::array<double, 2>{0.5, 0.0}));
}
