#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "avatar/frames.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

avatar::Pose4 randomPose(std::mt19937& gen) {
  std::uniform_real_distribution<double> t(-50.0, 50.0);
  std::uniform_real_distribution<double> a(-kPi, kPi);
  return {t(gen), t(gen), t(gen), a(gen)};
}

}  // namespace

TEST(Frames, WrapAngleHalfOpen) {
  EXPECT_DOUBLE_EQ(avatar::wrapAngle(-kPi), kPi);
  EXPECT_NEAR(avatar::wrapAngle(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    const double w = avatar::wrapAngle(a);
    EXPECT_GT(w, -kPi);
    EXPECT_LE(w, kPi);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-12);
  }
}

TEST(Frames, ComposeInverseIdentity) {
  std::mt19937 gen(7);
  for (int i = 0; i < 50; ++i) {
    const auto a = randomPose(gen);
    const auto id = a * a.inverse();
    EXPECT_NEAR(id.p.norm(), 0.0, 1e-9);
    EXPECT_NEAR(id.yaw, 0.0, 1e-12);
  }
}

TEST(Frames, BetweenAndPointTransformConsistent) {
  std::mt19937 gen(11);
  const auto a = randomPose(gen);
  const auto b = randomPose(gen);
  const auto rel = a.between(b);
  EXPECT_TRUE(((a * rel).p - b.p).isZero(1e-9));
  const Eigen::Vector3d q(1.0, -2.0, 3.0);
  EXPECT_TRUE((a.inverse() * (a * q) - q).isZero(1e-9));
}

TEST(Frames, MatchesPythonReferenceValues) {
  // compose([1,2,3,pi/2], [1,0,0,0]) == [1,3,3,pi/2] (avatar.geometry.compose)
  const avatar::Pose4 a(1.0, 2.0, 3.0, kPi / 2.0);
  const avatar::Pose4 b(1.0, 0.0, 0.0, 0.0);
  const auto c = a * b;
  EXPECT_NEAR(c.p.x(), 1.0, 1e-12);
  EXPECT_NEAR(c.p.y(), 3.0, 1e-12);
  EXPECT_NEAR(c.p.z(), 3.0, 1e-12);
  EXPECT_NEAR(c.yaw, kPi / 2.0, 1e-12);
}

TEST(Frames, NedEnuConversions) {
  const Eigen::Vector3d ned(1.0, 2.0, 3.0);  // north, east, down
  const auto enu = avatar::frames::nedToEnu(ned);
  EXPECT_TRUE(enu.isApprox(Eigen::Vector3d(2.0, 1.0, -3.0)));
  EXPECT_TRUE(avatar::frames::enuToNed(enu).isApprox(ned));
  EXPECT_TRUE(avatar::frames::frdToFlu({1.0, 2.0, 3.0}).isApprox(Eigen::Vector3d(1.0, -2.0, -3.0)));
  EXPECT_NEAR(avatar::frames::yawNedToEnu(0.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(avatar::frames::yawNedToEnu(kPi / 2.0), 0.0, 1e-12);
}
