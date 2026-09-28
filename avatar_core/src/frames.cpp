#include "avatar/frames.hpp"

#include <cmath>

namespace avatar {
namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double wrapAngle(double angle_rad) {
  double w = std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  if (w == -kPi) {
    w = kPi;  // atan2(-0, -1) = -pi exactly; map to +pi for the half-open interval (-pi, pi]
  }
  return w;
}

Eigen::Matrix3d rotZ(double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Eigen::Matrix3d r;
  r << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return r;
}

Pose4 Pose4::operator*(const Pose4& other) const {
  return Pose4(p + rotZ(yaw) * other.p, yaw + other.yaw);
}

Eigen::Vector3d Pose4::operator*(const Eigen::Vector3d& point) const {
  return p + rotZ(yaw) * point;
}

Pose4 Pose4::inverse() const { return Pose4(-(rotZ(-yaw) * p), -yaw); }

Pose4 Pose4::between(const Pose4& other) const { return inverse() * other; }

Eigen::Vector4d Pose4::vector() const { return {p.x(), p.y(), p.z(), yaw}; }

namespace frames {

Eigen::Vector3d nedToEnu(const Eigen::Vector3d& v) { return {v.y(), v.x(), -v.z()}; }
Eigen::Vector3d enuToNed(const Eigen::Vector3d& v) { return nedToEnu(v); }
Eigen::Vector3d frdToFlu(const Eigen::Vector3d& v) { return {v.x(), -v.y(), -v.z()}; }
Eigen::Vector3d fluToFrd(const Eigen::Vector3d& v) { return frdToFlu(v); }
double yawNedToEnu(double yaw_ned_rad) { return wrapAngle(kPi / 2.0 - yaw_ned_rad); }

}  // namespace frames
}  // namespace avatar
