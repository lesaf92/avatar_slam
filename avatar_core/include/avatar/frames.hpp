// 4-DoF pose algebra and frame conventions (docs/conventions.md §3-4).
//
// A Pose4 is a gravity-aligned transform [x, y, z, yaw] (metres, radians).
// T_a_b maps coordinates expressed in frame b into frame a: p_a = T_a_b * p_b.
// Mirrors avatar_py/avatar/geometry.py (the reference implementation).
#pragma once

#include <Eigen/Core>

namespace avatar {

/// Wrap an angle to (-pi, pi].
double wrapAngle(double angle_rad);

/// Rotation matrix about +z.
Eigen::Matrix3d rotZ(double yaw_rad);

/// Gravity-aligned 4-DoF transform.
struct Pose4 {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};  ///< translation [m]
  double yaw{0.0};                             ///< rotation about +z [rad]

  Pose4() = default;
  Pose4(double x, double y, double z, double yaw_rad) : p(x, y, z), yaw(wrapAngle(yaw_rad)) {}
  Pose4(const Eigen::Vector3d& t, double yaw_rad) : p(t), yaw(wrapAngle(yaw_rad)) {}

  /// Compose: (*this) ∘ other (apply other first).
  Pose4 operator*(const Pose4& other) const;
  /// Transform a point from the child frame into this frame.
  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const;
  /// Inverse transform.
  Pose4 inverse() const;
  /// Pose `other` expressed in this frame: this⁻¹ ∘ other.
  Pose4 between(const Pose4& other) const;
  /// [x, y, z, yaw] as a vector.
  Eigen::Vector4d vector() const;
};

namespace frames {

/// ENU from NED: (x_e, y_e, z_e) = (y_n, x_n, -z_n). Self-inverse.
Eigen::Vector3d nedToEnu(const Eigen::Vector3d& v);
Eigen::Vector3d enuToNed(const Eigen::Vector3d& v);
/// FLU from FRD: (x, -y, -z). Self-inverse.
Eigen::Vector3d frdToFlu(const Eigen::Vector3d& v);
Eigen::Vector3d fluToFrd(const Eigen::Vector3d& v);
/// Heading clockwise from North (NED) to ENU yaw (counter-clockwise from East).
double yawNedToEnu(double yaw_ned_rad);

}  // namespace frames
}  // namespace avatar
