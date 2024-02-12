#ifndef EULER_HPP_INCLUDED
#define EULER_HPP_INCLUDED


#include "constants.hpp"
#include "utils/helpers.hpp"
#include "types.hpp"
#include <array>
#include <eigen3/Eigen/Geometry>

namespace euler {

template<size_t size>
constexpr std::array<size_t, so<size>> axis_order;

template<>
inline constexpr std::array<size_t, so<3>> axis_order<3> {2, 1, 0};

/// Converts euler angles to the corresponding rotation matrix.
/// 
template<size_t size>
Rotation to_rotation(const EulerAngles<so<size>>& euler) {
  Rotation rot = Rotation::Identity();

  for (size_t i = 0; i < so<size>; ++i) {
    // Rotates around each axis with the given angle.
    rot = rot * Rotation(euler[i], Axis::Unit(axis_order<size>[i]));
  }

  return rot;
}

/// Rotate a given @p axis according to the given @p rotation.
///
template<size_t size>
Vector<size> rotate_axis(const EulerAngles<so<size>>& rotation, const Vector<size>& axis) {
  return to_rotation<size>(rotation) * axis;
}

/// Constructs the euler angle vector from a rotation matrix.
/// @note Only implemented for 3D spaces.
template<size_t size>
EulerAngles<so<size>> from(const RotationMatrix& rot_matrix);

template<>
inline EulerAngles<so<3>> from<3>(const RotationMatrix& rot_matrix) {
  return rot_matrix.eulerAngles(axis_order<3>[0], axis_order<3>[1], axis_order<3>[2]);
}

/// Constructs the euler angle vector from a angle axis.
///
template<size_t size>
EulerAngles<so<size>> from(const Rotation& aa) {
  return from<size>(aa.toRotationMatrix());
}

/// Constructs the euler angle vector from a quaternion.
///
template<size_t size>
EulerAngles<so<size>> from(const Quaternion& q) {
  return from<size>(q.toRotationMatrix());
}

/// Constructs the jacobian for conversions from euler angles derivatives to angular velocities.
///
template<size_t size>
Matrix<so<size>> jacobian(const EulerAngles<so<size>>& rotation) {
  Rotation rot = Rotation::Identity();

  Matrix<so<size>> jacobian;

  auto col_it = jacobian.colwise().begin();
  for (size_t i = 0; i < so<size>; ++i) {
    const Axis axis = Axis::Unit(axis_order<size>[i]);

    *col_it++ = rot * axis;

    rot = rot * Rotation(rotation[i], axis);
  }

  return jacobian;
}


} // namespace euler




#endif /* EULER_HPP_INCLUDED */
