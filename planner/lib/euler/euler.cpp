#include "euler.hpp"
#include "constants.hpp"

namespace euler {

using namespace model;

Rotation to_rotation(const EulerAngles& euler) {
  using Rotation = Eigen::AngleAxis<Scalar>;

  Rotation rot = Rotation::Identity();

  for (size_t i = 0; i < kinematics::orientation_size; ++i) {
    rot = rot * Rotation(euler[i], Rotation::VectorType::Unit(axis_order[i]));
  }

  return rot;
}

Vector<kinematics::os_size> rotate_axis(const EulerAngles& rotation, const Vector<kinematics::os_size>& axis) {
  return to_rotation(rotation) * axis;
}

EulerAngles from(const RotationMatrix& rot_matrix) {
  return rot_matrix.eulerAngles(axis_order[0], axis_order[1], axis_order[2]);
}

EulerAngles from(const Eigen::AngleAxis<Scalar>& aa) {
  return from(aa.toRotationMatrix());
}

EulerAngles from(const Quaternion& q) {
  return from(q.toRotationMatrix());
}

Matrix<kinematics::orientation_size> jacobian(const EulerAngles& rotation) {
  using Rotation = Eigen::AngleAxis<Scalar>;
  using Axis = Rotation::VectorType;

  Rotation rot = Rotation::Identity();

  Matrix<kinematics::orientation_size> jacobian;

  auto col_it = jacobian.colwise().begin();
  for (size_t i = 0; i < kinematics::orientation_size; ++i) {
    const Axis axis = Axis::Unit(axis_order[i]);

    *col_it++ = rot * axis;

    rot = rot * Rotation(rotation[i], Rotation::VectorType::Unit(axis_order[i]));
  }

  return jacobian;
}



}
