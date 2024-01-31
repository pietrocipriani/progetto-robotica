#ifndef EULER_HPP_INCLUDED
#define EULER_HPP_INCLUDED


#include "model.hpp"
#include "constants.hpp"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>

namespace euler {

using EulerAngles = model::Vector<kinematics::orientation_size>;
using Rotation = Eigen::AngleAxis<model::Scalar>;
using RotationMatrix = Eigen::AngleAxis<model::Scalar>::RotationMatrixType;

constexpr std::array<size_t, kinematics::orientation_size> axis_order {2, 1, 0};

/**
 * Converts euler angles to the corresponding rotation matrix.
 * ZYX rotation.
 */
Rotation to_rotation(const EulerAngles& euler);

/**
 * Rotate a given @p axis according to the given @p rotation.
 */
model::Vector<kinematics::os_size> rotate_axis(const EulerAngles& rotation, const model::Vector<kinematics::os_size>& axis);

/**
 * Constructs the euler angle vector from a rotation matrix.
 */
EulerAngles from(const RotationMatrix& rot_matrix);

/**
 * Constructs the euler angle vector from a angle axis.
 */
EulerAngles from(const Eigen::AngleAxis<model::Scalar>& aa);

/**
 * Constructs the euler angle vector from a quaternion.
 */
EulerAngles from(const model::Quaternion& q);


model::Matrix<kinematics::orientation_size> jacobian(const EulerAngles& rotation);


}




#endif /* EULER_HPP_INCLUDED */
