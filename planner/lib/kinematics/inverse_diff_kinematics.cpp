#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include <cmath>
#include <stdexcept>

namespace kinematics {

using namespace model;

/**
 * Type representing the geometric or analytical jacobian.
 * @note Specific for UR5.
 */
using Jacobian = Matrix<Pose::os_size + Pose::orientation_size, UR5::dof>;

/**
 * Type representing the inverse of the geometric or analytical jacobian.
 */
using InvJacobian = Matrix<Jacobian::ColsAtCompileTime, Jacobian::RowsAtCompileTime>;

/**
 * Type representing a movement like kinematics::Pose as a single vector for small variations.
 * Linear movement + angular movement.
 */
using Movement = Vector<Jacobian::RowsAtCompileTime>;

/**
 * Generate the geometric jacobian for the current configuration of @p robot.
 * @param robot The robot instance.
 * @return The geometric jacobian.
 */
Jacobian generate_jacobian(const UR5& robot) {
  Jacobian result;
  
  // End effector position in order to compute the radius.
  const auto end_effector_position = direct_kinematics(robot).position;

  // The origin of the frame associate to the frame of the iteration. Initially the base frame.
  // Origin recovered as `origin.translation()`.
  auto transformation = JointTransformation::Identity();

  // Column iterator.
  auto jac_column = result.colwise().begin();

  for (auto& joint : robot.joints) {
    // The Z axis of the joint frame in base frame coords.
    const Axis axis = transformation.rotation() * Axis::UnitZ();

    // The origin of the joint frame in the base frame coords.
    const Pose::Position origin = transformation.translation();

    // The rotational radius around `axis`.
    const Pose::Position radius = end_effector_position - origin;

    // The partial derivative of the direct kinematics for this joint (column of the jacobian).
    Movement derivative;
    derivative << axis.cross(radius),
                  axis;

    // Updates the column with the derivative.
    // Passes to the next column for the next iteration.
    *jac_column++ = derivative;

    // pass to the next frame.
    transformation = transformation * joint_transformation_matrix(joint);
  }

  return result;
}

/**
 * Function performing the inverse of a square matrix.
 * @param matrix The matric to invert.
 * @return The inverse.
 * @throw std::domain_error if the @p matrix is not invertible.
 */
template<size_t n>
Matrix<n, n> invert(const Matrix<n, n>& matrix) {
  if (std::abs(matrix.determinant()) < 1e-8) {
    throw std::domain_error("Singular configuration.");
  }

  return matrix.inverse();
}

/**
 * Function performing the right pseudo-inverse of the jacobian.
 * @param jacobian The jacobian to "invert".
 * @return The right pseudo-inverse.
 * @throw std::domain_error If the system is overdetermined or if in singular configuration.
 * @note This function performs the traditional inverse in case of square jacobian.
 */
InvJacobian invert(const Jacobian& matrix) {
  constexpr size_t rows = Jacobian::RowsAtCompileTime;
  constexpr size_t cols = Jacobian::ColsAtCompileTime;

  if constexpr (rows == cols) {
    // Square matrix, normal inversion.
    return invert<rows>(matrix);
  } else if constexpr (rows > cols) {
    // Overdetermined system
    // Solutions could exist, however it is not in the scope of this implementation.
    throw std::domain_error("Not enough degrees of freedom.");
  } else {
    // Right pseudo-inverse.
    return matrix.transpose() * invert<rows>(matrix * matrix.transpose());
  }
}

/**
 * Converts a kinematics::Pose into a compact vector.
 * @param movement The movement to convert.
 * @return A PoseVector representing the same velocity as @p pose as linear velocity and angular velocity,
 * @note Due to representation issues, @p pose should represent a "small" variation.
 */
Movement movement_as_vector(const Pose& movement) {
  // The axis of rotation * sin(dtheta/2).
  Axis axis = movement.orientation.vec();
  
  // The norm  of the axis should be sin(theta/2).
  Scalar norm = axis.norm();

  if (norm < 1e-8) {
    // Linear approximation in order to avoid normalization errors due to small vectors.
    // sin(theta/2) approximable to theta/2.
    // TODO: better to work with already time-cropped movements or velocities?
    axis *= 2;
  } else {
    // Correct inversion when enough precision is available.
    // NOTE: computationally heavier. Worth it?
    axis = 2 * std::asin(norm) * axis.normalized();
  }

  Movement velocity;
  // Concatenation of the position and the vector part of the quaternion (orientation).
  velocity << movement.position,
              axis;

  return velocity;
}

model::UR5::Configuration inverse_diff_kinematics(const model::UR5& robot, const Pose& movement) {
  Jacobian geometric_jacobian = generate_jacobian(robot);

  const InvJacobian inverse = invert(geometric_jacobian);

  const Movement movement_vector = movement_as_vector(movement);

  return inverse * movement_vector;
}


}
