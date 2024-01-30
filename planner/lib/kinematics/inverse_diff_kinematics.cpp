#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"
#include "utils.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace kinematics {

using namespace model;

/**
 * Type representing the geometric or analytical jacobian.
 * @note Specific for UR5.
 */
using Jacobian = Matrix<os_size + orientation_size, UR5::dof>;

/**
 * Type representing the inverse of the geometric or analytical jacobian.
 */
using InvJacobian = Matrix<Jacobian::ColsAtCompileTime, Jacobian::RowsAtCompileTime>;


/**
 * Function performing the inverse of a square matrix.
 * @param matrix The matric to invert.
 * @return The inverse.
 * @throw std::domain_error if the @p matrix is not invertible.
 */
template<size_t n>
Matrix<n, n> sq_invert(const Matrix<n, n>& matrix) {
  // TODO: calibrate threshold
  if (std::abs(matrix.determinant()) < 1e-5) {
    std::stringstream err; 
    err << "Singular configuration. Det = " << matrix.determinant();
    throw std::domain_error(err.str());
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
template<bool damped = true>
InvJacobian invert(const Jacobian& matrix) {
  constexpr size_t rows = Jacobian::RowsAtCompileTime;
  constexpr size_t cols = Jacobian::ColsAtCompileTime;

  if constexpr (damped) {
    // DLS inverse.

    const auto matrix_matrixt = matrix * matrix.transpose();

    // TODO: choose coefficient
    // constexpr Scalar damping_factor = 1e-2;

    // Error too high in certain cases.
    const Scalar damping_factor = sigmoid(1 / matrix_matrixt.determinant()) * 1;

    // The damping matrix
    auto damping = std::pow(damping_factor, 2) * Matrix<rows, rows>::Identity();



    auto matrix_to_invert = matrix_matrixt + damping;
    return matrix.transpose() * sq_invert<rows>(matrix_to_invert);
  } else if constexpr (rows == cols) {
    // Square matrix, normal inversion.
    return sq_invert<rows>(matrix);
  } else if constexpr (rows > cols) {
    // Overdetermined system
    // Solutions could exist, however it is not in the scope of this implementation.
    throw std::domain_error("Not enough degrees of freedom.");
  } else {
    // Right pseudo-inverse.
    return matrix.transpose() * sq_invert<rows>(matrix * matrix.transpose());
  }
}

// TODO: implement
model::Matrix<orientation_size, orientation_size> euler_angles_jacobian(const Pose_2::OrientationReference& orientation);

/**
 * Generate the inverse analytical jacobian for the current configuration of @p robot.
 * @param robot The robot instance.
 * @return The analytical jacobian.
 */
InvJacobian generate_inverse_jacobian(const UR5& robot) {
  Jacobian geometric;
  
  // TODO FIXME: recalculation with dpa.
  const auto current_pose = direct(robot);

  // End effector position in order to compute the radius.
  const auto end_effector_position = current_pose.position;

  // The origin of the frame associate to the frame of the iteration. Initially the base frame.
  // Origin recovered as `origin.translation()`.
  auto transformation = JointTransformation::Identity();

  // Column iterator.
  auto jac_column = geometric.colwise().begin();

  for (auto& joint : robot.joints) {
    // The Z axis of the joint frame in base frame coords.
    const Axis axis = transformation.rotation() * Axis::UnitZ();

    // The origin of the joint frame in the base frame coords.
    const Pose_2::Position origin = transformation.translation();

    // The rotational radius around `axis`.
    const Pose_2::Position radius = end_effector_position - origin;

    // The partial derivative of the direct kinematics for this joint (column of the jacobian).
    Movement::Container derivative;
    derivative << axis.cross(radius),
                  axis;

    // Updates the column with the derivative.
    // Passes to the next column for the next iteration.
    *jac_column++ = derivative;

    // pass to the next frame.
    transformation = transformation * joint_transformation_matrix(joint);
  }

  auto geom_inverse = invert(geometric);

  auto euler_to_omega = euler_angles_jacobian(current_pose.orientation);

  return geom_inverse * euler_to_omega;
}


model::UR5::Configuration inverse_diff(const model::UR5 &robot, const Movement::Container& movement) {
  // Geometric jacobian as we are working with quaternions.
  const auto inverse = generate_inverse_jacobian(robot);

  return inverse * movement;
}

model::UR5::Configuration inverse_diff(const model::UR5& robot, const Movement& movement) {
  return inverse_diff(robot, movement.vector);
}


model::UR5::Configuration dpa_inverse_diff(
  const model::UR5& robot,
  const Movement& movement,
  const Pose_2& desired_pose
) {
  // Direct kinematics for the effective position (error computation).
  const Pose_2 effective_position = direct(robot);

  // Compute the error, movement is required.
  // TODO: introduce a weight coefficient.
  // TODO: the error has to be limited in magnitude: cannot imply enormous movements when high.
  Movement error = desired_pose - effective_position;
  
  // TODO: error weight.
  error *= 1;

  /*Movement desired_movement = movement_as_vector(movement);

  Movement error_movement = movement_as_vector(error);

  Movement movement_vector = desired_movement + error_movement;

  return inverse_diff_kinematics(robot, movement_vector);*/


  // Compose the error with the desired movement.
  // TODO: prove the error converges to 0.
  error += movement;

  return inverse_diff(robot, error);
}


}
