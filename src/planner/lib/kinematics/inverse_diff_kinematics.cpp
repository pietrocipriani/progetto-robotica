#include "kinematics.hpp"
#include "constants.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"
#include "utils.hpp"
#include "euler.hpp"
#include "utils/coordinates.hpp"
#include "utils/unlazy.hpp"
#include <cassert>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace kinematics {

using namespace model;

using DefaultPose = Pose<coord::Lie, coord::Cartesian>;

/// Determinant of J·J^T to be considered near to a singularity.
///
// TODO: calibrate threshold.
constexpr Scalar min_safe_determinant = 1e-11;

/// Type representing the geometric or analytical jacobian.
/// @note Specific for UR5.
template<class Robot>
using Jacobian = Matrix<os_size + so<os_size>, Robot::dof>;

/// Type representing the inverse of the geometric or analytical jacobian.
///
template<class Robot>
using InvJacobian = Matrix<Jacobian<Robot>::ColsAtCompileTime, Jacobian<Robot>::RowsAtCompileTime>;


/// Function performing the inverse of a square matrix with (optional: default) determinant check.
/// @param safe If a check on the determinant has to be performed.
/// @param matrix The matric to invert.
/// @return The inverse.
/// @throw std::domain_error if the @p matrix is not invertible.
template<size_t n, bool safe = false>
Matrix<n, n> sq_invert(const Matrix<n, n>& matrix) {
  if constexpr (safe) {
    // NOTE: NDEBUG dependent check.
    assert(std::abs(matrix.determinant()) >= min_safe_determinant);
  } else if (std::abs(matrix.determinant()) < min_safe_determinant) {
    std::stringstream err;
    err << "Singular configuration. Det = " << matrix.determinant();
    throw std::domain_error(err.str());
  }

  return matrix.inverse();
}

/// Function performing the right pseudo-inverse of the jacobian.
/// @param jacobian The jacobian to "invert".
/// @return The right pseudo-inverse.
/// @throw std::domain_error If the system is overdetermined or if in singular configuration.
/// @note This function performs the traditional inverse in case of square jacobian.
template<class Robot, bool damped = false>
InvJacobian<Robot> invert(const Jacobian<Robot>& matrix) {
  constexpr size_t rows = Jacobian<Robot>::RowsAtCompileTime;
  constexpr size_t cols = Jacobian<Robot>::ColsAtCompileTime;

  if constexpr (damped) {
    // DLS inverse.
    
    const auto transpose = unlazy(matrix.transpose());

    // matrix * matrix^T
    const auto mmt = matrix * transpose;

    // TODO: calibrate damping factor.
    // With positive semidefinite matrixes this relation stands:
    // det(A + B) >= det(A) + det(B)
    // With real coefficients it is easily proovable that matrix_matrixt is a positive semidefinite matrix.
    // The identity matrix is positive definite.
    //
    // We want to allow the minimum damping factor to avoid the singularity threshold.
    // The above relation translates into:
    // det(matrix_matrixt + k²·I) >= det(mmt) + k^(2·rows) >= min_safe_determinant.
    // This means that
    // k^(2·rows) >= min_safe_determinant - det(mmt).
    // k >= pow( max(0, min_safe_determinant - det(mmt)) , 1 / 2·rows ).

    // As above, the min determinant of the damping matrix.
    // dummy precision to avoid subthreshold triggering just for approximations.
    Scalar min_damping_det = min_safe_determinant - mmt.determinant() + dummy_precision;
    // If the determinant can be negative, no damping factor is needed.
    min_damping_det = std::max(0.0, min_damping_det);

    // The det is k^(2·rows), extracting k².
    const Scalar damping_factor = std::pow(min_damping_det, 1.0 / rows);

    // The damping matrix
    auto damping = damping_factor * Matrix<rows>::Identity();

    auto matrix_to_invert = mmt + damping;
    return transpose * sq_invert<rows, true>(matrix_to_invert);
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

/// Generate the inverse analytical jacobian for the current configuration of @p robot.
/// @param robot The robot instance.
/// @param current_pose The precalculated current_pose
/// @return The analytical jacobian.
template<class Robot>
InvJacobian<Robot> inverse_geometrical_jacobian(const Robot& robot, const DefaultPose& current_pose) {
  Jacobian<Robot> geometric;

  // End effector position in order to compute the radius.
  const auto end_effector_position = current_pose.linear();

  // The origin of the frame associate to the frame of the iteration. Initially the base frame.
  // Origin recovered as `origin.translation()`.
  auto transformation = JointTransformation::Identity();

  // Used to iterate through columns.
  int column_index = 0;

  for (const auto& joint : robot.joints) {
    // The Z axis of the joint frame in base frame coords.
    const Axis axis = transformation.rotation() * Axis::UnitZ();

    // The origin of the joint frame in the base frame coords.
    const auto origin = transformation.translation();

    // The rotational radius around `axis`.
    const auto radius = end_effector_position - origin;

    // The partial derivative of the direct kinematics for this joint (column of the jacobian).
    DefaultPose::Derivative::Base column;
    column << axis.cross(radius) , axis;

    // Updates the column with the derivative.
    // Passes to the next column for the next iteration.
    geometric.col(column_index++) = std::move(column);

    // pass to the next frame.
    transformation = transformation * joint_transformation_matrix(joint);
  }

  return invert<Robot>(geometric);
}

template<class Robot>
typename Robot::Velocity inverse_diff(
  const Robot& robot,
  const DefaultPose::Derivative& movement,
  const DefaultPose& current_pose
) {
  const auto inverse = inverse_geometrical_jacobian(robot, current_pose);

  return typename Robot::Velocity(inverse * movement.vector());
}

template<class Robot>
typename Robot::Velocity inverse_diff(const Robot& robot, const DefaultPose::Derivative& movement) {
  return inverse_diff(robot, movement, direct(robot));
}

template<class Robot>
typename Robot::Velocity dpa_inverse_diff(
  const Robot& robot,
  const DefaultPose::Derivative& movement,
  const DefaultPose& desired_pose,
  const Time& dt
) {
  // Direct kinematics for the effective position (error computation).
  const auto effective_position = direct<Robot, coord::Lie, coord::Cartesian>(robot);

  // Compute the error, movement is required.
  // FIXME: with quaternions there is the possibility that `/dt` produces an "identity error".
  //        This implementation is actually buggy.
  //        The "official" implementation should be used.
  //        However this implementation is easily readable.
  //        A solution requires the caller to perform the crop, passing @p dt = 1.
  auto error = (desired_pose - effective_position) / dt;

  // Compose the error with the desired movement.
  error += movement;

  return inverse_diff(robot, error, effective_position);
}


template decltype(dpa_inverse_diff<model::UR5>) dpa_inverse_diff<model::UR5>;
template model::UR5::Velocity inverse_diff<model::UR5>(const model::UR5&, const DefaultPose::Derivative&);

}
