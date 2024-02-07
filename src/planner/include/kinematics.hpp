#ifndef KINEMATICS_HPP_INCLUDED
#define KINEMATICS_HPP_INCLUDED

#include "euler.hpp"
#include "model.hpp"
#include "utils.hpp"
#include "constants.hpp"
#include <type_traits>
#include "spaces.hpp"
#include "utils/coordinates.hpp"

namespace kinematics {
  

/// The pose of the end effector (position + orientation).
/// Position in cartesian coordinates.
/// Orientation as quaternions.
/// @note About the usage of quaternions:
/// - Euler angles have representational singularities and need the analytical jacobian.
/// - Angular velocities are much more intuitive that euler angles variations.
///   - Interpolation of euler angles can lead to "unexpected" movements.
///   - Quaternions consent numeric integration of the result (a problem of angular velocities).
/// However:
/// - Quaternions cannot be analytically integrated unless the rotation axis remains constant. // TODO: verify statement.
///   - Actually the project require rotations only around the Z axis. (loss of generality).
///   - Also in case of multiple rotations it is possible to segment the movement: cannot grant continuity in orientation variation.
/// - The project require rotations only around the Z axis (this is not a good point, loss of generality).
/// - The planner has full control on the orientation interpolation
template<
  coord::AngularSystem angular_system = coord::Lie,
  coord::LinearSystem linear_system = coord::Cartesian
> using Pose = OperationalSpace<linear_system, angular_system, os_size>;

template<
  coord::AngularSystem angular_system = coord::Lie,
  coord::LinearSystem linear_system = coord::Cartesian
> using Velocity = typename Pose<angular_system, linear_system>::Derivative;

template<
  coord::AngularSystem angular_system = coord::Lie,
  coord::LinearSystem linear_system = coord::Cartesian
> using Acceleration = typename Velocity<angular_system, linear_system>::Derivative;


/// Evaluates the direct kinematics of @p robot.
/// @param robot The robot configuration.
/// @return The pose of the end effector.
template<
  class Robot, coord::AngularSystem angular_system = coord::Lie,
  coord::LinearSystem linear_system = coord::Cartesian
> Pose<angular_system, linear_system> direct(const Robot& robot) noexcept;


/// Evaluates the direct kinematics of @p robot.
/// @param robot The robot configuration.
/// @param config The target robot configuration.
/// @return The pose of the end effector.
template<
  class Robot, coord::AngularSystem angular_system = coord::Lie,
  coord::LinearSystem linear_system = coord::Cartesian
> Pose<angular_system, linear_system> direct(const Robot& robot, const typename Robot::Configuration& config) noexcept;


/// Performs the inverse kinematics for @p robot.
/// @param robot The robot parameters.
/// @param pose The desired pose.
/// @return The "nearest" configuration to the current configuration to obtain the given @p pose.
/// @throws @p std::domain_error if the desired @p pose in not in the operational space.
/// @note UR5 is not redundant, however multiple (finite) solutions are allowed.
model::UR5::Configuration inverse(
  const model::UR5& robot,
  const Pose<coord::Lie, coord::Cartesian>& pose
);

/// Finds the configuration variation in order to obtain the desired velocity in the operational space.
/// @param robot The robot configuration.
/// @param velocity The desired velocity in the operational space.
/// @return The configuration variation.
/// @throws @p std::domain_error when in singularity.
/// @note Damped least-squares. This choice could be subject to change.
template<class Robot>
typename Robot::Velocity inverse_diff(
  const Robot& robot,
  const Velocity<coord::Lie, coord::Cartesian>& movement
);

/// Desired-pose-aware inverse differential kinematics implementation.
/// @param robot The robot configuration.
/// @param velocity The desired velocity in the operational space.
/// @param desired_pose The desired current pose.
/// @return The configuration variation.
/// @throws @p std::domain_error when in singularity.
/// @see kinematics::inverse_diff
/// @note Usage of cartesian coordinates and lie groups is forced to avoid wrap-around issues
///       in the error-calculation phase.
template<class Robot>
typename Robot::Velocity dpa_inverse_diff(
  const Robot& robot,
  const Velocity<coord::Lie, coord::Cartesian>& movement,
  const Pose<coord::Lie, coord::Cartesian>& desired_pose,
  const Time& dt
);

}

#endif /* KINEMATICS_HPP_INCLUDED */
