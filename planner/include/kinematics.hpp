#ifndef KINEMATICS_HPP_INCLUDED
#define KINEMATICS_HPP_INCLUDED

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include "model.hpp"

namespace kinematics {


/**
 * Type representing the pose of the end effector.
 */
struct Pose {
  /**
   * The size of the operational space.
   */
  static constexpr size_t os_size = 3;

  /**
   * The size of the orientation space.
   */
  static constexpr size_t orientation_size = os_size * (os_size - 1) / 2;

  using Position = model::Vector<os_size>;

  // Quaternion to avoid representation singularities.
  using Orientation = model::Quaternion;

  Position position;
  Orientation orientation;

  Pose(Position&& position, Orientation&& orientation);
  Pose(const Position& position, const Orientation& orientation);
};

/**
 * Evaluates the direct kinematics of @p robot.
 * @param robot The robot configuration.
 * @return The pose of the end effector.
 */
Pose direct_kinematics(const model::UR5& robot) noexcept;

/**
 * Performs the inverse kinematics for @p robot.
 * @param robot The robot parameters.
 * @param pose The desired pose.
 * @return The "nearest" configuration to the current configuration to obtain the given @p pose.
 * @throws @p std::domain_error if the desired @p pose in not in the operational space.
 * @note UR5 is not redundant, however multiple (finite) solutions are allowed.
 */
model::UR5::Configuration inverse_kinematics(const model::UR5& robot, const Pose& pose);

/**
 * Finds the configuration variation in order to obtain the desired velocity in the operational space.
 * @param robot The robot configuration.
 * @param velocity The desired velocity in the operational space.
 * @return The configuration variation.
 * @throws @p std::domain_error when in singularity.
 * @note No damped least-squares. A correct planning is assumed. This choice could be subject to change.
 */
model::UR5::Configuration inverse_diff_kinematics(const model::UR5& robot, const Pose& velocity);

}

#endif /* KINEMATICS_HPP_INCLUDED */
