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

  Pose() noexcept;
  Pose(Position&& position, Orientation&& orientation) noexcept;
  Pose(const Position& position, const Orientation& orientation) noexcept;

  /**
   * Moves `this` Pose by the given @p movement.
   * @param movement The movement.
   * @return `this` modified pose.
   */
  Pose& move(const Pose& movement);

  /**
   * Returns a pose given by `this` pose moved by @p movement.
   * @param movement The movement.
   * @return The modified pose.
   */
  Pose moved(const Pose& movement) const;

  /**
   * Returns the movement necessary to move from `this` position to the @p desired pose.
   * @param desired The desired pose.
   * @return The movement.
   */
  Pose error(const Pose& desired) const;

  /**
   * Inverts `this` movement.
   */
  Pose inverse() const;

  /**
   * Computes the norm of the pose.
   * Useful to compute the norm of `error`.
   */
  model::Scalar norm() const;


  /**
   * Multiplies this movement by the given coefficient.
   */
  Pose operator *(model::Scalar coefficient) const;

  /**
   * Multiplies this movement by the given coefficient.
   */
  Pose& operator *=(model::Scalar coefficient);

  /**
   * Linear interpolation between two positions.
   * @param other The target pose.
   * @param t The time [0; 1].
   * @return the linear interpolation between these two poses.
   * @note Uses SLERP for orientation.
   */
  Pose interpolate(const Pose& other, model::Scalar t) const;
  

};

/**
 * Evaluates the direct kinematics of @p robot.
 * @param robot The robot configuration.
 * @return The pose of the end effector.
 */
Pose direct_kinematics(const model::UR5& robot) noexcept;

/**
 * Evaluates the direct kinematics of @p robot.
 * @param robot The robot configuration.
 * @param config The target robot configuration.
 * @return The pose of the end effector.
 */
Pose direct_kinematics(const model::UR5& robot, const model::UR5::Configuration& config) noexcept;

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

/**
 * Desired-pose-aware inverse differential kinematics implementation.
 * @param robot The robot configuration.
 * @param velocity The desired velocity in the operational space.
 * @param desired_pose The desired current pose.
 * @return The configuration variation.
 * @throws @p std::domain_error when in singularity.
 * @see kinematics::inverse_diff_kinematics
 */
model::UR5::Configuration dpa_inverse_diff_kinematics(
  const model::UR5& robot,
  const Pose& velocity,
  const Pose& desired_pose
);

}

#endif /* KINEMATICS_HPP_INCLUDED */
