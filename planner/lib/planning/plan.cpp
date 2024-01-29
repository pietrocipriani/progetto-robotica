#include "planner.hpp"
#include "constants.hpp"
#include "interpolation.hpp"
#include "plan.hpp"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <functional>
#include <limits>

namespace planner {

using namespace model;


/**
 * Gets the nearest safe pose relative to @p pose.
 * @param pose The pose of the end effector.
 * @note Only lifting movements are considered safe when in the low-zone.
 */
os::Pose safe_pose(const os::Pose& pose) {
  static constexpr Scalar safe_z = table_distance - margin - std::numeric_limits<Scalar>::epsilon();

  auto safe_pose = pose;

  // The robot could already be in a safe position.
  safe_pose.position.z() = std::min(safe_pose.position.z(), safe_z);

  return safe_pose;
}

os::Pose block_pose_to_pose(const BlockPose& pose) {
  // TODO: dummy implementation
  return os::Pose(
    os::Position(pose.position.x(), pose.position.y(), table_distance),
    os::Pose::Orientation(Eigen::AngleAxis<Scalar>(pose.orientation, os::Pose::Orientation::Vector3::UnitZ()))
  );
}


/**
 * Generates the sequence of movements from the initial @p robot configuration, throught the picking
 * of one block from its starting position, to the relase of the block into its target position.
 * @param robot The current robot configuration.
 * @param movement The requested block movement.
 * @param dt The time granularity.
 * @return A sequence of configuration variations in order to perform the given movement.
 * @throw std::domain_error If one of the positions is not in the operational space.
 * @note Evaluate if its worth to run this into a thread in order to perform the next calculations
 *  during the physical driving of the robot for the previous movement.
 */
MovementSequence plan_movement(UR5& robot, const BlockMovement& movement, const Scalar dt) {
  const os::Pose start_pose = block_pose_to_pose(movement.start);
  const os::Pose target_pose = block_pose_to_pose(movement.target);

  // Pose outside of the low-zone just above the `start_pose`.
  const os::Pose start_safe_pose = safe_pose(start_pose);
  
  // Pose outside of the low-zone just above the `target_pose`.
  const os::Pose target_safe_pose = safe_pose(target_pose);

  // NOTE: only to throw exceptions prematurely in case of unreachability.
  kinematics::inverse(robot, start_safe_pose);
  kinematics::inverse(robot, target_safe_pose);
  kinematics::inverse(robot, start_pose);
  kinematics::inverse(robot, target_pose);

  MovementSequence seq;

  // Lifting sequence.
  os::Velocity velocity = lift_movement(robot, seq.picking, dt);
  velocity = trasversal_movement(robot, seq.picking, dt, velocity);
  descend_movement(robot, seq.picking, dt, velocity);

  // Dropping sequence.
  velocity = lift_movement(robot, seq.dropping, dt);
  velocity = trasversal_movement(robot, seq.dropping, dt, velocity);
  descend_movement(robot, seq.picking, dt, velocity);
  
  return seq;
}


}
