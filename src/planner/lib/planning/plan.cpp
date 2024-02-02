#include "kinematics.hpp"
#include "planner.hpp"
#include "constants.hpp"
#include "interpolation.hpp"
#include "sequencer.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <limits>
#include <list>

namespace planner {

/**
 * Gets the nearest safe pose relative to @p pose.
 * @param pose The pose of the end effector.
 * @note Only lifting movements are considered safe when in the low-zone.
 */
os::Position safe_pose(const os::Position& pose) {
  static constexpr Scalar safe_z = table_distance - margin - std::numeric_limits<Scalar>::epsilon();

  auto safe_pose = pose;

  // The robot could already be in a safe position.
  safe_pose.linear().z() = std::min(safe_pose.linear().z(), safe_z);

  return safe_pose;
}

/**
 * Checks the position of the end effector.
 * If it is too low (near the table) movement should be carefully planned.
 * @param current_pose The pose to check.
 * @return `true` if the robot is too low, `false` otherwise.
 */
bool unsafe(const os::Position& pose) {
  // only the z coord is checked. Relative to the robot base frame.
  auto& position = pose.linear().z();

  return position > table_distance - margin;
}

os::Position block_pose_to_pose(const BlockPose::Pose& pose) {
  // TODO: dummy implementation. Could require a transformation between the two frames.
  return os::Position(
    os::Position::Linear(pose.linear().x(), pose.linear().y(), table_distance),
    os::Position::Angular(Rotation(std::arg(pose.angular()), Axis::UnitZ()))
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
MovementSequence plan_movement(model::UR5& robot, const BlockMovement& movement, const Time& dt) {
  const os::Position start_pose = block_pose_to_pose(movement.start.pose);
  const os::Position target_pose = block_pose_to_pose(movement.target.pose);

  // Pose outside of the low-zone just above the `start_pose`.
  const os::Position start_safe_pose = safe_pose(start_pose);
  
  // Pose outside of the low-zone just above the `target_pose`.
  const os::Position target_safe_pose = safe_pose(target_pose);

  // NOTE: only to throw exceptions prematurely in case of unreachability.
  // NOTE: this is actually not accurate as a specific config could not be reachable via differentiation.
  kinematics::inverse(robot, start_safe_pose);
  kinematics::inverse(robot, target_safe_pose);
  kinematics::inverse(robot, start_pose);
  kinematics::inverse(robot, target_pose);

  MovementSequence seq;

  ViaPoints picking_viapt{start_safe_pose, start_pose};
  ViaPoints dropping_viapt{start_safe_pose, target_safe_pose, target_pose};
  
  os::Position current_pose = kinematics::direct(robot);

  if (unsafe(current_pose)) {
    picking_viapt.push_front(safe_pose(current_pose));
  }

  // Picking sequence.
  current_pose = via_point_sequencer(robot, seq.picking, current_pose, dt, std::move(picking_viapt));

  // Dropping sequence.
  via_point_sequencer(robot, seq.dropping, current_pose, dt, std::move(dropping_viapt));
  
  return seq;
}


}
