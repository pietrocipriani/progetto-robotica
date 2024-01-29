#ifndef PLAN_HPP_INCLUDED
#define PLAN_HPP_INCLUDED

#include "planner.hpp"
#include "model.hpp"

namespace planner {

/**
 * Plans the lifting movement in order to exit the low-zone.
 * @param robot The robot.
 * @param seq The container where to store the generated sequence.
 * @param dt The convertion factor from [sim] to [s].
 * @return The velocity in the joint space at the end of the movement.
 * @note The inital velocity of the end effector is assumed to be null.
 */
os::Velocity lift_movement(model::UR5& robot, MovementSequence::ConfigSequence& seq, model::Scalar dt);

/**
 * Plans the descend movement in order to release (or pick) a block.
 * @param robot The robot.
 * @param seq The container where to store the generated sequence.
 * @param dt The convertion factor from [sim] to [s].
 * @param initial_velocity The initial velocity to assert continuity.
 * @note The velocity in the joint space at the end of the movement is null.
 * @note This planner is allowed to force acceleration to avoid smashing.
 *        However this should not be the case with accurate higher level planning.
 */
void descend_movement(
  model::UR5& robot,
  MovementSequence::ConfigSequence& seq,
  model::Scalar dt,
  const js::Velocity& initial_velocity
);

/*
 * Plans the traversal movement (between two safe positions).
 * @param robot The robot.
 * @param seq The container where to store the generated sequence.
 * @param dt The convertion factor from [sim] to [s].
 * @param final_position The position where to move.
 * @param initial_velocity The initial_velocity to assert continuity.
 * @return The velocity in the joint space at the end of the movement.
 * @note The final velocity is calibrated to allow safe stopping during the descend phase.
 */
os::Velocity trasversal_movement(
  model::UR5& robot,
  MovementSequence::ConfigSequence& seq,
  model::Scalar dt,
  const js::Position& final_position,
  const js::Velocity& initial_velocity
);




}

#endif /* PLAN_HPP_INCLUDED */
