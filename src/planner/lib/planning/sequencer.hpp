#ifndef PLAN_HPP_INCLUDED
#define PLAN_HPP_INCLUDED

#include "planner.hpp"
#include "model.hpp"
#include "types.hpp"
#include <list>

namespace planner {

using ViaPoints = std::list<os::Position>;

/// Creates the config sequence from a set of viapoints.
/// No time constraints are given, the function will try to minimize the total time.
/// @param robot The robot.
/// @param seq The resulting sequence.
/// @param current_pose The precalculated current pose.
/// @param dt The simulation time quantization.
/// @param points The set of via points.
/// @return The effective position at the end of the movement.
TimeFunction<os::Position> via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
);


}

#endif /* PLAN_HPP_INCLUDED */
