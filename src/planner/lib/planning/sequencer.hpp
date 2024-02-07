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
/// @param current_pose The precalculated current pose.
/// @param dt The simulation time quantization.
/// @param viapoints The set of via points.
/// @param target_pose The final position.
/// @param finish_time The variable where to store the finish time.
/// @return The effective position at the end of the movement.
template<coord::LinearSystem linear_system, coord::AngularSystem angular_system>
#ifdef JOINT_SPACE_PLANNING
TimeFunction<model::UR5::Configuration>
#else
TimeFunction<kinematics::Pose<angular_system, linear_system>>
#endif
via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
);


}

#endif /* PLAN_HPP_INCLUDED */
