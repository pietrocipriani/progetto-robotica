#ifndef PLANNING_INTERNAL_HPP_INCLUDED
#define PLANNING_INTERNAL_HPP_INCLUDED

#include "constants.hpp"
#include "planner.hpp"



namespace planner {


/// Gets the nearest safe pose relative to @p pose.
/// @param pose The pose of the end effector.
/// @note Only lifting movements are considered safe when in the low-zone.
os::Position safe_pose(const os::Position& pose);


/// Checks the position of the end effector.
/// If it is too low (near the table) movement should be carefully planned.
/// @param current_pose The pose to check.
/// @return `true` if the robot is too low, `false` otherwise.
bool unsafe(const os::Position& pose);


os::Position block_pose_to_pose(const BlockPose::Pose& pose);


}

#endif /* PLANNING_INTERNAL_HPP_INCLUDED */
