#ifndef INTERPOLATION_HPP_INCLUDED
#define INTERPOLATION_HPP_INCLUDED

#include "planner.hpp"
#include "model.hpp"
#include "interpolation/quadratic.hpp"
#include "interpolation/linear.hpp"
#include "interpolation/parabolic.hpp"
#include "interpolation/stop_and_play.hpp"

namespace planner {



/// Operational space uniformly accelerated motion with speed saturation.
/// @param initial_position The position at time 0 s of the end effector.
/// @param initial_velocity The velocity at time 0 s of the end effector.
/// @param final_velocity The saturation velocity.
/// @param acceleration The magnitude of the acceleration for velocity change.
/// @return The interpolating function of time that returns the position in a given instant.
/// @note The length is measured in [m].
///       The time is measured in [sim].
/// @note This function is applied in the lifting phase of the movement, no rotation needed.
///       Rotation addition is difficult due to the choice of using quaternions.
///       Alternatively the same axis of rotation must be imposed.
///       Otherwise numeric integration can be performed.
[[deprecated("Migrate to quadratic + linear interpolation.")]]
TimeFunction<os::Position> os_uam_interpolation(
  const os::Position& initial_position,
  const os::Velocity& initial_velocity,
  const os::Velocity& final_velocity,
  Scalar acceleration
);


}

#endif /* INTERPOLATION_HPP_INCLUDED */
