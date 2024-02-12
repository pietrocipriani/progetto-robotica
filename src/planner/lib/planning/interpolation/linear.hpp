#ifndef LINEAR_HPP_INCLUDED
#define LINEAR_HPP_INCLUDED


#include "constants.hpp"
#include "planner.hpp"

namespace planner {


/// Linearly interpolates between @p initial_position and @p final_position int the given @p duration.
/// @param initial_position The initial position.
/// @param final_position The final position.
/// @param start_time The time at which the position is @p initial_position.
/// @param duration The time to move from @p initial_position to @p final_position.
/// @return An interpolating function of time between the two positions.
template<class Point>
TimeFunction<Point> linear_interpolation(
  const Point& initial_position, const Point& final_position,
  const Time& start_time, const Time& duration
) {
  using namespace uniformed_rotation_algebra;

  // To avoid division by zero.
  if (duration < dummy_precision) {
    return [=]([[maybe_unused]] const Time& time) { return final_position; };
  }

  // FIXME: with quaternions there is the possibility that wrong velocities are produced.
  //        The problem is that quaternions are not a good choice for velocities due to the
  //         periodicity of the components.
  auto velocity = unlazy((final_position - initial_position) / duration);

  return [=, velocity = std::move(velocity)](const Time& time) {
    return initial_position + velocity * (time - start_time);
  };
}


}


#endif /* LINEAR_HPP_INCLUDED */
