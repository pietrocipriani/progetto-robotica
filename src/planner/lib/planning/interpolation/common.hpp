#ifndef PLANNING_INTERPOLATION_COMMON_HPP_INCLUDED
#define PLANNING_INTERPOLATION_COMMON_HPP_INCLUDED

#include "types.hpp"

namespace planner {

/// Structure representing the position, passage time and acceleration time for a point in the space.
/// Used for the stop&play interpolation.
template<class Point>
struct Params {
  Point point;
  Time time;
  Time accel_delta;
};


}


#endif /* PLANNING_INTERPOLATION_COMMON_HPP_INCLUDED */
