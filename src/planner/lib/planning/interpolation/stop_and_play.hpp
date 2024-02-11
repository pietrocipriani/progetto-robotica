#ifndef STOP_AND_PLAY_HPP_INCLUDED
#define STOP_AND_PLAY_HPP_INCLUDED


#include "linear.hpp"
#include "quadratic.hpp"
#include "planner.hpp"
#include "utils.hpp"
#include "common.hpp"
#include <algorithm>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <vector>

namespace planner {

namespace internal {

struct Timestamps {
  Time start, end, start_linear, end_linear, delta;

  template<class Point>
  Timestamps(const Params<Point>& start, const Params<Point>& end) 
    : start(start.times.time), end(end.times.time)
    , start_linear(start.times.time + start.times.accel_delta)
    , end_linear(end.times.time - end.times.accel_delta)
    , delta(end.times.time - start.times.time - (start.times.accel_delta + end.times.accel_delta) / 2) {}
  
};

/// Performs the stop&play interpolation between two points.
/// @param start The starting point.
/// @param end The ending point.
/// @param chain The Container where to store the chain of functions.
/// @note This interpolation produces a chain of 3 functions (acc + linear + dec).
template<class Point>
void interpolation2(
  const Params<Point>& start,
  const Params<Point>& end,
  typename SequencedFunction<TimeFunction<Point>, Time>::Container& chain
) {
  using namespace internal;
  using namespace uniformed_rotation_algebra;

  // Timestamp.
  Timestamps t(start, end);
  
   // Velocity.
  auto v = unlazy((end.point - start.point) / t.delta);

  auto q1 = quadratic_acceleration(start.point, v, t.start, start.times.accel_delta);
  chain.emplace_back(std::move(q1), t.start);

  if (t.start_linear < t.end_linear) {
    auto l = linear_interpolation(start.point, end.point, t.start + start.times.accel_delta / 2, t.delta);
    chain.emplace_back(std::move(l), t.start_linear);
  }

  auto q2 = quadratic_deceleration(end.point, v, t.end, end.times.accel_delta);
  chain.emplace_back(std::move(q2), t.end_linear);
}

}

/// Creates a stop&play interpolation between the @p points.
/// @param start The starting point.
/// @param points The intermediate points + ending.
/// @return The interpolating function.
/// @note @p start as special point to avoid empty lists.
///       This can be non-optimal for certain use cases.
/// @note Degenerates in a constant function in case of only one parameter.
template<class Point, template<class T> class Container = std::initializer_list>
auto stop_and_play_interpolation(
  const Params<Point>& start,
  const Container<Params<Point>>& points
) {
  using namespace internal;
  using Sequence = SequencedFunction<TimeFunction<Point>, Time>;

  // The resulting function is a chain of interpolations.
  typename Sequence::Container chain;

  // points.size() - 1 segments.
  // For each segment 3 functions are needed (acc + lin + dec).
  chain.reserve(3 * (points.size() - 1));


  if (points.size() == 0) {
    // Constant function in case of only one point.
    chain.emplace_back([p = start.point]([[maybe_unused]] const Time& _) {return p;}, start.times.time);
  } else {
    // This is a valid iterator as points.size() >= 1.
    auto i1 = points.begin();

    interpolation2(start, *i1, chain);
    for (; std::next(i1) != points.end(); i1 = std::next(i1)) {
      interpolation2(*i1, *std::next(i1), chain);
    }
  }

  auto s = Sequence(std::move(chain));

  return s;
}


}



#endif /* STOP_AND_PLAY_HPP_INCLUDED */
