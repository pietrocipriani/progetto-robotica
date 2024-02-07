#ifndef PARABOLIC_HPP_INCLUDED
#define PARABOLIC_HPP_INCLUDED


#include "constants.hpp"
#include "linear.hpp"
#include "quadratic.hpp"
#include "planner.hpp"
#include "stop_and_play.hpp"
#include "common.hpp"
#include <algorithm>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <vector>

namespace planner {

/// Possible modes of points:
/// - Point: an extremal point.
/// - ViaPoint: an (unreached) via point.
enum class Mode { Point, ViaPoint };

namespace internal {

struct QuadraticTimestamps {
  Time start, end, start_linear, end_linear, delta;

private:
  template<Mode mode>
  static constexpr Scalar factor = mode == Mode::Point ? 1.0 : 0.5;

public:

  template<Mode start_mode, Mode end_mode, class Point>
  static QuadraticTimestamps build(const Params<Point>& start, const Params<Point>& end) {
    return {
      start.time, end.time,
      start.time + start.accel_delta * factor<start_mode>,
      end.time - end.accel_delta * factor<end_mode>,
      end.time - start.time - start.accel_delta * (factor<start_mode> - 0.5) - end.accel_delta * (factor<end_mode> - 0.5)
    };
  }
  
};

/// Performs the parabolic interpolation between three points.
/// @param start_mode The type of the starting point.
/// @param end_mode The type of the ending point.
/// @param start The starting point.
/// @param via The via point.
/// @param end The ending point.
/// @param chain The Container where to store the chain of functions.
template<Mode start_mode, Mode end_mode, class Point>
void interpolation3(
  const Params<Point>& start,
  const Params<Point>& via,
  const Params<Point>& end,
  typename SequencedFunction<TimeFunction<Point>, Time>::Container& chain
) {
  using namespace uniformed_rotation_algebra;

  // The chain must start with a Mode::Point.
  assert(start_mode == Mode::Point || !chain.empty());

  // Timestamps.
  auto t12 = QuadraticTimestamps::build<start_mode, Mode::ViaPoint>(start, via);
  auto t23 = QuadraticTimestamps::build<Mode::ViaPoint, end_mode>(via, end);
  
  // Velocities.
  auto v12 = unlazy((via.point - start.point) / t12.delta);
  auto v23 = unlazy((end.point - via.point) / t23.delta);

  if constexpr (start_mode == Mode::Point) {
    auto q = quadratic_acceleration(start.point, v12, t12.start, start.accel_delta);
    auto l = linear_interpolation(start.point, via.point, t12.start + start.accel_delta / 2, t12.delta);

    assert((start.point - q(t12.start)).norm() < dummy_precision);
    assert((l(t12.start_linear) - q(t12.start_linear)).norm() < dummy_precision);

    chain.emplace_back(std::move(q), t12.start);
    chain.emplace_back(std::move(l), t12.start_linear);
  }

  auto q = quadratic_interpolation(chain.back()(t12.end_linear), v12, v23, t12.end_linear, via.accel_delta);
  auto l = linear_interpolation(via.point, end.point, t23.start, t23.delta);

  assert((q(t12.end_linear) - chain.back()(t12.end_linear)).norm() < dummy_precision);
  assert((q(t23.start_linear) - l(t23.start_linear)).norm() < dummy_precision);

  chain.emplace_back(std::move(q), t12.end_linear);
  chain.emplace_back(std::move(l), t23.start_linear);

  if constexpr (end_mode == Mode::Point) {
    auto q = quadratic_deceleration(end.point, v23, t23.end, end.accel_delta);
  
    assert((q(t23.end_linear) - l(t23.end_linear)).norm() < dummy_precision);
    assert((q(t23.end) - end.point).norm() < dummy_precision);

    chain.emplace_back(q, t23.end_linear);
  }
}


}


/// Creates a parabolic interpolation between the @p points.
/// @param start The starting point.
/// @param via_points The intermediate via points.
/// @param end The ending point.
/// @return The interpolating function.
/// @note Degenerates in a stop&play interpolation in case in case of only 2 points.
template<class Point, template<class T> class Container = std::initializer_list>
auto parabolic_interpolation(
  const Params<Point>& start,
  const Container<Params<Point>>& via_points,
  const Params<Point>& end
) {
  using namespace internal;
  using Sequence = SequencedFunction<TimeFunction<Point>, Time>;

  if (via_points.size() == 0) {
    return stop_and_play_interpolation(start, {end});
  }

  // The resulting function is a chain of interpolations.
  typename Sequence::Container chain;

  // - quadratic acceleration from the starting node.
  // - linear movement after the starting node.
  // - quadratic deceleration to the final node.
  // - For each via point:
  //   - quadratic acceleration.
  //   - linear movement.
  chain.reserve(3 + 2 * via_points.size());

  if (via_points.size() == 1) {
    interpolation3<Mode::Point, Mode::Point>(start, *via_points.begin(), end, chain);
  } else {
    auto i1 = via_points.begin();
    // This is a valid iterator as via_points.size() >= 2.
    auto i2 = std::next(i1);

    interpolation3<Mode::Point, Mode::ViaPoint>(start, *i1, *i2, chain);
    for (auto i3 = std::next(i2); i3 != via_points.end(); i1 = i2, i2 = i3, i3 = std::next(i2)) {
      interpolation3<Mode::ViaPoint, Mode::ViaPoint>(*i1, *i2, *i3, chain);
    }
    interpolation3<Mode::ViaPoint, Mode::Point>(*i1, *i2, end, chain);
  }

  return Sequence(std::move(chain));
}


}



#endif /* PARABOLIC_HPP_INCLUDED */
