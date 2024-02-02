#include "../interpolation.hpp"
#include "planner.hpp"

namespace planner {


template<class Point>
TimeFunction<Point> linear_interpolation(
  const Point& initial_position, const Point& final_position,
  const Time& start_time, const Time& duration
) {
  auto velocity = (final_position - initial_position) / duration;

  return [=, velocity = std::move(velocity)](const Time& time) {
    return initial_position + velocity * (time - start_time);
  };
}

template decltype(linear_interpolation<os::Position>) linear_interpolation<os::Position>;
template decltype(linear_interpolation<js::Position>) linear_interpolation<js::Position>;


}
