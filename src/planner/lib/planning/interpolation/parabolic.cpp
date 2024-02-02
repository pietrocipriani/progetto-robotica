#include "../interpolation.hpp"
#include "planner.hpp"
#include <algorithm>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <vector>

namespace planner {

using namespace model;

template<class Point>
struct ParabolicParams {
  Point point;
  Time time;
  Time accel_delta;
};

enum class Mode { Point, ViaPoint };

template<class Function, class Domain>
struct RestrictedFunction {
  Function f;
  Domain begin;

  template<class F>
  bool operator<(const RestrictedFunction<F, Domain>& other) const {
    return begin < other.begin;
  }
  template<class F>
  bool operator<(const Domain& arg) const {
    return begin < arg;
  }

  template<class... Args>
  typename std::invoke_result_t<Function, Args...> operator()(Args&& ...args) const {
    return std::invoke(f, std::forward(args)...);
  }
};


template<class Function, class Domain>
class SequencedFunction {
public:
  using Container = std::vector<RestrictedFunction<Function, Domain>>;
private:
  using It = typename Container::iterator;
  Container functions;
  mutable It it = functions.begin();

  bool can_use(It it, const Domain& arg) const {
    if (it == functions.end()) return false;
    auto next = std::next(it);
    return it->begin <= arg && (next == functions.end() || next->begin >= arg);
  }
  
public:
  SequencedFunction(Container&& fs) : functions(std::move(fs)) {
    assert(!fs.empty() && std::is_sorted(functions.begin(), functions.end()));
  }

  typename std::invoke_result_t<Function, Domain> operator()(const Domain& arg) const {
    if (can_use(it)) return it(arg);
    if (can_use(std::next(it))) return (++it)(arg);
    it = std::lower_bound(functions.begin(), functions.end(), arg);
    if (it == functions.end()) it = functions.begin();
    return it(arg);
  }
};



struct QuadraticTimestamps {
  Time start, end, start_linear, end_linear, delta;

private:
  template<Mode mode>
  static constexpr Scalar factor = mode == Mode::Point ? 1.0 : 0.5;

public:

  template<Mode start_mode, Mode end_mode, class Point>
  static QuadraticTimestamps build(const ParabolicParams<Point>& start, const ParabolicParams<Point>& end) {
    return {
      start.time, end.time,
      start.time + start.accel_delta * factor<start_mode>,
      end.time - end.accel_delta * factor<end_mode>,
      end.time - start.time - start.accel_delta * (factor<start_mode> - 0.5) - end.accel_delta * (factor<end_mode> - 0.5)
    };
  }
  
};

template<Mode start_mode, Mode end_mode, class Point>
void interpolation3(
  const ParabolicParams<Point>& start,
  const ParabolicParams<Point>& via,
  const ParabolicParams<Point>& end,
  typename SequencedFunction<TimeFunction<Point>, Time>::Container& chain
) {
  assert(start_mode == Mode::ViaPoint || !chain.empty());

  // Timestamps.
  auto t12 = QuadraticTimestamps::build<start_mode, Mode::ViaPoint>(start, via);
  auto t23 = QuadraticTimestamps::build<Mode::ViaPoint, end_mode>(via, end);
  
  // Velocities.
  auto v12 = (via.point - start.point) / t12.delta;
  auto v23 = (end.point - via.point) / t23.delta;

  if constexpr (start_mode == Mode::Point) {
    auto l = quadratic_acceleration(start.point, v12, t12.start, start.accel_delta);
    auto q = linear_interpolation(start.point, via.point, t12.start + start.accel_delta / 2, t12.delta);
    chain.emplace_back(std::move(l), t12.start);
    chain.emplace_back(std::move(q), t12.start_linear);
  }

  auto q = quadratic_interpolation(chain.back()(t12.end_linear), v12, v23, t12.end_linear, via.accel_delta);
  auto l = linear_interpolation(via.point, end.point, t23.start, t23.delta);
  chain.emplace_back(std::move(q), t12.end_linear);
  chain.emplace_back(std::move(l), t23.start_linear);

  if constexpr (end_mode == Mode::Point) {
    chain.push_back({
      quadratic_deceleration(end.point, v23, t23.end, end.accel_delta),
      t23.end_linear
    });
  }
}

template<class Point>
TimeFunction<Point> interpolation2(
  const ParabolicParams<Point>& start,
  const ParabolicParams<Point>& end,
  typename SequencedFunction<TimeFunction<Point>, Time>::Container& chain
) {
  // Timestamp.
  auto t = QuadraticTimestamps::build<Mode::Point, Mode::Point>(start, end);
  
  // Velocity.
  auto v = (end.point - start.point) / t.delta;

  chain.push_back({
    quadratic_acceleration(start.point, v, t.start, start.accel_delta),
    t.start
  });
  chain.push_back({
    linear_interpolation(start.point, end.point, t.start + start.accel_delta / 2, t.delta),
    t.start_linear
  });
  chain.push_back({
    quadratic_deceleration(end.point, v, t.end, end.accel_delta),
    t.end_linear
  });
}

TimeFunction<os::Position> parabolic_interpolation(
  const ParabolicParams<os::Position>& start,
  const std::initializer_list<ParabolicParams<os::Position>>& via_points,
  const ParabolicParams<os::Position>& end
) {
  using Point = os::Position;
  using Sequence = SequencedFunction<TimeFunction<Point>, Time>;

  Sequence::Container chain;

  // - quadratic acceleration from the starting node.
  // - linear movement after the starting node.
  // - quadratic deceleration to the final node.
  // - For each via point:
  //   - quadratic acceleration.
  //   - linear movement.
  chain.resize(3 + 2 * via_points.size());

  if (via_points.size() < 1) {
    return interpolation2(start, end, chain);
  } else if (via_points.size() == 1) {
    interpolation3<Mode::Point, Mode::Point>(start, *via_points.begin(), end, chain);
  } else {
    auto i1 = via_points.begin();
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
