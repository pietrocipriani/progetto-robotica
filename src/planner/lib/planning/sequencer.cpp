#include "sequencer.hpp"
#include "interpolation/parabolic.hpp"
#include "interpolation/stop_and_play.hpp"
#include "planner.hpp"
#include "utils/coordinates.hpp"
#include "interpolation.hpp"
#include <algorithm>


namespace planner {

using LinearParams = Params<os::Position::Linear>;
using AngularParams = Params<os::Position::Angular>;

LinearParams make_lp(const os::Position::Linear& p, const Time& t, const Time& acc) {
  using namespace coordinates;

  return {convert<Cartesian, Cylindrical>(p), t, acc};
}
AngularParams make_ap(const os::Position::Angular& p, const Time& t, const Time& acc) {
  using namespace coordinates;

  return {p, t, acc};
}


TimeFunction<os::Position> via_point_sequencer(
  const os::Position& current_pose,
  const ViaPoints& viapoints,
  const os::Position& target_pose,
  Time& finish_time
) {
  using namespace coordinates;

  LinearParams start_lp = make_lp(current_pose.linear(), 0, 4);
  AngularParams start_ap = make_ap(current_pose.angular(), 0, 4);

  LinearParams end_lp = make_lp(target_pose.linear(), 10.0 * (viapoints.size() + 1), 4);
  AngularParams end_ap = make_ap(target_pose.angular(), 10.0 * (viapoints.size() + 1), 4);

  std::vector<LinearParams> vlp;
  std::vector<AngularParams> vap;
  vlp.reserve(viapoints.size());
  vap.reserve(viapoints.size() + 1);

  for (auto& pose : viapoints) {
    vlp.push_back(make_lp(pose.linear(), 10.0 * (vlp.size() + 1), 4));
    vap.push_back(make_ap(pose.angular(), 10.0 * (vap.size() + 1), 4));
  }
  vap.push_back(end_ap);

  finish_time = end_lp.time;

  auto linear_fun = parabolic_interpolation(start_lp, vlp, end_lp);
  auto angular_fun = stop_and_play_interpolation(start_ap, vap);
  
  return [lf = std::move(linear_fun), af = std::move(angular_fun)](const Time& t) {
    return os::Position(convert<Cylindrical, Cartesian>(lf(t)), af(t));
  };
}


}

