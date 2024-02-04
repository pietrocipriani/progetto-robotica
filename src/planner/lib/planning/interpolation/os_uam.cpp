/*#include "../interpolation.hpp"

namespace planner {

using namespace model;


std::function<os::Position(Scalar)> os_uam_interpolation(
  const os::Position& initial_position,
  const os::Velocity& initial_velocity,
  const os::Velocity& final_velocity,
  Scalar acceleration
) {
  // We are only interested on the magnitude of the acceleration.
  acceleration = std::abs(acceleration);

  const auto delta_v = final_velocity - initial_velocity;

  // The acceleration vector.
  const auto acc_vector = delta_v.normalized() * acceleration;

  // The time at which the final_velocity is reached.
  // NOTE: actually IEC 559 would not have required the check.
  const Scalar saturation_time = acceleration == 0
    ? std::numeric_limits<Scalar>::infinity()
    : delta_v.norm() / acceleration;
  
  return [=, &initial_position, &initial_velocity, &final_velocity](Scalar time) -> os::Position {
    if (time < saturation_time) {
      // TODO: check if the quaternion algebra is applicable.
      // NOTE: this function is applied in the lifting phase where no rotations are required.
      // NOTE: model::Scalar can represent up to 16 decimal digits.
      //        This means that `time` has to be < 10^8 to avoid representation issues.
      //        This means that the simulation cannot last more than 100.000 s at 1kHz.
      //        This is an acceptable threshold.
      return initial_position + initial_velocity * time + 0.5 * acc_vector * std::pow(time, 2);
    } else {
      return initial_position + final_velocity * time - 0.5 * saturation_time * delta_v;
    }
  };
}


}*/
