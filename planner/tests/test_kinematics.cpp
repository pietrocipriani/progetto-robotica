#include <cmath>
#include <cstdlib>
#include <exception>
#include <sstream>
#include <stdexcept>
#include "tester.hpp"
#include "kinematics.hpp"
#include "model.hpp"

using namespace kinematics;
using namespace model;

bool test_movement();
bool test_direct_kinematics();
bool test_inverse_kinematics();
bool test_inverse_diff_kinematics();

int main() {

  test("all", test_movement);

  // Assuming the correct operation of `direct_kinematics`.
  // test("inverse kinematics", test_inverse_kinematics);

  // TODO: kinematics test

  return EXIT_SUCCESS;
}

bool test_movement() {
  // 100 ms as dt.
  constexpr Scalar dt = 0.01;

  // default constructed robot.
  UR5 robot;

  Pose position = direct_kinematics(robot);

  // 1 min simulation.
  for (Scalar time = 0.0; time < 60.0; time += dt) {
    Pose velocity(
      Pose::Position::Random().normalized() * 0.01 * dt,
      Pose::Orientation(Eigen::AngleAxis<Scalar>(0.1 * dt, Pose::Orientation::Vector3::Random().normalized()))
    );

    const UR5::Configuration config_variation = inverse_diff_kinematics(robot, velocity);

    position += velocity;
    robot.config += config_variation;

    auto effective_pos = direct_kinematics(robot);
    Scalar error = std::sqrt((position.position - effective_pos.position).squaredNorm() + std::pow(abs(position.orientation.angularDistance(effective_pos.orientation)), 2));

    // TODO: calibrate tollerance, error is acceptable due to time quantization.
    if (error >= 4e-3) {
      std::stringstream buf;
      buf << "Failed at time " << time << " s with error = " << error;
      throw std::runtime_error(buf.str());
    }
  }

  return true;
}
bool test_direct_kinematics() { return false; }
/*bool test_inverse_kinematics() {
  UR5 robot;

  const auto initial_pos = direct_kinematics(robot);
  robot.config = inverse_kinematics(robot, initial_pos);
  const auto final_pos = direct_kinematics(robot);

  Scalar error = (final_pos - initial_pos).norm();

  if (error > 1e-10) {
    std::stringstream buf;
    buf << "Failed with error = " << error;
    throw std::runtime_error(buf.str());
  }

  return true;

  // TODO: test exceptions throw.
}*/
bool test_inverse_diff_kinematics() { return false; }

