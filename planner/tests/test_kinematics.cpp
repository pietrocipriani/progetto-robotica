#include <cstdlib>
#include <exception>
#include "tester.hpp"
#include "../include/kinematics.hpp"
#include "../include/model.hpp"

using namespace kinematics;
using namespace model;

bool test_movement();
bool test_direct_kinematics();
bool test_inverse_kinematics();
bool test_inverse_diff_kinematics();

int main() {

  test("all", test_movement);

  // TODO: kinematics test

  return EXIT_SUCCESS;
}

bool test_movement() {
  // 100 ms as dt.
  constexpr Scalar dt = 0.1;

  // default constructed robot.
  UR5 robot;

  Pose position = direct_kinematics(robot);

  // 1 min simulation.
  for (Scalar time = 0.0; time < 60.0; time += dt) {
    const auto velocity = Pose::Random().normalized();
    
    const auto config_variation = inverse_diff_kinematics(robot, velocity);

    position += velocity * dt;
    robot.config += config_variation * dt;

    Scalar error = (position - direct_kinematics(robot)).norm();

    // TODO: calibrate tollerance, error is acceptable due to time quantization.
    if (error > 1e-10) {
      std::cerr << "Failed at time " << time << " s with error = " << error << std::endl;
      return false;
    }
  }

  return true;
}
bool test_direct_kinematics() { return false; }
bool test_inverse_kinematics() { return false; }
bool test_inverse_diff_kinematics() { return false; }

