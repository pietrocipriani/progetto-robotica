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
  test("inverse kinematics", test_inverse_kinematics);

  // TODO: kinematics test

  return EXIT_SUCCESS;
}

Scalar error(const Pose& p1, const Pose& p2) {
  Scalar sq_translation_error = (p1.position - p2.position).squaredNorm();
  Scalar rotation_error = (p1.orientation.angularDistance(p2.orientation));
  Scalar sq_rotation_error = std::pow(rotation_error, 2);
  return std::sqrt(sq_translation_error + sq_rotation_error);
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
    Scalar error = ::error(position, effective_pos);

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
bool test_inverse_kinematics() {
  UR5 robot;

  for (int i = 0; i < 10000; ++i) {
    robot.config = UR5::Configuration::Random() * M_PI * 2;
    const auto initial_pos = direct_kinematics(robot);

    try {
      robot.config = inverse_kinematics(robot, initial_pos);
    } catch (std::domain_error& e) {
      std::stringstream buf;
      buf << "Failed with error = " << e.what() << " at iteration " << i << ". Solution: " << robot.config;
      throw std::domain_error(buf.str());
    }

    const auto final_pos = direct_kinematics(robot);
    Scalar error = ::error(final_pos, initial_pos);

    if (error > 1e-10) {
      std::stringstream buf;
      buf << "Failed with error = " << error << " at iteration " << i;
      throw std::runtime_error(buf.str());
    }
  }

  return true;

  // TODO: test exceptions throw.
}
bool test_inverse_diff_kinematics() { return false; }

