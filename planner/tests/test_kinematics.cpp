#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include "tester.hpp"
#include "kinematics.hpp"
#include "model.hpp"

using namespace kinematics;
using namespace model;

bool test_movement();
bool test_path();
bool test_direct_kinematics();
bool test_inverse_kinematics();
bool test_inverse_diff_kinematics();

int main() {

  test("direct", test_direct_kinematics);
  test("all", test_movement);

  // Assuming the correct operation of `direct_kinematics`.
  test("inverse kinematics", test_inverse_kinematics);


  test("path", test_path);

  // TODO: kinematics test

  return EXIT_SUCCESS;
}

std::ostream& operator<<(std::ostream& out, const Pose_2& pose) {
  out << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ", ";
  auto axis = pose.orientation * (Pose_2::Orientation::Vector3::UnitZ() * 0.02);
  out << axis.x() << ", " << axis.y() << ", " << axis.z();

  return out;
}

bool test_movement() {
  // 100 ms as dt.
  constexpr Scalar dt = 0.01;

  // default constructed robot.
  UR5 robot;

  Pose_2 position = direct(robot);

  // 4 min simulation.
  for (Scalar time = 0.0; time < 60.0 * 2.6; time += dt) {
    Movement velocity(
      Movement::Traslation::Random().normalized() * 0.01 * dt,
      // Can only rotate around the z axis.
      Movement::Rotation::UnitZ() * 0.1 * dt
    );
    // TODO: cannot choose a random path.

    const UR5::Configuration config_variation = dpa_inverse_diff(robot, velocity, position);

    position += velocity;
    robot.config += config_variation;

    auto effective_pos = direct(robot);
    Scalar error = (position - effective_pos).norm();

    // TODO: calibrate tollerance
    if (error > 1e-2) {
      std::stringstream buf;
      buf << "Failed at time " << time << " s with error = " << error
          << " linear part error = " << (position - effective_pos).traslation.norm()
          << " rot part error = " << (effective_pos - position).rotation.norm();
      throw std::runtime_error(buf.str());
    }
  }

  return true;
}
bool test_direct_kinematics() {
  UR5 robot;

  const auto pos = direct(robot);

  const Pose_2 correct(
    Pose_2::Position(0.1518323030153386210550081614201189950109, -0.1908279822262344826988567092485027387738,  0.4550005526318916526662405885872431099415),
    // TODO: convert to euler angles.
    model::Quaternion(0.7111026719196652523535817636002320796251, -0.05565392930722438957769071521397563628852, 0.1157674536882463689480005086807068437338, -0.6912550374557274723841260311019141227007).toRotationMatrix().eulerAngles(0, 1, 2)
  );

  return (correct - pos).norm() < 1e-50;
}
bool test_inverse_kinematics() {
  UR5 robot;

  for (int i = 0; i < 10000; ++i) {
    robot.config = UR5::Configuration::Random() * M_PI * 2;
    const auto initial_pos = direct(robot);

    try {
      robot.config = inverse(robot, initial_pos);
    } catch (std::domain_error& e) {
      std::stringstream buf;
      buf << "Failed with error = " << e.what() << " at iteration " << i << ". Solution: " << robot.config;
      throw std::domain_error(buf.str());
    }

    const auto final_pos = direct(robot);
    Scalar error = (final_pos - initial_pos).norm();

    if (std::isnan(error)) {
      throw std::runtime_error("Unexpected NaN.");
    }

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

Pose::Orientation from_zyx_euler_angles(const Pose::Orientation::VectorType& euler_angles) {
  return Pose::Orientation(
    Pose::Orientation::AngleAxisType(euler_angles[0], Pose::Orientation::AngleAxisType::VectorType::UnitZ()) *
    Pose::Orientation::AngleAxisType(euler_angles[1], Pose::Orientation::AngleAxisType::VectorType::UnitY()) *
    Pose::Orientation::AngleAxisType(euler_angles[2], Pose::Orientation::AngleAxisType::VectorType::UnitX())
  );   
}

Pose interpolate(const Pose& from, const Pose& to, Scalar time) {
  return Pose(from.position * (1 - time) + to.position * time, from.orientation.slerp(time, to.orientation));
}

bool test_path() {
  const Pose initial(Pose::Position(0.3, 0.3, 0.1), Pose::Orientation::Identity());
  const Pose::Position final_position(0.5, 0.5, 0.5);
  const Pose final[]{
    {final_position, from_zyx_euler_angles({M_PI / 3, M_PI_2 - 0.1, M_PI / 3})},
    {final_position, from_zyx_euler_angles({M_PI_4, M_PI_4, M_PI_4})},
    {final_position, from_zyx_euler_angles({-M_PI_2 + 0.1, M_PI / 3, 2 * M_PI / 3})}
  };

  constexpr Scalar dt = 0.01;
  
  UR5 robot;

  const UR5::Configuration initial_config = inverse(robot, initial);

  // TODO: kp, kq

  for (auto& final_pose : final) {
    std::clog << "-------------------------- NEW BATCH -------------------------" << std::endl;

    char filename[] = "/tmp/test_path_XXXXXX\0csv";
    char * temp = mktemp(filename);

    size_t len = strlen(temp);

    std::ofstream file;

    if (len == 0) {
      std::cerr << "Cannot create the csv file." << std::endl;
    } else {
      temp[len] = '.';
      std::cerr << "Plot available at " << temp << "." << std::endl;
      file = std::ofstream(temp);
    }

    robot.config = initial_config;

    Pose current = initial;

    constexpr Scalar max_time = 10;
    for (Scalar time = 0; time < max_time; time += dt) {

      Pose next_pos = interpolate(initial, final_pose, time / max_time);

      Pose movement = current.error(next_pos);

      robot.config += dpa_inverse_diff(robot, movement, current);

      current = std::move(next_pos);

      auto p = direct(robot);
      file << p << ", " << current << "\n";
    }
    Pose movement = current.error(final_pose);
    robot.config += dpa_inverse_diff(robot, movement, current);
    current = final_pose;
    
    auto effective_pos = direct(robot);

    std::clog << "Final error: " << effective_pos.error(final_pose).norm() << std::endl;
    std::clog << "Orientation error: " << effective_pos.error(final_pose).orientation.vec().norm() << std::endl;
    std::clog << "Configuration: " << robot.config.transpose() << std::endl;
  }

  return true;

}
