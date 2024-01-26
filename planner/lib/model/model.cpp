#include "model.hpp"
#include <array>
#include <cmath>
#include <limits>
#include "utils.hpp"

namespace model {

constexpr RevoluteJoint::RevoluteJoint(
  Scalar d, Scalar& theta, Scalar a, Scalar alpha,
  Scalar min_config, Scalar max_config
) noexcept : d(d), theta(theta), a(a), alpha(alpha), min_config(min_config), max_config(max_config) {}


/**
 * Default homing configuration for the UR5 manipulator.
 * Exported by `params.py`.
 */
// TODO: not hardcoded: Json?
inline const UR5::Configuration ur5_default_homing_config{-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};

UR5::UR5() noexcept : UR5(ur5_default_homing_config) {}

/**
 * Generates the UR5 joint parameters for a robot.
 * @param config The config of the robot to which link the thetas.
 * @return The list of parameters.
 * @note Due to design choices, the joint variable parameter (theta) has to be linked with the robot configuration.
 * @note The choosen measurement units are 'meter' and 'radiant'.
 */
std::array<RevoluteJoint, UR5::dof> generate_ur5_parameters(
  UR5::Configuration& config,
  const Scalar& scale_factor = 1.0
) {
  // TODO: Only C++20 offers non-C style math constants?
  static constexpr Scalar pi2 = M_PI_2;

  // TODO: make config reference joints.
  /*using Parameters = std::array<Scalar, UR5::dof>;

  static constexpr Parameters ds{{0.1625, 0, 0, 0.1333, 0.0997, 0.0996}};
  static constexpr Parameters thetas{{-0.32, -0.78, -2.56, -1.63, -1.57, 3.49}};
  static constexpr Parameters as{{0, -0.425, -0.3922, 0, 0, 0}};
  static constexpr Parameters alphas{{pi2, 0, 0, pi2, -pi2, 0}};*/

  // TODO: add constraints.
  std::array<RevoluteJoint, UR5::dof> joints {{
    {0.1625 , config[0] ,  0      ,  pi2 },
    {0      , config[1] , -0.425  ,  0   },
    {0      , config[2] , -0.3922 ,  0   },
    {0.1333 , config[3] ,  0      ,  pi2 },
    {0.0997 , config[4] ,  0      , -pi2 },
    {0.0996 , config[5] ,  0      ,  0   }
  }};

  // Scales every linear parameter.
  for (auto& joint : joints) {
    joint.a *= scale_factor;
    joint.d *= scale_factor;
  }

  return joints;
}

UR5::UR5(const Configuration& homing_config) noexcept
  : config(homing_config), joints(generate_ur5_parameters(config)) {}


UR5::UR5(Configuration&& homing_config) noexcept
  : config(std::move(homing_config)), joints(generate_ur5_parameters(config)) {}


}
