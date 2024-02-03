#include "kinematics.hpp"
#include "direct_kinematics.hpp"
#include "model.hpp"
#include "euler.hpp"
#include "types.hpp"
#include <cerrno>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace kinematics {

using namespace model;

/**
 * List of possible configurations.
 */
using Configurations = std::vector<UR5::Configuration>;

enum Theta : size_t {
  t1 = 0, t2 = 1, t3 = 2, t4 = 3, t5 = 4, t6 = 5
};

#define theta_1 config[t1]
#define theta_2 config[t2]
#define theta_3 config[t3]
#define theta_4 config[t4]
#define theta_5 config[t5]
#define theta_6 config[t6]

/**
 * Calls @p function @p n times with the various parameters in @p variable_argument
 * and condensate the configs into a single Configurations.
 * @param index The index of the theta parameter in order to update the configs. 1-based.
 * @param Function The Function type. Must accept as last argument an Arg.
 *  Must be a function returning a Configurations or a UR5::Configuration.
 * @param ArgList The type of a list containing the values for the variable argument.
 * @param Args the other parameters for @p Function.
 * @param variable_argument The list of variable arguments.
 * @param function The function to generate configurations.
 * @param args The arguments for function.
 * @return The list of configurations.
 * @throw Whatever is thrown by @p function.
 */
template<Theta theta, bool fail_silently, typename Function, typename ArgList, class... Args>
Configurations expand(
  const ArgList& variable_argument, Function&& function, UR5::Configuration::Base& config, Args&&... args
) {
  Configurations result;

  // Generate solutions for every possible value of the variable argument.
  for (const auto& arg : variable_argument) {
    config[theta] = arg;
    try {
      // Invokes the function.
      auto configs = std::invoke(function, config, args...);

      if constexpr (std::is_same_v<UR5::Configuration, std::remove_cv_t<decltype(configs)>>) {
        // If function returns a Configuration...
        result.push_back(configs);
      } else {
        // If function returns a Configurations...
        result.insert(result.end(), configs.begin(), configs.end());
      }
    } catch (const std::domain_error& e) {
      if constexpr (!fail_silently) {
        throw;
      }
    }
  }

  return result;
}

/**
 * Calculates the angle in the xy plane of the vector @p vector.
 * @param vector The vector.
 * @return The angle.
 */
Scalar vector_angle_xy(const Axis& vector) {
  return std::atan2(vector.y(), vector.x());
}

/**
 * Calculates the inverse transformation matrix for @p joint with the given @p theta.
 * @param joint The transforming joint.
 * @param theta The configuration theta.
 * @return The transformation matrix for the inverse transformation.
 */
JointTransformation inverse_trans(const RevoluteJoint& joint, Scalar theta) {
  return joint_transformation_matrix(joint, theta).inverse();
}

/**
 * Calculates the rotated @p axis after the transformation @trans.
 * @param trans The transformation.
 * @param axis The axis.
 * @return The resulting axis.
 * @note Only the rotation is applied.
 */
Axis rotate(const JointTransformation& trans, const Axis& axis) {
  return trans.linear() * axis;
}


/**
 * Computes the two solutions of cos(theta) = @p cos.
 * @param cos The cosine value.
 * @return An array containing the two solutions.
 * @throw std::domain_error In case of domain errors.
 * @note Can return two coincident solutions.
 */
std::array<Scalar, 2> acos(Scalar cos) {
  errno = 0;
  Scalar res = std::acos(cos);

  // Check if a domain error occoured.
  if (errno == EDOM) {
    throw std::domain_error("|cos| > 1");
  }

  // Returns the two solutions.
  return {{-res, res}};
}

/**
 * Computes the two solutions of sin(theta) = @p sin.
 * @param sin The sine value.
 * @return An array containing the two solutions.
 * @throw std::domain_error In case of domain errors.
 * @note Can return two coincident solutions.
 */
std::array<Scalar, 2> asin(Scalar sin) {
  errno = 0;
  Scalar res = std::asin(sin);

  // Check if a domain error occoured.
  if (errno == EDOM) {
    throw std::domain_error("|sin| > 1");
  }

  // Returns the two solutions.
  return {{res, M_PI - res}};
}

/**
 * Performs the inverse kinematics for @p robot given theta_3.
 * @param robot The robot parameters.
 * @param direct_kin The precalculated direct kinematics from frame 6 to frame 1.
 * @param theta_3 The value of theta_5.
 * @param origin_3 The origin_3 respect to origin_1. For efficency only.
 * @return The configuration to obtain the given @p pose (theta_2, theta_3, theta_4, theta_5, theta_6).
 */
UR5::Configuration configs_given_theta_3(
  UR5::Configuration::Base& config, const UR5& robot, JointTransformation direct_kin, const Pose::Linear& origin_3
) {
  // Radius angle + angle inside the triangle.
  theta_2 = vector_angle_xy(-origin_3)
          + std::asin(robot.elbow.a * sin(theta_3) / origin_3.norm());

  // Inversion of the elbow transformation
  const auto cancel_theta_3 = inverse_trans(robot.elbow, theta_3);

  // Inversion of the shoulder transformation
  const auto cancel_theta_2 = inverse_trans(robot.shoulder, theta_2);

  // Transformation from frame 4 to frame 3.
  direct_kin = cancel_theta_3 * cancel_theta_2 * direct_kin;

  // x4 axis in frame 3 coords.
  const Axis x4_frame_3 = rotate(direct_kin, Axis::UnitX());

  theta_4 = vector_angle_xy(x4_frame_3);

  return UR5::Configuration(config);
}

/**
 * Performs the inverse kinematics for @p robot given theta_5.
 * @param robot The robot parameters.
 * @param direct_kin The precalculated direct kinematics from frame 6 to frame 1.
 * @param theta_5 The value of theta_5.
 * @return The possible configurations to obtain the given @p pose (theta_2, theta_3, theta_4, theta_5, theta_6).
 * @throws @p std::domain_error if the desired @p pose in not in the operational space.
 * @note UR5 is not redundant, however multiple (finite) solutions are allowed.
 */
Configurations configs_given_theta_5(
  UR5::Configuration::Base& config, const UR5& robot, JointTransformation direct_kin
) {
  // The transformation matrix from frame 1 to frame 6.
  // TODO: more efficient to only invert the rotation.
  //       Complete inversion for readibility only.
  const JointTransformation inverse_kin = direct_kin.inverse();

  // Axis z1 in the end effector frame.
  const Axis z1_frame_6 = rotate(inverse_kin, Axis::UnitZ());

  // The coefficient of alignment between z1 and z6. Lower: the smaller the difference.
  const Scalar alignment_z1_z6 = z1_frame_6.head<2>().norm();

  // TODO: define tollerance
  // NOTE: it can be proven that this condition is equivalent to sin(theta_5) < eps.
  // NOTE: it can be proven that it is sufficient to take eps' = eps / sqrt(2)
  //         in order to have this condition implied by x < eps' && y < eps'.
  //       This is an acceptable crop.
  if (alignment_z1_z6 < 1e-7) {
    // If z1 and z6 are aligned, there exist infinite solutions.
    // Cannot represent continuous solutions. Arbitrary value for theta_6.
    // NOTE: IEEE 754 compliant systems have std::atan2(0, 0) == 0.
    theta_6 = 0;
  } else {
    theta_6 = -vector_angle_xy(z1_frame_6 / std::sin(theta_5));
  }

  // Inversion of the wrist3 transformation
  const auto cancel_theta_6 = inverse_trans(robot.wrist3, theta_6);

  // Inversion of the wrist2 transformation
  const auto cancel_theta_5 = inverse_trans(robot.wrist2, theta_5);

  // Transformation from frame 4 to frame 1.
  direct_kin = direct_kin * cancel_theta_6 * cancel_theta_5;

  // The origin of frame 3 respect to frame 1.
  // Obtained going backwards along y4 of d4.
  const Pose::Linear origin_3 = direct_kin * (- Axis::UnitY() * robot.wrist1.d);

  // The magnitude of the radius squared between origin_3 and origin_1.
  const Scalar radius_sq = origin_3.squaredNorm();

  // Given a2, a3 the vectors associated with links 2 and 3.
  // radius_sq = |a2 + a3|^2 <=> radius_sq = |a2|^2 + |a3|^2 + 2·|a2|·|a3|·cos(theta_3)
  // cos(theta_3) = (radius_sq - |a2|^2 - |a3|^2) / 2·|a2|·|a3|
  const Scalar cos_theta_3 = (radius_sq - std::pow(robot.shoulder.a, 2) - std::pow(robot.elbow.a, 2))
                           / (2 * robot.shoulder.a * robot.elbow.a);

  // Outside of the cos domain.
  if (std::abs(cos_theta_3) > 1) {
    throw std::domain_error("Unreachable, too distant.");
  }

  const auto thetas_3 = acos(cos_theta_3);

  Configurations configs = expand<t3, false>(thetas_3, configs_given_theta_3, config, robot, direct_kin, origin_3);
  
  return configs;
}

/**
 * Performs the inverse kinematics for @p robot given theta_1.
 * @param robot The robot parameters.
 * @param direct_kin The precalculated direct kinematics for efficiency.
 * @param theta_1 The value of theta_1.
 * @return The possible configurations to obtain the given @p pose.
 * @throws @p std::domain_error if the desired @p pose in not in the operational space.
 * @note UR5 is not redundant, however multiple (finite) solutions are allowed.
 */
Configurations configs_given_theta_1(
  UR5::Configuration::Base& config, const UR5& robot, JointTransformation direct_kin
) {
  // Inversion of the base joint transformation.
  // NOTE: The inverse should be computed efficiently by Eigen due to the isometry property of the transformation.
  //       Eventually can be costructed directly as the inverse transformation.
  const auto cancel_theta_1 = inverse_trans(robot.base, theta_1);

  // Direct kinematics relative to frame 1.
  direct_kin = cancel_theta_1 * direct_kin;

  // The origin of frame 6.
  const Pose::Linear origin_6 = direct_kin.translation();

  // The value of theta_5 (+-)
  const auto thetas_5 = acos((origin_6.z() - robot.wrist1.d) / robot.wrist3.d);

  return expand<t5, true>(thetas_5, configs_given_theta_5, config, robot, direct_kin);
}

/**
 * Performs the inverse kinematics for @p robot.
 * @param robot The robot parameters.
 * @param pose The desired pose.
 * @return The possible configurations to obtain the given @p pose.
 * @throws @p std::domain_error if the desired @p pose in not in the operational space.
 * @note UR5 is not redundant, however multiple (finite) solutions are allowed.
 */
Configurations configurations(const UR5& robot, const Pose& pose) {
  UR5::Configuration::Base config;

  // Construct the direct kinematics matrix from the final pose.
  #ifndef USE_EULER_ANGLES
    JointTransformation direct_kin = Translation(pose.linear()) * pose.angular();
  #else
    JointTransformation direct_kin = Translation(pose.linear()) * euler::to_rotation<os_size>(pose.angular());
  #endif

  // The origin of frame 5.
  // Obtained going backwards along z6 of d6.
  // NOTE: the axis inversion has to be performed before the homogeneous transformation.
  const Pose::Linear origin_5 = direct_kin * (- Axis::UnitZ() * robot.wrist3.d);

  // The distance in the xy plane between origin_5 and origin_0.
  const Scalar distance_5_0_xy = origin_5.head<2>().norm();

  // Check if the distance is smaller than d4.
  // In such a case the requested pose is unreachable.
  if (distance_5_0_xy < robot.wrist1.d) {
    throw std::domain_error("Unreachable pose: internal cylinder.");
  }

  // The angle between origin and origin_5 in the xy plane.
  const Scalar angle_5_0_xy = vector_angle_xy(origin_5);

  // The shift angle respect to the planar arm.
  auto thetas_1 = asin(robot.wrist1.d / distance_5_0_xy);

  // Adding the global rotation.
  for (auto& val : thetas_1) val += angle_5_0_xy;
  
  // The two solutions are symmetrical, if the first fails also the other will.
  return expand<t1, false>(thetas_1, configs_given_theta_1, config, robot, direct_kin);
}

UR5::Configuration inverse(const UR5& robot, const Pose& pose) {
  // Possible configurations
  const auto configs = configurations(robot, pose);

  if (configs.empty()) {
    throw std::domain_error("Unreachable position.");
  }

  // TODO: choose the solution.
  return configs.front();
}

#undef theta_1
#undef theta_2
#undef theta_3
#undef theta_4
#undef theta_5
#undef theta_6


}