#ifndef CONSTANTS_HPP_INCLUDED
#define CONSTANTS_HPP_INCLUDED

#include <cmath>
#include "model.hpp"

namespace planner {

/**
 * The distance of the table from the base of the robot.
 */
constexpr model::Scalar table_distance = 0.7;

/**
 * The low zone. Risk of bouncing into obstacles.
 */
constexpr model::Scalar margin = 0.2;

/**
 * The maximum rotational speed a joint can have. [rad/s]
 */
// TODO: check if there is a gear ratio.
constexpr model::Scalar max_joint_speed = M_PI;

/**
 * The maximum velocity in the operational space of the end effector. [m/s]
 */
constexpr model::Scalar max_speed = 0.01;

/**
 * Very rough estimation of the max acceleration for the base joint with
 * a fully extended arm. [rad/s²]
 * Only half of the available torque is used.
 */
constexpr model::Scalar max_joint_accel = 40 * 0.5;

/**
 * The distance of the back control panel (minimum distance in the xy plane).
 */
// TODO: placeholder.
constexpr model::Scalar back_obstacle_distance = 0.35;

}




namespace kinematics {

/**
 * The size of the operational space.
 */
static constexpr size_t os_size = 3;

/**
 * The size of the orientation space.
 * @note Binomial(os_size, 2).
 */
static constexpr size_t orientation_size = os_size * (os_size - 1) / 2;


}



#endif /* CONSTANTS_HPP_INCLUDED */
