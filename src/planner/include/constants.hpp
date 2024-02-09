#ifndef CONSTANTS_HPP_INCLUDED
#define CONSTANTS_HPP_INCLUDED

#include <cmath>
#include "types.hpp"

namespace planner {

/**
 * The distance of the table from the base of the robot.
 */
constexpr Scalar table_distance = 0.70;

/**
 * The offset along the x axis of the arm operational space with respect to the gazebo frame.
 */
constexpr Scalar gazebo_to_os_x = 0.5;

/**
 * The offset along the y axis of the arm operational space with respect to the gazebo frame.
 * Note that after subtracting the offset, the y axis also needs to be inverted.
 */
constexpr Scalar gazebo_to_os_y = 0.35;

/**
 * The low zone. Risk of bouncing into obstacles.
 */
constexpr Scalar margin = 0.1;

/**
 * The maximum velocity in the operational space of the end effector. [m/s]
 */
[[deprecated("Use max_linear_speed and max_angular_speed instead.")]]
constexpr Scalar max_speed = 0.01;

/// The maximum linear speed in the operational space for the end effector. [m/s]
///
constexpr Scalar max_linear_speed = 0.1;

/// The maximum angular speed for the end effector. [rad/s].
///
constexpr Scalar max_angular_speed = M_PI_4;

/**
 * The distance of the back control panel (minimum distance in the xy plane).
 */
// TODO: placeholder.
constexpr Scalar back_obstacle_distance = 0.35;

}




namespace kinematics {

/**
 * The size of the operational space.
 */
static constexpr size_t os_size = 3;

}

Scalar constexpr dummy_precision = 1e-8;

#endif /* CONSTANTS_HPP_INCLUDED */
