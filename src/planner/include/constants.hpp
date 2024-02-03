#ifndef CONSTANTS_HPP_INCLUDED
#define CONSTANTS_HPP_INCLUDED

#include <cmath>
#include "types.hpp"

namespace planner {

/**
 * The distance of the table from the base of the robot.
 */
constexpr Scalar table_distance = 0.7;

/**
 * The low zone. Risk of bouncing into obstacles.
 */
constexpr Scalar margin = 0.1;

/**
 * The maximum velocity in the operational space of the end effector. [m/s]
 */
constexpr Scalar max_speed = 0.01;

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
