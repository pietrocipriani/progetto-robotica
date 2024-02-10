#ifndef PLANNER_HPP_INCLUDED
#define PLANNER_HPP_INCLUDED

#include "model.hpp"
#include "spaces.hpp"
#include "utils.hpp"
#include "kinematics.hpp"
#include "block_type.hpp"
#include <queue>
#include <stdexcept>
#include <variant>
#include <vector>

namespace planner {

namespace os {
using Position = kinematics::Pose<>;
using Velocity = kinematics::Velocity<>;
using Acceleration = kinematics::Acceleration<>;
}
namespace js {
using Position = model::UR5::Configuration;
using Velocity = model::UR5::Velocity;
using Acceleration = model::UR5::Acceleration;
}

/// The class representing the pose of a block on the table.
///
struct BlockPose {
public:

  using Pose = OperationalSpace<coord::Cartesian, coord::Euler, 2>;

  /// The type of the block.
  ///
  Block block;

  Pose pose;

  Scalar hit_box_radius;

  BlockPose(Block block, Scalar x, Scalar y, Scalar angle) noexcept;

  /// Checks if @p this block collides with @p other.
  /// Collision is computed according to position and @p hit_box_radius.
  /// @param other The other `BlockPose` to check the collision with.
  /// @return `true` if the two hitbox collides, `false` otherwise.
  bool collides(const BlockPose& other) const;

  os::Position to_os_position() const;
};

/// Class representing a planned movement of a block from a starting pose to a target pose.
///
struct BlockMovement {
  /// The initial pose of the block.
  ///
  BlockPose start;

  /// The target pose of the block.
  ///
  BlockPose target;

  BlockMovement(const BlockPose& start, const BlockPose& target) noexcept;
  BlockMovement(BlockPose&& start, BlockPose&& target) noexcept;
};

/// Base exception class for @p generate_block_positioning_order.
/// Permits to recover partial data after an error.
class MovementGenerationRecoveringException : public std::logic_error {
public:
  std::queue<BlockMovement> recovered_data;

  MovementGenerationRecoveringException(const std::string& what, std::queue<BlockMovement>&& movement);
  MovementGenerationRecoveringException(const char* what, std::queue<BlockMovement>&& movement);
};

/// Custom exception.
/// @see @p generate_block_positioning_order
class ConflictingPositionsException : public MovementGenerationRecoveringException {
public:
  ConflictingPositionsException(const std::string& what, std::queue<BlockMovement>&& movement);
  ConflictingPositionsException(const char* what, std::queue<BlockMovement>&& movement);
};

/// Custom exception.
/// @see @p generate_block_positioning_order.
class DeadlockException : public MovementGenerationRecoveringException {
public:
  DeadlockException(const std::string& what, std::queue<BlockMovement>&& movement);
  DeadlockException(const char* what, std::queue<BlockMovement>&& movement);
};

/// Sequence of configurations to perform the movement of a certain block.
/// @note Two phases are returned to allow the controller to perform the picking.
struct MovementSequence {
  struct ConfigGenerator {
    using Point = model::UR5::Configuration;
    using Ret = std::tuple<Point, bool>;

    std::function<Ret()> next;

    ConfigGenerator(std::function<Ret()>&& next);

    class const_iterator {
    private:
      std::function<Ret()> next;
      Ret current;

      const_iterator(const ConfigGenerator& parent);
      const_iterator();

      friend ConfigGenerator;
    public:
      using iterator_category = std::input_iterator_tag;
      using value_type = Point;
      using difference_type = Time;
      using pointer = void;
      using reference = value_type;
      using const_reference = const value_type;

      bool operator!=(const const_iterator& other) const;

      bool operator==(const const_iterator& other) const;

      const_reference operator*();
      const_reference operator*() const;

      const_iterator operator++();
      const_iterator operator++(int);
    };

    const_iterator begin() const;
    const_iterator end() const;
  };

  ConfigGenerator lazy_picking, lazy_dropping;
};

/// Generates dependencies between the positioning of the blocks.
/// Some blocks could be into the target position of another block.
/// @param blocks The initial positions of the recognized blocks.
/// @param targets The desired block target positions.
/// @return A queue of `BlockMovement` with the order of movements in order to avoid conflicts.
/// @throw std::invalid_argument If @p blocks contains more than one block of each type.
/// @throw planner::ConflictingPositionsException If two blocks are initially too near to perform
///  safely the picking of any.
/// @throw planner::DeadlockException If it is impossible to perform movements due to circular dependencies.
// TODO: Avoid deadlocks searching for auxiliary targets, or return non circular dependencies only.
std::queue<BlockMovement> generate_block_positioning_order(
  const std::vector<BlockPose>& blocks,
  const std::unordered_map<Block, BlockPose>& targets
);

/// Generates the sequence of movements from the initial @p robot configuration, throught the picking
/// of one block from its starting position, to the relase of the block into its target position.
/// @param robot The current robot configuration.
/// @param movement The requested block movement.
/// @param dt The time granularity.
/// @return A sequence of configurations in order to perform the given movement.
/// @throw std::domain_error If one of the positions is not in the operational space.
// TODO: Is it worth to run this into a thread in order to perform the next calculations
//  during the physical driving of the robot for the previous movement?
MovementSequence plan_movement(
  model::UR5& robot,
  const BlockMovement& movement,
  const Time& dt
);

}

#endif /* PLANNER_HPP_INCLUDED */
