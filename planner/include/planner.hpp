#ifndef PLANNER_HPP_INCLUDED
#define PLANNER_HPP_INCLUDED

#include "model.hpp"
#include "utils.hpp"
#include <queue>
#include <stdexcept>
#include <vector>

namespace planner {

/**
 * Enum representing the types of blocks.
 */
enum class Block {
  NO_BLOCK, BLOCK_1, BLOCK_2, BLOCK_3
};

/**
 * The class representing the pose of a block on the table.
 */
struct BlockPose {
private:
  using Scalar = model::Scalar;

public:

  /**
   * The type of the block.
   */
  Block block;

  /**
   * The 2D position of the block.
   */
  Scalar x, y, angle;

  /**
   * The bounding box in order to perform safe movements.
   * Circular in order to simplify the checking.
   */
  Scalar hit_box_radius;


  constexpr BlockPose(
    Scalar x, Scalar y, Scalar angle,
    Scalar hit_box_radius,
    Block block = Block::NO_BLOCK
  ) noexcept;
};

/**
 * Class representing a planned movement of a block from a starting pose to a target pose.
 */
struct BlockMovement {
  /**
   * The initial pose of the block.
   */
  BlockPose start;

  /**
   * The target pose of the block.
   */
  BlockPose target;

  constexpr BlockMovement(const BlockPose& start, const BlockPose& end) noexcept;
  constexpr BlockMovement(BlockPose&& start, BlockPose&& end) noexcept;
};

/**
 * Base exception class for @p generate_block_positioning_order.
 * Permits to recover partial data after an error.
 */
class MovementGenerationRecoveringException : public std::logic_error {
public:
  std::queue<BlockMovement> recovered_data;

  MovementGenerationRecoveringException(const std::string& what, std::queue<BlockMovement>&& movement);
  MovementGenerationRecoveringException(const char* what, std::queue<BlockMovement>&& movement);
};

/**
 * Custom exception.
 * @see @p generate_block_positioning_order
 */
class ConflictingPositionsException : public MovementGenerationRecoveringException {
public:
  ConflictingPositionsException(const std::string& what, std::queue<BlockMovement>&& movement);
  ConflictingPositionsException(const char* what, std::queue<BlockMovement>&& movement);
};

/**
 * Custom exception.
 * @see @p generate_block_positioning_order
 */
class DeadlockException : public MovementGenerationRecoveringException {
public:
  DeadlockException(const std::string& what, std::queue<BlockMovement>&& movement);
  DeadlockException(const char* what, std::queue<BlockMovement>&& movement);
};

/**
 * Generates dependencies between the positioning of the blocks.
 * Some blocks could be into the target position of another block.
 * @param blocks The initial positions of the recognized blocks. 
 * @param targets The desired block target positions.
 * @return A queue of `BlockMovement` with the order of movements in order to avoid conflicts.
 * @throw std::invalid_argument If @p blocks contains more than one block of each type.
 * @throw planner::ConflictingPositionsException If two blocks are initially too near to perform
 *  safely the picking of any.
 * @throw planner::DeadlockException If it is impossible to perform movements due to circular dependencies.
 */
// TODO: Avoid deadlocks searching for auxiliary targets, or return non circular dependencies only.
std::queue<BlockMovement> generate_block_positioning_order(
  const std::vector<BlockPose>& blocks,
  const std::unordered_map<Block, BlockPose>& targets
);

/**
 * Generates the sequence of movements from the initial @p robot configuration, throught the picking
 * of one block from its starting position, to the relase of the block into its target position.
 * @param robot The current robot configuration.
 * @param movement The requested block movement.
 * @param dt The time granularity.
 * @return A sequence of configuration variations in order to perform the given movement.
 * @throw std::domain_error If one of the positions is not in the operational space.
 * @note Evaluate if its worth to run this into a thread in order to perform the next calculations
 *  during the physical driving of the robot for the previous movement.
 */
std::queue<model::UR5::Configuration> joint_space_planning(
  model::UR5& robot,
  const BlockMovement& movement,
  const model::Scalar dt
);

}

#endif /* PLANNER_HPP_INCLUDED */
