#include "planner.hpp"

namespace planner {


BlockPose::BlockPose(
  Scalar x, Scalar y, Scalar angle,
  Scalar hit_box_radius,
  Block block
) noexcept : block(block), position(x, y), orientation(angle), hit_box_radius(hit_box_radius) {}

bool BlockPose::collides(const BlockPose& other) const {
  auto distance = position - other.position;
  Scalar min_distance = hit_box_radius + other.hit_box_radius;

  return distance.norm() < min_distance;
}


BlockMovement::BlockMovement(const BlockPose& start, const BlockPose& target) noexcept
  : start(start), target(target) {}

BlockMovement::BlockMovement(BlockPose&& start, BlockPose&& target) noexcept
  : start(std::move(start)), target(std::move(target)) {}


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


}
