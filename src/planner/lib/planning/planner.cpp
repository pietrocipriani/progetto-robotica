#include "planner.hpp"
#include <complex>
#include <queue>

namespace planner {


BlockPose::BlockPose(
  Scalar x, Scalar y, Scalar angle,
  Scalar hit_box_radius,
  Block block
) noexcept : block(block), pose({x, y}, BlockPose::Pose::Angular(angle)), hit_box_radius(hit_box_radius) {}

bool BlockPose::collides(const BlockPose& other) const {
  auto distance = pose.linear() - other.pose.linear();
  Scalar min_distance = hit_box_radius + other.hit_box_radius;

  return distance.norm() < min_distance;
}


BlockMovement::BlockMovement(const BlockPose& start, const BlockPose& target) noexcept
  : start(start), target(target) {}

BlockMovement::BlockMovement(BlockPose&& start, BlockPose&& target) noexcept
  : start(std::move(start)), target(std::move(target)) {}


std::queue<BlockMovement> generate_block_positioning_order(
  const std::vector<BlockPose>& blocks,
  const std::unordered_map<Block, BlockPose>& targets
) {
  std::queue<BlockMovement> order;

  // TODO: dummy implementation.
  for (auto& b : blocks) {
    order.emplace(b, targets.at(b.block));
  }
  
  return order;
}


os::Position safe_pose(const os::Position& pose) {
  static constexpr Scalar safe_z = table_distance - margin - std::numeric_limits<Scalar>::epsilon();

  auto safe_pose = pose;

  // The robot could already be in a safe position.
  safe_pose.linear().z() = std::min(safe_pose.linear().z(), safe_z);

  return safe_pose;
}



bool unsafe(const os::Position& pose) {
  // only the z coord is checked. Relative to the robot base frame.
  auto& position = pose.linear().z();

  return position > table_distance - margin;
}


os::Position block_pose_to_pose(const BlockPose::Pose& pose) {
  // TODO: dummy implementation. Could require a transformation between the two frames.
  return os::Position(
    os::Position::Linear(pose.linear().x(), pose.linear().y(), table_distance),
    os::Position::Angular(Rotation(pose.angular()[0], Axis::UnitZ()))
  );
}

}
