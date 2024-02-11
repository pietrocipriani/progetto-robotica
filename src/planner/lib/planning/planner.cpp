#include "planner.hpp"
#include <complex>
#include <queue>

namespace planner {


BlockPose::BlockPose(Block block, Scalar x, Scalar y, Scalar angle) noexcept
  : block(block), pose({x, y}, BlockPose::Pose::Angular(angle)) {}

BlockPose BlockPose::pad_pose(Block block) noexcept {
  auto [x, y, angle] = get_pad_position(block);
  return BlockPose(block, x, y, angle);
}

bool BlockPose::collides(const BlockPose& other) const {
  auto distance = pose.linear() - other.pose.linear();
  Scalar min_distance = get_hit_box_radius(block) + get_hit_box_radius(other.block);

  return distance.norm() < min_distance;
}

os::Position BlockPose::to_os_position() const {
  return os::Position(
    os::Position::Linear(
      pose.linear().x() - gazebo_to_os_x,
      -(pose.linear().y() - gazebo_to_os_y),
      table_distance - get_gripping_height(block)
    ),
    os::Position::Angular(Rotation(-pose.angular()[0], Axis::UnitZ()))
  );
}


BlockMovement::BlockMovement(const BlockPose& start, const BlockPose& target) noexcept
  : start(start), target(target) {}

BlockMovement::BlockMovement(BlockPose&& start, BlockPose&& target) noexcept
  : start(std::move(start)), target(std::move(target)) {}




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


MovementSequence::ConfigGenerator::ConfigGenerator(std::function<Ret()>&& next)
    : next(std::move(next)) {}

MovementSequence::ConfigGenerator::const_iterator::const_iterator(const ConfigGenerator& parent)
      : next(parent.next), current(next()) {}

MovementSequence::ConfigGenerator::const_iterator::const_iterator()
      : next(), current(std::make_tuple(model::UR5::Configuration(), true)) {}

bool MovementSequence::ConfigGenerator::const_iterator::operator!=(const const_iterator& other) const {
  return std::get<bool>(current) != std::get<bool>(other.current);
}

bool MovementSequence::ConfigGenerator::const_iterator::operator==(const const_iterator& other) const {
  return !(*this != other);
}

MovementSequence::ConfigGenerator::const_iterator::const_reference MovementSequence::ConfigGenerator::const_iterator::operator*() {
  return std::get<Point>(current);
}

MovementSequence::ConfigGenerator::const_iterator::const_reference MovementSequence::ConfigGenerator::const_iterator::operator*() const {
  return std::get<Point>(current);
}

MovementSequence::ConfigGenerator::const_iterator MovementSequence::ConfigGenerator::const_iterator::operator++() {
  current = next();
  return *this;
}

MovementSequence::ConfigGenerator::const_iterator MovementSequence::ConfigGenerator::const_iterator::operator++(int) {
  auto copy = *this;
  current = next();
  return copy;
}

MovementSequence::ConfigGenerator::const_iterator MovementSequence::ConfigGenerator::begin() const {
  return const_iterator(*this);
}

MovementSequence::ConfigGenerator::const_iterator MovementSequence::ConfigGenerator::end() const {
  return const_iterator();
}


}
