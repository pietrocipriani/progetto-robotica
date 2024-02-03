#include <cstdlib>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <vector>
#include "model.hpp"
#include "tester.hpp"
#include "../include/planner.hpp"
#include "types.hpp"

using namespace planner;

bool test_conflict_management();
bool test_movement_planning();
bool test_via_points();

int main() {

  //test("conflict management", test_conflict_management);
  //test("conflict management", test_movement_planning);
  test("via points", test_via_points);

  // TODO: planning test

  return EXIT_SUCCESS;
}

/*bool test_conflict_management() {
  std::vector<BlockPose> poses {
    BlockPose(0, 0, 0, 1, Block::BLOCK_1),
    BlockPose(3, 0, 0, 1, Block::BLOCK_2),
    BlockPose(1.5, 3, 0, 1, Block::BLOCK_3),
  };
  std::unordered_map<Block, BlockPose> targets {
    {Block::BLOCK_1, BlockPose(4.5, 0, 0, 1)},
    {Block::BLOCK_2, BlockPose(0, 3, 0, 1)},
    {Block::BLOCK_3, BlockPose(3, 3, 0, 1)},
  };

  std::queue<BlockMovement> movements = generate_block_positioning_order(poses, targets);

  if (
    movements.front().start.block != Block::BLOCK_3 ||
    (movements.pop(), movements.front().start.block != Block::BLOCK_2) ||
    (movements.pop(), movements.front().start.block != Block::BLOCK_3)
  ) {
    throw std::runtime_error("unexpected sequence");
  }

  // TODO: test deadlocks.
  // TODO: check exceptions.

  return true;
}*/

#include "../lib/planning/interpolation.hpp"
#include "../lib/planning/sequencer.hpp"

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
    os::Position::Angular(Rotation(std::arg(pose.angular()), Axis::UnitZ()))
  );
}

std::ostream& operator<<(std::ostream& out, const kinematics::Pose& pose) {
  Eigen::IOFormat format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ");

  out << pose.linear().transpose().format(format) << format.coeffSeparator;
  #ifndef USE_EULER_ANGLES
    Axis axis = pose.angular() * (Axis::UnitX() * 0.02);
  #else
    auto axis = euler::rotate_axis<os_size>(pose.angular(), Axis::UnitZ() * 0.02);
  #endif
  out << axis.transpose().format(format);

  return out;
}

bool test_via_points() {
  model::UR5 robot;

  const BlockMovement movement(
    {0.3, 0.3, 0, 0},
    {-0.3, -0.3, 0, 0}
  );

  constexpr Time dt = 0.001;

  const os::Position start_pose = block_pose_to_pose(movement.start.pose);
  const os::Position target_pose = block_pose_to_pose(movement.target.pose);

  // Pose outside of the low-zone just above the `start_pose`.
  const os::Position start_safe_pose = safe_pose(start_pose);
  
  // Pose outside of the low-zone just above the `target_pose`.
  const os::Position target_safe_pose = safe_pose(target_pose);

  planner::ViaPoints picking_viapt{start_safe_pose};
  planner::ViaPoints dropping_viapt{start_safe_pose, target_safe_pose};
  
  os::Position current_pose = kinematics::direct(robot);

  if (unsafe(current_pose)) {
    picking_viapt.push_front(safe_pose(current_pose));
  }

  Time finish;

  auto f1 = via_point_sequencer(current_pose, picking_viapt, start_pose, finish);
    
  char filename[] = "/tmp/test_via_points_XXXXXX\0csv";
  char * temp = mktemp(filename);

  size_t len = strlen(temp);

  std::ofstream file;

  if (len == 0) {
    std::cerr << "Cannot create the csv file." << std::endl;
  } else {
    temp[len] = '.';
    std::cerr << "Plot available at " << temp << "." << std::endl;
    file = std::ofstream(temp);
  }
  
  for (Time t = 0.0; t <= finish; t += dt) {
    file << f1(t) << '\n';
  }


  auto f2 = via_point_sequencer(start_pose, dropping_viapt, target_pose, finish);

  for (Time t = 0.0; t <= finish; t += dt) {
    file << f2(t) << '\n';
  }
  

  return true;
}


bool test_movement_planning() { return false; }
