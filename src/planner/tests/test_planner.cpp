#include <cstdlib>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <vector>
#include "kinematics.hpp"
#include "model.hpp"
#include "tester.hpp"
#include "../include/planner.hpp"
#include "types.hpp"
#include "utils/coordinates.hpp"
#include "block_type.hpp"

using namespace planner;

bool test_conflict_management();
bool test_movement_planning();
bool test_planner();
bool test_linear_interpolator();

int main() {

  test("conflict management", test_conflict_management);
  //test("conflict management", test_movement_planning);
  test("linear", test_linear_interpolator);
  test("planner", test_planner);

  // TODO: planning test

  return EXIT_SUCCESS;
}

bool test_conflict_management() {
  std::vector<BlockPose> poses {
    BlockPose(Block::B_1x1_H, 0, 0, 0),
    BlockPose(Block::B_2x1_H, 3, 0, 0),
    BlockPose(Block::B_2x1_L, 1.5, 3, 0),
  };
  std::unordered_map<Block, BlockPose> targets {
    {Block::B_1x1_H, BlockPose(Block::B_1x1_H, 4.5, 0, 0)},
    {Block::B_2x1_L, BlockPose(Block::B_2x1_L, 0, 3, 0)},
    {Block::B_2x1_H, BlockPose(Block::B_2x1_H, 3, 3, 0)},
  };

  std::queue<BlockMovement> movements = generate_block_positioning_order(poses, targets);

  if (
    movements.front().start.block != Block::B_2x1_H ||
    (movements.pop(), movements.front().start.block != Block::B_2x1_L) ||
    (movements.pop(), movements.front().start.block != Block::B_1x1_H)
  ) {
    throw std::runtime_error("unexpected sequence");
  }

  // TODO: test deadlocks.
  // TODO: check exceptions.

  return true;
}

#include "../lib/planning/interpolation.hpp"
#include "../lib/planning/sequencer.hpp"
#include "../lib/planning/internal.hpp"
#include "planner.hpp"

constexpr Scalar ur5_default_homing_config_init[] = {-0.32, -0.78, -2.56, -1.63, -1.57, 3.49};

std::ostream& operator<<(std::ostream& out, const kinematics::Pose<>& pose) {
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

bool test_planner() {
  model::UR5 robot(model::UR5::Configuration(model::UR5::Configuration::Base{ur5_default_homing_config_init}));

  const BlockMovement movements[]{
    {BlockPose(Block::B_2x1_H, 0.8, 0.2, 0)},
    {BlockPose(Block::B_3x1_H, 0.68, 0.45, 0.0077)},
    {BlockPose(Block::B_1x1_H, 0.84, 0.68, 0.60)},
    {BlockPose(Block::B_2x2_U, 0.58, 0.73, 0.35)}
  };

  constexpr Time dt = 0.001;

  char filename[] = "/tmp/test_via_points_XXXXXX\0csv";
  char *temp = mktemp(filename);

  size_t len = strlen(temp);

  std::ofstream file;

  if (len == 0) {
    std::cerr << "Cannot create the csv file." << std::endl;
  } else {
    temp[len] = '.';
    std::cerr << "Plot available at " << temp << "." << std::endl;
    file = std::ofstream(temp);
  }

  for (auto&& movement : movements) {
    auto configs = planner::plan_movement(robot, movement, dt);

    for (auto&& config : configs.lazy_picking) {
      file << kinematics::direct(robot, config) << '\n';
    }
    for (auto&& config : configs.lazy_dropping) {
      file << kinematics::direct(robot, config) << '\n';
    }
  }

  return true;
}


bool test_linear_interpolator() {
  os::Position::Linear start(0,0,0);
  os::Position::Linear end(1,1,1);
  Time start_t = 0;
  Time end_t = 10;

  auto l = linear_interpolation(start, end, start_t, end_t);

  return l(start_t).isApprox(start) && l(end_t).isApprox(end);
}


bool test_movement_planning() { return false; }

