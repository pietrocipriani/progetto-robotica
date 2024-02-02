#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <vector>
#include "tester.hpp"
#include "../include/planner.hpp"

using namespace planner;

bool test_conflict_management();
bool test_movement_planning();

int main() {

  test("conflict management", test_conflict_management);
  test("conflict management", test_movement_planning);

  // TODO: planning test

  return EXIT_SUCCESS;
}

bool test_conflict_management() {
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
}
bool test_movement_planning() { return false; }

