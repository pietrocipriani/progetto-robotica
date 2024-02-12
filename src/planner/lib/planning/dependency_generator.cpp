#include "planner.hpp"
#include <algorithm>
#include <stdexcept>
#include <unordered_set>

namespace planner {

/// Graph as adjlist.
///
using Graph = std::vector<std::vector<size_t>>;

/// @throw planner::ConflictingPositionsException If two blocks are initially too near to perform
///        safely the picking of any.
Graph generate_dependency_graph(const std::vector<BlockMovement>& blocks) {
  // TODO: optimize with a lookup bucket table.
  
  Graph graph(blocks.size());
  
  for (size_t i = 0; i < blocks.size(); ++i) {
    const BlockMovement& movement = blocks[i];

    for (size_t j = 0; j < blocks.size(); ++j) {
      if (i == j) continue;

      const BlockMovement& other = blocks[j];

      // NOTE: double check in case of asymmetric checking.
      if (movement.start.collides(other.start) && other.start.collides(movement.start)) {
        throw planner::ConflictingPositionsException(
          "Two conflicting blocks have been detected. Cannot pick any of the two.",
          {}
        );
      }

      // `i` depends on `j` if it cannot be picked or dropped.
      if (movement.target.collides(other.start) || movement.start.collides(other.start)) {
        graph[i].push_back(j);
      }
    }
  }

  return graph;
}

/// Post order DFS.
///
void dfs(const Graph& graph, std::vector<bool>& visited, std::queue<size_t>& stack, size_t current) {
  visited[current] = true;

  for (auto&& neighbour : graph[current]) {
    if (!visited[neighbour]) dfs(graph, visited, stack, neighbour);
  }
  
  stack.push(current);
}

/// Topological (reversed) order of a graph.
/// @param graph The directed graph.
/// @return A list of ordered indexes in order of dependencies.
/// @note The resulting list is reversed. The first one is the independent one.
/// @note The graph should be acyclic. With cyclic graphs the result is undefined.
///       However non cyclic dependencies are respected.
std::queue<size_t> top_sort(const Graph& graph) {
  std::vector<bool> visited(graph.size(), false);
  
  std::queue<size_t> result;

  for (size_t i = 0; i < graph.size(); ++i) {
    if (!visited[i]) dfs(graph, visited, result, i);
  }

  return result;
}


/// Generates dependencies between the positioning of the blocks.
/// Some blocks could be into the target position of another block.
/// @param blocks The initial positions of the recognized blocks. 
/// @param targets The desired block target positions.
/// @return A queue of `BlockMovement` with the order of movements in order to avoid conflicts.
/// @throw std::invalid_argument If @p blocks contains more than one block of each type.
/// @throw planner::ConflictingPositionsException If two blocks are initially too near to perform
///        safely the picking of any.
/// @throw planner::DeadlockException If it is impossible to perform movements due to circular dependencies.
// TODO: Avoid deadlocks searching for auxiliary targets, or return non circular dependencies only.
std::queue<BlockMovement> generate_block_positioning_order(
  const std::vector<BlockPose>& blocks,
  const std::unordered_map<Block, BlockPose>& targets
) {
  std::queue<BlockMovement> result;

  // TODO: Just index the blocks and use a vector.
  std::unordered_set<Block> already_existing_blocks;
  already_existing_blocks.reserve(blocks.size());

  // The list of movements.
  std::vector<BlockMovement> movements;
  movements.reserve(blocks.size());

  for (auto&& b : blocks) {
    // Checks if the block is already present.
    auto [pos, inserted] = already_existing_blocks.insert(b.block);

    if (!inserted) {
      // TODO: print block.
      throw std::invalid_argument("There is more than one block [x].");
    }

    movements.emplace_back(b, targets.at(b.block));
  }

  const Graph dependency_graph = generate_dependency_graph(movements);

  // First value is the independent one. Or it should be without cycles.
  std::queue<size_t> order = top_sort(dependency_graph);

  std::vector<bool> removed(blocks.size(), false);

  while (!order.empty()) {
    const size_t block = order.front();
    
    order.pop();

    const auto& dependencies = dependency_graph[block];

    bool cyclic = std::any_of(dependencies.begin(), dependencies.end(), [&](size_t i) {return !removed[i];});

    removed[block] = true;

    if (cyclic) {
      throw DeadlockException("A deadlock has been detected.", std::move(result));

      // NOTE: a possible implementation to avoid deadlocks.
      // BlockPose free = get_auxiliary_block_pose();
      //
      // result.push(BlockMovement(movements[block].start, free));
      // movements[block].start = free;
      //
      // order.push(block);
    } else {
      result.push(movements[block]);
    }
  }

  return result;
}


}
