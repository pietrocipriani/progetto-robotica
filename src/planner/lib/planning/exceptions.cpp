#include "planner.hpp"

namespace planner {



MovementGenerationRecoveringException::MovementGenerationRecoveringException(
  const std::string& what, std::queue<BlockMovement>&& movement
) : std::logic_error(what), recovered_data(std::move(movement)) {}

MovementGenerationRecoveringException::MovementGenerationRecoveringException(
  const char* what, std::queue<BlockMovement>&& movement
) : std::logic_error(what), recovered_data(std::move(movement)) {}

ConflictingPositionsException::ConflictingPositionsException(
  const std::string& what, std::queue<BlockMovement>&& movement
) : MovementGenerationRecoveringException(what, std::move(movement)) {} 

ConflictingPositionsException::ConflictingPositionsException(
  const char* what, std::queue<BlockMovement>&& movement
) : MovementGenerationRecoveringException(what, std::move(movement)) {}

DeadlockException::DeadlockException(
  const std::string& what, std::queue<BlockMovement>&& movement
) : MovementGenerationRecoveringException(what, std::move(movement)) {} 

DeadlockException::DeadlockException(
  const char* what, std::queue<BlockMovement>&& movement
) : MovementGenerationRecoveringException(what, std::move(movement)) {}





}
