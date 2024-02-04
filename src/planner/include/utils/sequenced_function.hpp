#ifndef UTILS_SEQUENCED_FUNCTION_HPP_INCLUDED
#define UTILS_SEQUENCED_FUNCTION_HPP_INCLUDED


#include <algorithm>
#include <functional>
#include <type_traits>

/// Function restricted in domain by a lower bound.
/// @param Function The function type.
/// @param Domain The domain space type.
template<class Function, class Domain>
struct RestrictedFunction {
  /// The current function.
  ///
  Function f;

  /// The lower bound for the domain.
  ///
  Domain begin;

  RestrictedFunction() noexcept = default;

  RestrictedFunction(const Function& f, const Domain& begin)
    : f(f), begin(begin) {}
  RestrictedFunction(Function&& f, const Domain& begin)
    : f(std::move(f)), begin(begin) {}

  /// Compares this RestrictedFunction with another by domain.
  /// The two functions do not need to map the same spaces, only domain coherence is required.
  /// @param F The other function.
  /// @return `true` If @p begin is lower than @p other.begin, `false` otherwise.
  template<class F>
  bool operator<(const RestrictedFunction<F, Domain>& other) const {
    return begin < other.begin;
  }

  /// Checks if @p arg is valid for this RestrictedFunction.
  /// @param arg The argument to check.
  /// @return true if @arg is into the domain.
  bool operator<(const Domain& arg) const {
    return begin < arg;
  }

  /// Invokes the underlying function, no checks are performed.
  ///
  typename std::invoke_result_t<Function, Domain> operator()(const Domain& arg) const {
    return std::invoke(f, arg);
  }
  
  /// Invokes the underlying function, no checks are performed.
  ///
  typename std::invoke_result_t<Function, Domain> operator()(Domain&& arg) const {
    return std::invoke(f, std::move(arg));
  }
};

/// Function defined by cases in a total ordered @p Domain.
/// @note Use std::function to avoid the usage of the exact same function.
template<class Function, class Domain>
class SequencedFunction {
public:

  /// The type containing the list of functions.
  ///
  using Container = std::vector<RestrictedFunction<Function, Domain>>;

private:
  using It = typename Container::const_iterator;
  Container functions;
  
  /// Cached function to have constant time evaluation with sequential generation.
  ///
  mutable It it = functions.cbegin();

  /// Checks if the @p it is usable with the given @p arg.
  ///
  bool can_use(It it, const Domain& arg) const {
    if (it == functions.cend()) return false;

    // Function only lower-bounded. The next function could take over after a while.
    auto next = std::next(it);

    return it->begin <= arg && (next == functions.cend() || next->begin >= arg);
  }

  /// Gets the correct function and updates the cached iterator.
  /// @note Constant time in case of sequential access.
  ///       Logarithm time in the number of functions in case of random access.
  /// @note The limit for the first function is ignored.
  typename It::reference current(const Domain& arg) const {
    if (can_use(it, arg)) return *it;
    if (can_use(std::next(it), arg)) return *++it;

    // Binary search of the first function that can accept @p arg.
    it = std::lower_bound(functions.cbegin(), functions.cend(), arg);

    // If there is no function for the given argument, the first function is extended.
    if (it == functions.cend()) it = functions.cbegin();
    return *it;
  }
  
public:
  /// Creates a SequencedFunction from the list of functions.
  /// The list must be sorted and contain at least one function.
  /// @param fs The list of functions.
  SequencedFunction(Container&& fs) : functions(std::move(fs)) {
    assert(!functions.empty() && std::is_sorted(functions.begin(), functions.end()));
  }

  /// Invokes the valid underlying function.
  ///
  typename std::invoke_result_t<Function, Domain> operator()(const Domain& arg) const {
    return current(arg)(arg);
  }

  /// Invokes the valid underlying function.
  ///
  typename std::invoke_result_t<Function, Domain> operator()(Domain&& arg) const {
    return current(arg)(std::move(arg));
  }

};



#endif /* UTILS_SEQUENCED_FUNCTION_HPP_INCLUDED */
