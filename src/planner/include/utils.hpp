#ifndef UTILS_HPP_INCLUDED
#define UTILS_HPP_INCLUDED

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
#include <cassert>
#include <cmath>
#include "types.hpp"

#include "utils/math.hpp"
#include "utils/sequenced_function.hpp"
#include "utils/coordinates.hpp"
#include "utils/unlazy.hpp"


// TODO: Split into multiple files. Bigger than i thought.


namespace /* internal */ {


/**
 * Shorthand for the value type of an iterator.
 */
template<class Iterator>
using it_value_type = typename std::iterator_traits<Iterator>::value_type;

/**
 * Tuple of value_types given a list of iterators.
 */
template<class... Iterators>
using IteratorsTuple = std::tuple<it_value_type<Iterators>...>;

/**
 * Shorthand for the `const_iterator` type of an iterable.
 */
template<class Iterable>
using get_const_iterator = typename std::remove_reference_t<Iterable>::const_iterator;

/**
 * Checks if all the values in @p vals are equal to each other.
 * Checked according to `T::operator==`.
 * @param vals The list of values to check.
 * @return `true` if all the elements are the same, `false` otherwise.
 * @note An empty list is considered as every element is equal to each other.
 */
template<typename T>
constexpr bool equals(const std::initializer_list<T>& vals) {
  if (vals.size() == 0) return true;

  auto it = vals.begin();
  const T& val = *it++;

  for (; it != vals.end(); ++it) {
    if (*it != val) return false;
  }
  return true;
}

/**
 * `zip` implementation given a list of iterators.
 * @param size The number of elements to zip. Smaller of the size of each of the iterators.
 * @param iterators The iterators to zip.
 * @return A vector of tuples.
 */
template<class... Iterators>
std::vector<IteratorsTuple<Iterators...>> iterators_zip(size_t size, Iterators... iterators) {
  using Tuple = IteratorsTuple<Iterators...>;  
  
  std::vector<Tuple> result;
  result.reserve(size);

  for (size_t i = 0; i < size; ++i) {
    Tuple tuple(*iterators++...);
    result.push_back(tuple);
  }

  return result;
}



}

/**
 * Zips together @p containers like python zip, returning a std::tuple.
 * @param Iterables The types of the iterables.
 * @param ensure_same_size If the function has to throw in case of different sizes of the containers.
 * @param containers The containers.
 * @return A vector of tuples of the corresponding elements.
 */
template<class... Iterables, bool ensure_same_size = true>
std::vector<IteratorsTuple<get_const_iterator<Iterables>...>> zip(Iterables&&... containers) {
  std::initializer_list sizes{static_cast<size_t>(containers.size())...};

  if constexpr (ensure_same_size) {
    assert(equals(sizes));
  }

  // Python `zip` implementation creates a zipper of length equal to the minimum size of the iterables.
  const size_t min_size = std::min(sizes);

  return iterators_zip(min_size, containers.cbegin()...);
}


/**
 * Functor representing a composition of @p Function1 and @p Function2.
 * Just for clearance: f2(f1(...)).
 * @param Function1 The invocable type of the inner function.
 * @param Function2 The invocable type of the outer function.
 */
template<class Function1, class Function2>
class FunctionComposition {
private:
  Function1 f1;
  Function2 f2;

  /**
   * Shorthand for the return type given the @p Args to the inner function.
   */
  template<class... Args>
  using ComposedReturn = std::invoke_result_t<Function2, std::invoke_result_t<Function1, Args...>>;
public:
  /**
   * Constructs the composition functor of f1 and f2.
   * @param f1 The inner function.
   * @param f2 The outer function.
   */
  FunctionComposition(const Function1& f1, const Function2& f2) noexcept : f1(f1), f2(f2) {}

  /**
   * Constructs the composition functor of f1 and f2.
   * @param f1 The inner function.
   * @param f2 The outer function.
   */
  FunctionComposition(Function1& f1, Function2& f2) noexcept : f1(std::move(f1)), f2(std::move(f2)) {}

  /**
   * Calls the composed function with the given args.
   * @param args The arguments for the invoke.
   * @return The image of the composition, given @p args.
   */
  template<class... Args>
  ComposedReturn<Args...> operator ()(Args&&... args) {
    return std::invoke(f2, std::invoke(f1));
  }
};

/**
 * A generic identity functor.
 */
template<class T>
struct Identity {
  T operator()(T t) const { return t; }
};

/**
 * A simple implementation for the java 'Stream' concept.
 * @param Iterator The underlying iterator.
 * @param Output The type of the values produced by this stream.
 */
template<class Iterator, class Output = typename std::iterator_traits<Iterator>::value_type>
class Stream {
public:
  using Input = typename std::iterator_traits<Iterator>::value_type;
  using Function = std::function<Output(const Input&)>;
private:
  Iterator _begin, _end;
  Function transformation;

public:

  /**
   * The const iterator for this collection.
   */
  struct const_iterator {
  private:
    Function& transformation;
    Iterator current;
  public:
    using iterator_category = std::input_iterator_tag;
    using value_type = typename Function::result_t;
    using difference_type = typename std::iterator_traits<Iterator>::difference_type;
    using pointer = void;
    using reference = value_type;
    using const_reference = const value_type;

    const_iterator(const Function& transformation, Iterator current)
      : transformation(transformation), current(current) {}

    bool operator!=(const const_iterator& other) const {
      return !operator==(other);
    }

    bool operator==(const const_iterator& other) const {
      return current == other.current;
    }

    const_reference operator*() {
      return transformation(*current);
    }
    const_reference operator*() const {
      return transformation(*current);
    }

    const_iterator operator++() {
      current++;
      return *this;
    }

    const_iterator operator++(int) {
      auto prev = *this;
      current++;
      return prev;
    }

  };
  
  // Also the normal iterator is a const iterator.
  // However the elements are created "on the fly", no problem in storing non const copied values.
  using iterator = const_iterator;
  using iterator::value_type;
  using iterator::reference;
  using iterator::const_reference;
  using iterator::difference_type;
  using std::iterator_traits<Iterator>::size_type;

  Stream() = default;
  Stream(Iterator begin, Iterator end, const Function& transformation = Function()) noexcept;
  Stream(Iterator begin, Iterator end, Function&& transformation = Function()) noexcept;

  iterator begin() {
    return iterator(transformation, _begin);
  }
  const_iterator begin() const {
    return cbegin();
  }
  const_iterator cbegin() const {
    return const_iterator(transformation, _begin);
  }
  iterator end() {
    return iterator(transformation, _end);
  }
  const_iterator end() const {
    return cend();
  }
  const_iterator cend() const {
    return const_iterator(transformation, _end);
  }

  /**
   * Pipes a stream throught a transformation.
   * @param function The transformation.
   */
  template<class Function2>
  Stream<Iterator, std::invoke_result_t<Function2, Output>> operator|(Function2&& function) {
    return Stream(_begin, _end, FunctionComposition(transformation, std::forward(function)));
  }

};



template<class Integer>
struct Range {
private:
  Integer _begin, _end;
public:
  using value_type = Integer;
  using reference = Integer;
  using const_reference = Integer;
  using difference_type = Integer;
  using size_type = Integer;
  class iterator {
    Integer num;
  public:
    using iterator_category = std::input_iterator_tag;
    using value_type = Range::value_type;
    using reference = Range::reference;
    using pointer = void;
    using const_reference = Range::const_reference;
    using difference_type = Range::difference_type;
    using size_type = Range::size_type;

    iterator(Integer num = 0) : num(num) {}

    iterator& operator++() {
      ++num;
      return *this;
    }
    iterator operator++(int) {
      iterator prev = *this;
      ++num;
      return prev;
    }
    bool operator==(const iterator& other) const {
      return num == other.num;
    }
    bool operator!=(const iterator& other) const {
      return !operator==(other);
    }
    reference operator*() const {
      return num;
    }
  };
  using const_iterator = iterator;

  Range(Integer end) : _begin(0), _end(end) {}
  Range(Integer begin, Integer end) : _begin(begin), _end(end) {}

  iterator begin() const { return _begin; }
  iterator cbegin() const { return _begin; }
  iterator end() const { return _end; }
  iterator cend() const { return _end; }
};


template<class T, class Target>
constexpr bool is_quasi = std::is_same_v<std::remove_cv_t<std::remove_reference_t<T>>, Target>;

template<size_t size>
constexpr size_t so = size * (size - 1) / 2;

template<class T>
constexpr size_t dimension = T::RowsAtCompileTime;

template<>
inline constexpr size_t dimension<Quaternion> = Quaternion::Dim;

template<>
inline constexpr size_t dimension<Complex> = 1;

// Namespace to avoid the unwanted usage of these functions.
namespace quaternion_rotation_algebra {

// Quaternion algebra for rotations.
inline Quaternion operator-(const Quaternion& q) {
  return q.conjugate();
}
inline Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
  return (q2 * q1).normalized();
}
inline Quaternion& operator+=(Quaternion& q1, const Quaternion& q2) {
  return q1 = q1 + q2;
}
inline Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
  return -q2 + q1;
}
/**
 * @p this - @p other = @p result | @p other + @p result = @p this.
 * desired - effective = error | effective + error = result.
 *
 * Returns the distance between @p other and @p this.
 * @param other The starting pose.
 * @return The distance as movement.
 */
inline Quaternion& operator-=(Quaternion& q1, const Quaternion& q2) {
  return q1 = q1 - q2;
}
inline Quaternion operator*(const Quaternion& q1, const Scalar& c) {
  return pow(q1, c).normalized();
}
inline Quaternion operator*(const Scalar& c, const Quaternion& q1) {
  return q1 * c;
}
inline Quaternion& operator*=(Quaternion& q1, const Scalar& c) {
  return q1 = q1 * c;
}
inline Quaternion operator/(const Quaternion& q1, const Scalar& c) {
  return q1 * (1 / c);
}
inline Quaternion& operator/=(Quaternion& q1, const Scalar& c) {
  return q1 *= (1 / c);
}

}

#endif /* UTILS_HPP_INCLUDED */
