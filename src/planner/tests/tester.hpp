#ifndef TESTER_HPP_INCLUDE
#define TESTER_HPP_INCLUDE

#include <concepts>
#include <iostream>
#include <ostream>
#include <string>


/**
 * Invoke `tester` and prints the result ("PASSED" or "FAILED").
 * @param name An identifier for the test.
 * @param tester The test to be performed.
 * @note If `tester` throws, the test is "FAILED".
 */
#if __cplusplus >= 202002L
template<std::predicate Tester>
#else
template<class Tester>
#endif
void test(const std::string& name, const Tester& tester) noexcept {
  // ANSI escape codes for colored + bold output.
  constexpr char RED_ANSI[] = "\x1b[1;31m";
  constexpr char GREEN_ANSI[] = "\x1b[1;32m";
  constexpr char DEFAULT_ANSI[] = "\x1b[m";

  std::cout << "test " << name << ": ";

  // defaults to failed test in case of exceptions.
  bool success = false;
  try {
    success = tester();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cout << "test " << name << ": ";
  } catch (...) {
    std::cerr << "Unknown error." << std::endl;
    std::cout << "test " << name << ": ";
  }

  if (success) {
    std::cout << GREEN_ANSI << "PASSED";
  } else {
    std::cout << RED_ANSI << "FAILED";
  }

  std::cout << DEFAULT_ANSI << std::endl;
}


#endif /* TESTER_HPP_INCLUDE */
