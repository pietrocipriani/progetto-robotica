#include <cstdlib>
#include "tester.hpp"

// Volete che usiamo cppunit?

bool false_predicate();
bool true_predicate();
bool test_direct_kinematics();


int main() {

  test("false test", false_predicate);
  test("true test", true_predicate);
  test("direct kinematics", test_direct_kinematics);

  // TODO: kinematics test

  return EXIT_SUCCESS;
}

bool false_predicate() { return false; }

bool true_predicate() { return true; }

bool test_direct_kinematics() {
  // TODO: Implementation
  return true;
}
