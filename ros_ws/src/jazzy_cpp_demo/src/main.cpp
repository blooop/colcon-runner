#include <iostream>

#include "jazzy_cpp_demo/adder.hpp"

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  const int lhs = 2;
  const int rhs = 3;
  std::cout << "Sum: " << jazzy_cpp_demo::add(lhs, rhs) << std::endl;
  return 0;
}
