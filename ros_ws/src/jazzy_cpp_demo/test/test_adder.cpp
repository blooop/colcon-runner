#include <gtest/gtest.h>

#include "jazzy_cpp_demo/adder.hpp"

TEST(AdderTests, AddsPositiveNumbers) {
  EXPECT_EQ(jazzy_cpp_demo::add(2, 3), 5);
  EXPECT_EQ(jazzy_cpp_demo::add(10, 0), 10);
}

TEST(AdderTests, HandlesNegatives) {
  EXPECT_EQ(jazzy_cpp_demo::add(-4, -6), -10);
  EXPECT_EQ(jazzy_cpp_demo::add(-4, 6), 2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
