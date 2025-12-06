cmake_minimum_required(VERSION 3.16)
project(jazzy_cpp_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(CTest)
if(NOT BUILD_TESTING)
  return()
endif()

find_package(GTest REQUIRED)

add_library(jazzy_cpp_demo_lib SHARED src/adder.cpp)
target_include_directories(jazzy_cpp_demo_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(sum_example src/main.cpp)
target_link_libraries(sum_example PRIVATE jazzy_cpp_demo_lib)

enable_testing()
include(GoogleTest)
add_executable(adder_test test/test_adder.cpp)
target_link_libraries(adder_test PRIVATE jazzy_cpp_demo_lib GTest::gtest_main)
gtest_discover_tests(adder_test)

install(TARGETS jazzy_cpp_demo_lib sum_example adder_test
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
