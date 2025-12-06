cmake_minimum_required(VERSION 3.16)
project(jazzy_py_demo)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install the Python sources into the workspace install tree
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/jazzy_py_demo
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
  FILES_MATCHING PATTERN "*.py"
)

include(CTest)
if(NOT BUILD_TESTING)
  return()
endif()

set(pytest_report ${CMAKE_CURRENT_BINARY_DIR}/pytest.xml)
add_test(
  NAME jazzy_py_demo_pytest
  COMMAND
    ${Python3_EXECUTABLE}
    -m pytest
    -q
    --junit-xml=${pytest_report}
    ${CMAKE_CURRENT_SOURCE_DIR}/test
)
set_tests_properties(
  jazzy_py_demo_pytest
  PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
)
