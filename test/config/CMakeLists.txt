cmake_minimum_required(VERSION 3.10)
project(test_pdt_config)

# Specify the unit test as an executable target.
add_executable(test_pdt_config
  unit_tests.cpp)

# Specify the link targets for this target.
target_link_libraries(test_pdt_config
  PRIVATE
  doctest
  pdt
  pdt_config)
  
list(APPEND CMAKE_MODULE_PATH ${doctest_SOURCE_DIR}/scripts/cmake)
include(doctest)
doctest_discover_tests(test_pdt_config)
