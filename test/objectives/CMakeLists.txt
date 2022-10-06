cmake_minimum_required(VERSION 3.10)
project(test_pdt_objectives)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the unit test as a target.
add_executable(test_pdt_objectives
  unit_tests.cpp)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(test_pdt_objectives SYSTEM
  PRIVATE
  ${EIGEN_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(test_pdt_objectives
  PRIVATE
  doctest
  pdt
  pdt_config
  pdt_factories
  pdt_objectives)

list(APPEND CMAKE_MODULE_PATH ${doctest_SOURCE_DIR}/scripts/cmake)
include(doctest)
doctest_discover_tests(test_pdt_objectives)
