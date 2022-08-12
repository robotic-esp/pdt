cmake_minimum_required(VERSION 3.10)
project(test_esp_optimization_objectives)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the unit test as a target.
add_executable(test_esp_optimization_objectives
  unit_tests.cpp)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(test_esp_optimization_objectives SYSTEM
  PRIVATE
  ${EIGEN_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(test_esp_optimization_objectives
  PRIVATE
  doctest
  esp_ompl_tools
  esp_configuration
  esp_factories
  esp_optimization_objectives)

list(APPEND CMAKE_MODULE_PATH ${doctest_SOURCE_DIR}/scripts/cmake)
include(doctest)
doctest_discover_tests(test_esp_optimization_objectives)
