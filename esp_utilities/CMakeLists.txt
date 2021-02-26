cmake_minimum_required(VERSION 3.10)
project(esp_utilities)

# Find the dependencies of this library.
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(esp_utilities
  src/get_best_cost.cpp)

# Specify the include directories for this target.
target_include_directories(esp_utilities
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  PUBLIC
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_utilities
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_common)
