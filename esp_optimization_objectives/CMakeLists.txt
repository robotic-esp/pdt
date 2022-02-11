cmake_minimum_required(VERSION 3.10)
project(esp_optimization_objectives)

# Find the dependencies of this library.
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# Specify the library as a target.
add_library(esp_optimization_objectives
  src/potential_field_optimization_objective.cpp
  src/max_min_clearance_optimization_objective.cpp
  src/reciprocal_clearance_optimization_objective.cpp)

# Specify our include directories for this target.
target_include_directories(esp_optimization_objectives
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_optimization_objectives SYSTEM
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_optimization_objectives
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration)

