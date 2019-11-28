cmake_minimum_required(VERSION 2.8)
project(esp_optimization_objectives)

# Include necessary header directories.
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_optimization_objectives
  src/potential_field_optimization_objective.cpp
  src/max_min_clearance_optimization_objective.cpp
  src/reciprocal_clearance_optimization_objective.cpp
  )

# Link the library.
target_link_libraries(esp_optimization_objectives
  PUBLIC # Libraries exposed in API.
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration)

# Specify the include directories exported by this library.
target_include_directories(esp_optimization_objectives PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
