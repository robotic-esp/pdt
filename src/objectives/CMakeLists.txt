cmake_minimum_required(VERSION 3.10)
project(pdt_objectives)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(pdt_objectives
  src/potential_field_optimization_objective.cpp
  src/max_min_clearance_optimization_objective.cpp
  src/reciprocal_clearance_optimization_objective.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_objectives
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_objectives SYSTEM
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_objectives
  PRIVATE
  pdt
  PUBLIC
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES}
  pdt_common
  pdt_config)

