cmake_minimum_required(VERSION 3.10)
project(pdt_obstacles)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(pdt_obstacles
  src/base_obstacle.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_obstacles
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_obstacles SYSTEM
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_obstacles
  PRIVATE
  pdt
  PUBLIC
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES})

