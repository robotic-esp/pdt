cmake_minimum_required(VERSION 3.10)
project(pdt_spaces)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(pdt_spaces
  src/SE3WAxisAngleBoundStateSpace.cpp
  src/SO3WAxisAngleBoundStateSpace.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_spaces
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(pdt_spaces
  PRIVATE
  pdt
  PUBLIC
  ${OMPL_LIBRARIES}
  pdt_config)
