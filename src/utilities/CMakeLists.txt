cmake_minimum_required(VERSION 3.10)
project(pdt_utilities)

# Find the dependencies of this library.
set_package_properties(Eigen3 PROPERTIES
  URL "http://eigen.tuxfamily.org"
  PURPOSE "A general linear algebra library.")
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(pdt_utilities
  src/get_best_cost.cpp
  src/set_local_seed.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_utilities
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_utilities SYSTEM
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_utilities
  PRIVATE
  pdt
  PUBLIC
  ${OMPL_LIBRARIES}
  pdt_common
  pdt_config)
