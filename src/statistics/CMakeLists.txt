cmake_minimum_required(VERSION 3.10)
project(pdt_statistics)

# Specify this library as a target.
add_library(pdt_statistics
  src/multiquery_statistics.cpp
  src/planning_statistics.cpp
  src/population_statistics.cpp)

# Specify the include directories for this library.
target_include_directories(pdt_statistics
  PUBLIC
  ${CMAKE_SOURCE_DIR}/thirdparty
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this library.
target_link_libraries(pdt_statistics
  PRIVATE
  pdt
  PUBLIC
  ${OMPL_LIBRARIES}
  pdt_config
  pdt_utilities)
