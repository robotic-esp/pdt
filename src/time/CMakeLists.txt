cmake_minimum_required(VERSION 3.10)
project(pdt_time)

# Specify the library as a target.
add_library(pdt_time
  src/CumulativeTimer.cpp
  src/time.cpp)

# Specify the include directories for this target.
target_include_directories(pdt_time
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(pdt_time
  PRIVATE
  pdt)
