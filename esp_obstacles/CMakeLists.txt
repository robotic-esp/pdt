cmake_minimum_required(VERSION 2.8)
project(esp_obstacles)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_obstacles
  src/base_obstacle.cpp
  src/binary_map.cpp
  src/cutout_hyperrectangles.cpp
  src/hyperrectangle.cpp
  src/repeating_hyperrectangles.cpp)

# Link the library.
target_link_libraries(esp_obstacles PUBLIC
  ${OMPL_LIBRARIES})

# Specify the include directories exported by this library.
target_include_directories(esp_obstacles PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
