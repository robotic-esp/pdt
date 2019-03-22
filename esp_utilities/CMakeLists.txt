cmake_minimum_required(VERSION 2.8)
project(esp_utilities)

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
add_library(esp_utilities
  src/planner_tools.cpp
  src/plotting_tools.cpp
  src/recording_tools.cpp)

# Link the library with libraries exposed in API.
target_link_libraries(esp_utilities PUBLIC
  ${OMPL_LIBRARIES}
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_utilities PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
