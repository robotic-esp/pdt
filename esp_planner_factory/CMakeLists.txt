cmake_minimum_required(VERSION 2.8)
project(esp_planner_factory)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_planner_factory
  src/planner_factory.cpp)

# Link the library with libraries not exposed in API.
target_link_libraries(esp_planner_factory PRIVATE
  nlohmann_json::nlohmann_json)

# Link the library with libraries exposed in API.
target_link_libraries(esp_planner_factory PUBLIC
  stdc++fs
  ${OMPL_LIBRARIES}
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_planner_factory PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
