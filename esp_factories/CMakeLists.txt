cmake_minimum_required(VERSION 2.8)
project(esp_factories)

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
add_library(esp_factories
  src/planner_factory.cpp)

# Link the library with libraries not exposed in API.
target_link_libraries(esp_factories PRIVATE
  nlohmann_json::nlohmann_json)

# Link the library with libraries exposed in API.
target_link_libraries(esp_factories PUBLIC
  stdc++fs
  ${OMPL_LIBRARIES}
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_factories PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
