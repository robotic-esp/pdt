cmake_minimum_required(VERSION 3.10)
project(esp_factories)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_factories
  src/context_factory.cpp
  src/planner_factory.cpp)

# Link the library.
target_link_libraries(esp_factories
  PUBLIC # Libraries exposed in API.
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration
  esp_open_rave
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_factories PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
