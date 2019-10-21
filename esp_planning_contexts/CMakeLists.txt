cmake_minimum_required(VERSION 2.8)
project(esp_planning_contexts)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_planning_contexts
  src/base_context.cpp
  src/context_validity_checker.cpp
  src/context_validity_checker_gnat.cpp
  src/centre_square.cpp
  src/dividing_walls.cpp
  src/double_enclosure.cpp
  src/flanking_gap.cpp
  src/four_rooms.cpp
  src/goal_enclosure.cpp
  src/obstacle_free.cpp
  src/random_rectangles.cpp
  src/random_rectangles_multi_start_goal.cpp
  src/repeating_rectangles.cpp
  # src/spiral.cpp
  src/start_enclosure.cpp
  src/wall_gap.cpp
  )

# Link the library.
target_link_libraries(esp_planning_contexts
  PUBLIC # Libraries exposed in API.
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration
  esp_obstacles
  esp_optimization_objectives
  esp_time)

# Specify the include directories exported by this library.
target_include_directories(esp_planning_contexts PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
