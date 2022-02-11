cmake_minimum_required(VERSION 3.10)
project(esp_planning_contexts)

# Find the dependencies of this library.
find_package(ompl REQUIRED)

# Specify this library as a target.
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
  src/narrow_passage.cpp
  src/obstacle_free.cpp
  src/random_rectangles.cpp
  src/random_rectangles_multi_start_goal.cpp
  src/real_vector_geometric_context.cpp
  src/reeds_shepp_random_rectangles.cpp
  src/reeds_shepp_validity_checker.cpp
  src/repeating_rectangles.cpp
  # src/spiral.cpp
  src/start_enclosure.cpp
  src/wall_gap.cpp)

# Specify our include directories for this target.
target_include_directories(esp_planning_contexts
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_planning_contexts SYSTEM 
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_planning_contexts
  PRIVATE
  esp_ompl_tools
  ${OpenRAVE_LIBRARIES}
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration
  esp_obstacles
  esp_optimization_objectives
  esp_time)
