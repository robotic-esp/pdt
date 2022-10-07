cmake_minimum_required(VERSION 3.10)
project(pdt_planning_contexts)

# Specify this library as a target.
add_library(pdt_planning_contexts
  src/base_context.cpp
  src/context_validity_checker.cpp
  src/context_validity_checker_gnat.cpp
  src/center_square.cpp
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
target_include_directories(pdt_planning_contexts
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_planning_contexts SYSTEM 
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_planning_contexts
  PRIVATE
  pdt
  ${OpenRAVE_LIBRARIES}
  PUBLIC
  ${OMPL_LIBRARIES}
  pdt_common
  pdt_config
  pdt_objectives
  pdt_obstacles
  pdt_time)
