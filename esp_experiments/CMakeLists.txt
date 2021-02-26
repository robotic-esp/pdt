# TODO: Why it will not link when the boost libraries are not specified last is
# beyond me.
cmake_minimum_required(VERSION 3.10)
project(esp_experiments)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options thread)
include_directories(${Boost_INCLUDE_DIR})

find_package(ompl REQUIRED)

# Specify the benchmark executable target.
add_executable(benchmark
  src/benchmark.cpp)

# Specify the link targets for the benchmark target.
target_link_libraries(benchmark
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_performance_loggers
  esp_tikz
  esp_time
  esp_utilities
  ${Boost_LIBRARIES})

# Specify the benchmark_report executable target.
add_executable(benchmark_report
  src/benchmark_report.cpp)

# Specify the link targets for the benchmark_report target.
target_link_libraries(benchmark_report
  PRIVATE
  esp_ompl_tools
  PUBLIC
  esp_configuration
  esp_statistics
  esp_tikz)

# Specify the collision_detection_test executable target.
add_executable(collision_detection_test
  src/collision_detection_test)

# Specify the link targets for the collision_detection_test target.
target_link_libraries(collision_detection_test
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  ${Boost_LIBRARIES})

# Specify the visualization target.
add_executable(visualization
  src/visualization.cpp)

# Specify the link targets for the visualization target.
target_link_libraries(visualization
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${Pangolin_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_time
  esp_utilities
  esp_visualization
  ${Boost_LIBRARIES})

# Specify the clearance objective test executable target.
add_executable(objective_sanity_check
  src/objective_sanity_check.cpp)

target_link_libraries(objective_sanity_check
  PRIVATE
  esp_ompl_tools
  PUBLIC
  esp_configuration
  esp_factories
  esp_planning_contexts
  ${Boost_LIBRARIES})
