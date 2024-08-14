# TODO: Why it will not link when the boost libraries are not specified last is
# beyond me.
cmake_minimum_required(VERSION 3.10)
project(pdt_experiments)

set_package_properties(Boost PROPERTIES
  URL "https://boost.org"
  PURPOSE "Provides standardized libraries, such as program options, filesystem I/O, etc.")
find_package(Boost 1.58 REQUIRED COMPONENTS program_options thread)
include_directories(${Boost_INCLUDE_DIR})

# Specify the benchmark executable target.
add_executable(benchmark
  src/benchmark.cpp)

# Specify the link targets for the benchmark target.
target_link_libraries(benchmark
  PRIVATE
  pdt
  PUBLIC
  ${OMPL_LIBRARIES}
  Boost::boost
  Boost::program_options
  Boost::thread
  pdt_config
  pdt_factories
  pdt_planning_contexts
  pdt_loggers
  pdt_reports
  pdt_time
  pdt_utilities)

# Specify the benchmark_report executable target.
add_executable(benchmark_report
  src/benchmark_report.cpp)

# Specify the link targets for the benchmark_report target.
target_link_libraries(benchmark_report
  PRIVATE
  pdt
  PUBLIC
  Boost::boost
  pdt_config
  pdt_reports
  pdt_statistics)

# Specify the collision_detection_test executable target.
add_executable(collision_detection_test
  src/collision_detection_test.cpp)

# Specify the link targets for the collision_detection_test target.
target_link_libraries(collision_detection_test
  PRIVATE
  pdt
  PUBLIC
  Boost::boost
  Boost::program_options
  Boost::thread
  ${OMPL_LIBRARIES}
  pdt_config
  pdt_factories
  pdt_planning_contexts)

# Specify the visualization target.
add_executable(visualization
  src/visualization.cpp)

# Specify the link targets for the visualization target.
target_link_libraries(visualization
  PRIVATE
  pdt
  PUBLIC
  Boost::boost
  Boost::program_options
  Boost::thread
  ${Pangolin_LIBRARIES}
  ${OMPL_LIBRARIES}
  pdt_config
  pdt_factories
  pdt_planning_contexts
  pdt_time
  pdt_utilities
  pdt_visualization)

# Specify the clearance objective test executable target.
add_executable(objective_sanity_check
  src/objective_sanity_check.cpp)

target_link_libraries(objective_sanity_check
  PRIVATE
  pdt
  PUBLIC
  Boost::boost
  Boost::program_options
  Boost::thread
  pdt_config
  pdt_factories
  pdt_planning_contexts)
