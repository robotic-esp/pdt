cmake_minimum_required(VERSION 3.10)
project(esp_factories)

# Find the dependencies of this library.
find_package(ompl REQUIRED)
find_package(Boost 1.58 REQUIRED COMPONENTS system)

# Specify the library as a target.
add_library(esp_factories
  src/context_factory.cpp
  src/planner_factory.cpp)

# Specify the include directories for this target.
target_include_directories(esp_factories PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_factories
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration
  esp_open_rave
  esp_planning_contexts)
