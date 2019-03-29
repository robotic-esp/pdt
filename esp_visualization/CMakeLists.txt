cmake_minimum_required(VERSION 2.8)
project(esp_visualization)

# Find Pangolin.
find_package(Pangolin REQUIRED)

# Include necessary header directories.
include_directories(
  ${Pangolin_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_visualization
  src/base_visualizer.cpp
  src/interactive_visualizer.cpp)

# Link the library.
target_link_libraries(esp_visualization
  PUBLIC # Libraries exposed in API.
  ${OMPL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  esp_common
  esp_factories
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_visualization PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
