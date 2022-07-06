cmake_minimum_required(VERSION 3.10)
project(esp_visualization)

# Find the dependencies for this library.
find_package(Pangolin REQUIRED)

# Make the path to the source directory available in code.
configure_file(src/fonts.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp @ONLY)

# Specify the library as a target.
add_library(esp_visualization
  src/base_visualizer.cpp
  src/interactive_visualizer.cpp
  src/planner_specific_data.cpp
  src/tikz_visualizer.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp)

# Specify our include directories for this target.
target_include_directories(esp_visualization
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_visualization SYSTEM
  PUBLIC
  ${Pangolin_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_visualization
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  pango_core
  pango_display
  pango_plot
  pango_windowing
  esp_common
  esp_factories
  esp_optimization_objectives
  esp_planning_contexts
  esp_tikz)
