cmake_minimum_required(VERSION 3.10)
project(pdt_visualization)

# Find the dependencies for this library.
set_package_properties(Pangolin PROPERTIES
  URL "https://github.com/stevenlovegrove/Pangolin"
  PURPOSE "A GUI for a basic visualizer.")
find_package(Pangolin REQUIRED)

# Make the path to the source directory available in code.
configure_file(src/fonts.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp @ONLY)

# Specify the library as a target.
add_library(pdt_visualization
  src/base_visualizer.cpp
  src/interactive_visualizer.cpp
  src/planner_specific_data.cpp
  src/tikz_visualizer.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_visualization
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_visualization SYSTEM
  PUBLIC
  ${Pangolin_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_visualization
  PRIVATE
  pdt
  PUBLIC
  ${OMPL_LIBRARIES}
  pango_core
  pango_display
  pango_plot
  pango_windowing
  pdt_common
  pdt_factories
  pdt_objectives
  pdt_pgftikz
  pdt_planning_contexts
  pdt_utilities)
