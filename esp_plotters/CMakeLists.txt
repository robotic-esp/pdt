cmake_minimum_required(VERSION 3.10)
project(esp_plotters)

# Specify the library as a target.
add_library(esp_plotters
  src/latex_plotter.cpp
  src/median_cost_at_first_vs_query_line_plotter.cpp
  src/median_cost_at_last_vs_query_line_plotter.cpp
  src/median_summed_cost_at_time_vs_query_line_plotter.cpp
  src/median_summed_time_at_first_vs_query_line_plotter.cpp
  src/median_time_at_first_vs_query_line_plotter.cpp
  src/overview_plotter.cpp
  src/query_cost_at_first_vs_time_at_first_scatter_plotter.cpp
  src/query_median_cost_at_first_vs_median_time_at_first_point_plotter.cpp
  src/query_median_cost_vs_time_line_plotter.cpp
  src/query_percentile_cost_vs_time_line_plotter.cpp
  src/query_success_vs_time_line_plotter.cpp
  src/query_time_at_first_histogram_plotter.cpp
  src/success_at_time_vs_query_line_plotter.cpp)

# Specify the include directories for this target.
target_include_directories(esp_plotters
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_plotters
  PRIVATE
  esp_ompl_tools
  PUBLIC
  esp_common
  esp_configuration
  esp_factories
  esp_statistics
  esp_tikz)
