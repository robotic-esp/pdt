cmake_minimum_required(VERSION 3.10)
project(esp_plotters)

# Specify the library as a target.
add_library(esp_plotters
  src/cost_percentile_evolution_plotter.cpp
  src/initial_solution_duration_histogram_plotter.cpp
  src/initial_solution_scatter_plotter.cpp
  src/median_cost_evolution_plotter.cpp
  src/median_cumulative_cost_plotter.cpp
  src/median_cumulative_duration_plotter.cpp
  src/median_initial_solution_plotter.cpp
  src/median_initial_solution_query_plotter.cpp
  src/median_initial_solution_cost_query_plotter.cpp
  src/median_final_cost_query_plotter.cpp
  src/overview_plotter.cpp
  src/latex_plotter.cpp
  src/success_plotter.cpp
  src/success_rate_per_query_plotter.cpp)

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
