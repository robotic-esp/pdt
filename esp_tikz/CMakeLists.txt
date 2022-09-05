cmake_minimum_required(VERSION 3.10)
project(esp_tikz)

# Specify the library as a target.
add_library(esp_tikz
  src/base_report.cpp
  src/cost_percentile_evolution_plotter.cpp
  src/experiment_report.cpp
  src/kpi_table.cpp
  src/mq_kpi_table.cpp
  src/initial_solution_duration_histogram_plotter.cpp
  src/initial_solution_scatter_plotter.cpp
  src/median_cost_evolution_plotter.cpp
  src/median_cumulative_cost_plotter.cpp
  src/median_cumulative_duration_plotter.cpp
  src/median_initial_solution_plotter.cpp
  src/median_initial_solution_query_plotter.cpp
  src/median_initial_solution_cost_query_plotter.cpp
  src/median_final_cost_query_plotter.cpp
  src/multiquery_report.cpp
  src/overview_plotter.cpp
  src/latex_plotter.cpp
  src/pgf_axis.cpp
  src/pgf_fillbetween.cpp
  src/pgf_plot.cpp
  src/pgf_table.cpp
  src/success_plotter.cpp
  src/success_rate_per_query_plotter.cpp
  src/tabularx.cpp
  src/tikz_draw.cpp
  src/tikz_node.cpp
  src/tikz_picture.cpp)

# Specify the include directories for this target.
target_include_directories(esp_tikz
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_tikz
  PRIVATE
  esp_ompl_tools
  PUBLIC
  esp_common
  esp_configuration
  esp_factories
  esp_statistics)
