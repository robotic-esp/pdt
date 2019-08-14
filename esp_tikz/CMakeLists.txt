cmake_minimum_required(VERSION 2.8)
project(esp_tikz)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Add the library.
add_library(esp_tikz
  src/cost_percentile_evolution_plotter.cpp
  src/experiment_report.cpp
  src/kpi_table.cpp
  src/initial_solution_duration_pdf_plotter.cpp
  src/initial_solution_scatter_plotter.cpp
  src/median_cost_evolution_plotter.cpp
  src/median_initial_solution_plotter.cpp
  src/overview_plotter.cpp
  src/latex_plotter.cpp
  src/pgf_axis.cpp
  src/pgf_fillbetween.cpp
  src/pgf_plot.cpp
  src/pgf_table.cpp
  src/success_plotter.cpp
  src/tabularx.cpp
  src/tikz_node.cpp
  src/tikz_picture.cpp)

target_link_libraries(esp_tikz
  esp_common
  esp_configuration
  esp_factories
  esp_statistics)

# Specify the include directories exported by this library.
target_include_directories(esp_tikz
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
