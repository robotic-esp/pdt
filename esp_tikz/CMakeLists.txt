cmake_minimum_required(VERSION 2.8)
project(esp_tikz)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Add the library.
add_library(esp_tikz
  src/experiment_report.cpp
  src/initial_solution_duration_pdf_picture.cpp
  src/median_cost_plotter.cpp
  src/overview_plotter.cpp
  src/latex_plotter.cpp
  src/pgf_axis.cpp
  src/pgf_fillbetween.cpp
  src/pgf_plot.cpp
  src/pgf_table.cpp
  src/success_plotter.cpp
  src/tikz_picture.cpp)

target_link_libraries(esp_tikz
  esp_common
  esp_configuration
  esp_statistics)

# Specify the include directories exported by this library.
target_include_directories(esp_tikz
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
