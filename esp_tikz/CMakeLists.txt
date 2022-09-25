cmake_minimum_required(VERSION 3.10)
project(esp_tikz)

# Specify the library as a target.
add_library(esp_tikz
  src/kpi_table.cpp
  src/mq_kpi_table.cpp
  src/pgf_axis.cpp
  src/pgf_fillbetween.cpp
  src/pgf_plot.cpp
  src/pgf_table.cpp
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
