cmake_minimum_required(VERSION 3.10)
project(pdt_pgftikz)

# Specify the library as a target.
add_library(pdt_pgftikz
  src/define_latex_colors.cpp
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
target_include_directories(pdt_pgftikz
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(pdt_pgftikz
  PRIVATE
  pdt
  PUBLIC
  pdt_common
  pdt_config
  pdt_factories
  pdt_statistics)
