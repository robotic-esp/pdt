cmake_minimum_required(VERSION 3.10)
project(pdt_reports)

# Specify the library as a target.
add_library(pdt_reports
  src/base_report.cpp
  src/multiquery_report.cpp
  src/single_query_report.cpp)

# Specify the include directories for this target.
target_include_directories(pdt_reports
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(pdt_reports
  PRIVATE
  pdt
  PUBLIC
  pdt_common
  pdt_config
  pdt_factories
  pdt_pgftikz
  pdt_plotters
  pdt_statistics)
