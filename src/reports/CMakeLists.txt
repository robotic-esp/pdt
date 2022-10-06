cmake_minimum_required(VERSION 3.10)
project(esp_reports)

# Specify the library as a target.
add_library(esp_reports
  src/base_report.cpp
  src/multiquery_report.cpp
  src/single_query_report.cpp)

# Specify the include directories for this target.
target_include_directories(esp_reports
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_reports
  PRIVATE
  pdt
  PUBLIC
  esp_common
  esp_config
  esp_factories
  esp_pgftikz
  esp_plotters
  esp_statistics)
