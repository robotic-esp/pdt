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
  esp_ompl_tools
  PUBLIC
  esp_common
  esp_configuration
  esp_factories
  esp_statistics
  esp_tikz)
