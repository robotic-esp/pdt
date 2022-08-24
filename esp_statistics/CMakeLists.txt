cmake_minimum_required(VERSION 3.10)
project(esp_statistics)

# Specify this library as a target.
add_library(esp_statistics
  src/multi_query_statistics.cpp
  src/statistics.cpp
  src/population_statistics.cpp)

# Specify the include directories for this library.
target_include_directories(esp_statistics
  PUBLIC
  ${CMAKE_SOURCE_DIR}/thirdparty
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this library.
target_link_libraries(esp_statistics
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration)
