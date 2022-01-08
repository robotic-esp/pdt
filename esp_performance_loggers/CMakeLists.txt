cmake_minimum_required(VERSION 3.10)
project(esp_performance_loggers)

# Find the dependencies of this library.
find_package(ompl REQUIRED)

# Specify the library as a target.
add_library(esp_performance_loggers
  src/performance_loggers.cpp)

# Specify the include directories for this library.
target_include_directories(esp_performance_loggers
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this library.
target_link_libraries(esp_performance_loggers
  PRIVATE
  esp_ompl_tools
  stdc++fs
  PUBLIC
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  esp_time)
