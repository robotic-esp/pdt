cmake_minimum_required(VERSION 3.10)
project(esp_performance_loggers)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_performance_loggers
  src/performance_loggers.cpp)

# Link the library.
target_link_libraries(esp_performance_loggers
  PRIVATE # Libraries not exposed in API.
  stdc++fs
  PUBLIC # Libraries exposed in API.
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  esp_time)

# Specify the include directories exported by this library.
target_include_directories(esp_performance_loggers PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
