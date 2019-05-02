cmake_minimum_required(VERSION 2.8)
project(esp_utilities)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_utilities
  src/get_best_cost.cpp)

target_link_libraries(esp_utilities
  ${OMPL_LIBRARIES}
  esp_common)

# Specify the include directories exported by this library.
target_include_directories(esp_utilities
  PUBLIC
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)
