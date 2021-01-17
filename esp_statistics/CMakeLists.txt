cmake_minimum_required(VERSION 3.10)
project(esp_statistics)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_statistics
  src/statistics.cpp)

target_link_libraries(esp_statistics
  esp_configuration)

# Specify the include directories exported by this library.
target_include_directories(esp_statistics
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
