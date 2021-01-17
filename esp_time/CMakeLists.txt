cmake_minimum_required(VERSION 3.10)
project(esp_time)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_time
  src/time.cpp)

# Specify the include directories exported by this library.
target_include_directories(esp_time
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
