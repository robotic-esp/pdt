cmake_minimum_required(VERSION 2.8)
project(esp_time)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

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
