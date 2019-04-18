cmake_minimum_required(VERSION 2.8)
project(esp_statistics)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_statistics
  src/performance_statistics.cpp)

target_link_libraries(esp_statistics
  )

# Specify the include directories exported by this library.
target_include_directories(esp_statistics
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
