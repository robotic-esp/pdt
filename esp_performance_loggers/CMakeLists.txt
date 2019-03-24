cmake_minimum_required(VERSION 2.8)
project(esp_performance_loggers)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Include necessary header directories.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_performance_loggers
  src/performance_loggers.cpp)

# Link the library.
target_link_libraries(esp_performance_loggers
  PRIVATE # Libraries not exposed in API.
  esp_utilities)

# Specify the include directories exported by this library.
target_include_directories(esp_performance_loggers PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
