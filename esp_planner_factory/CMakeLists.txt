cmake_minimum_required(VERSION 2.8)
project(esp_planner_factory)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++14 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Include local header files.
include_directories(
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_planner_factory
  src/planner_factory.cpp)

# Link the library.
target_link_libraries(esp_planner_factory
  # Libraries to be linked go here.
  )

# Specify the include directories exported by this library.
target_include_directories(esp_planner_factory PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
