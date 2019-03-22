cmake_minimum_required(VERSION 2.8)
project(esp_obstacles)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Find OMPL.
find_package(ompl REQUIRED)

# Include necessary header directories.
include_directories(
  ../include
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_obstacles
  src/BaseObstacle.cpp
  src/CsvObstacle.cpp
  src/CutoutObstacles.cpp
  src/HyperrectangleObstacles.cpp
  src/RepeatingHyperrectangleObstacles.cpp)

# Link the library.
target_link_libraries(esp_obstacles PUBLIC
  ${OMPL_LIBRARIES})

# Specify the include directories exported by this library.
target_include_directories(esp_obstacles PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
