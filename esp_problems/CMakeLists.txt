cmake_minimum_required(VERSION 2.8)
project(esp_problems)

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
add_library(esp_problems
  src/BaseExperiment.cpp
  src/CentreSquareExperiment.cpp
  src/DeadEndExperiment.cpp
  src/DividingWallExperiment.cpp
  src/DoubleEnclosureExperiment.cpp
  src/FlankingGapExperiment.cpp
  src/GoalEnclosureExperiment.cpp
  src/MultiStartGoalExperiment.cpp
  src/ObstacleFreeExperiment.cpp
  src/RandomRectanglesExperiment.cpp
  src/RandomRectanglesWithDividingWallExperiment.cpp
  src/RegularRectanglesExperiment.cpp
  src/RRTsharpResponseExperiment.cpp
  src/SpiralExperiment.cpp
  src/StartEnclosureExperiment.cpp
  src/TightlyBoundingRectangle.cpp
  src/WallGapExperiment.cpp
  )

# Link the library.
target_link_libraries(esp_problems PUBLIC
  ${OMPL_LIBRARIES}
  esp_obstacles)

# Specify the include directories exported by this library.
target_include_directories(esp_problems PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
