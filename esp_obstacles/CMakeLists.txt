cmake_minimum_required(VERSION 2.8)
project(esp_obstacles)

# Include necessary header directories.
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_obstacles
  src/base_obstacle.cpp
  # src/binary_map.cpp
  # src/cutout_obstacles.cpp
  src/hyperrectangle.cpp
  # src/hyperrectangles.cpp
  # src/repeating_hyperrectangles.cpp
  )

# Link the library.
target_link_libraries(esp_obstacles
  PUBLIC # Libraries exposed in API.
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES})

# Specify the include directories exported by this library.
target_include_directories(esp_obstacles PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
