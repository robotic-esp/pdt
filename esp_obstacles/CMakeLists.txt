cmake_minimum_required(VERSION 3.10)
project(esp_obstacles)

# Find the dependencies of this library.
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# Specify the library as a target.
add_library(esp_obstacles
  src/base_obstacle.cpp)

# Specify our include directories for this target.
target_include_directories(esp_obstacles
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_obstacles SYSTEM
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_obstacles
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${EIGEN3_LIBRARIES}
  ${OMPL_LIBRARIES})

