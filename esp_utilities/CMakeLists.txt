cmake_minimum_required(VERSION 3.10)
project(esp_utilities)

# Find the dependencies of this library.
find_package(Eigen3 REQUIRED)

# Specify the library as a target.
add_library(esp_utilities
  src/get_best_cost.cpp)

# Specify our include directories for this target.
target_include_directories(esp_utilities
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_utilities SYSTEM
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_utilities
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_common)
