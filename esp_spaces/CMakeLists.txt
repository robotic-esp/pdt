cmake_minimum_required(VERSION 3.10)
project(esp_spaces)

# Specify the library as a target.
add_library(esp_spaces
  src/SE3WAxisAngleBoundStateSpace.cpp
  src/SO3WAxisAngleBoundStateSpace.cpp)

# Specify our include directories for this target.
target_include_directories(esp_spaces
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_spaces
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration)
