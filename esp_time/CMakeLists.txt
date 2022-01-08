cmake_minimum_required(VERSION 3.10)
project(esp_time)

# Specify the library as a target.
add_library(esp_time
  src/time.cpp)

# Specify the include directories for this target.
target_include_directories(esp_time
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_time
  PRIVATE
  esp_ompl_tools)
