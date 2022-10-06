cmake_minimum_required(VERSION 3.10)
project(esp_loggers)

# Specify the library as a target.
add_library(esp_loggers
  src/performance_loggers.cpp)

# Specify our include directories for this library.
target_include_directories(esp_loggers
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_loggers SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this library.
target_link_libraries(esp_loggers
  PRIVATE
  pdt
  stdc++fs
  PUBLIC
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  esp_time)
