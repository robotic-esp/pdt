cmake_minimum_required(VERSION 3.10)
project(pdt_loggers)

# Specify the library as a target.
add_library(pdt_loggers
  src/performance_loggers.cpp)

# Specify our include directories for this library.
target_include_directories(pdt_loggers
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_loggers SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this library.
target_link_libraries(pdt_loggers
  PRIVATE
  pdt
  stdc++fs
  PUBLIC
  ${OMPL_LIBRARIES}
  #${Boost_LIBRARIES}
  pdt_time)
