# ESP OMPL TOOLS
# Authors: Jonathan Gammell, Marlin Strub
cmake_minimum_required(VERSION 3.5)
project(esp_ompl_tools)

# Enforce out-of-source builds.
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "Please build this porject out-of-source.")
endif()

# Set standard, enable warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -pedantic-errors -fdiagnostics-color=auto)

# Build in release if no build type is specified.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Put executables in bin directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

# Display system info.
message(STATUS "CMAKE_SYSTEM_NAME          = ${CMAKE_SYSTEM_NAME}"         )
message(STATUS "CMAKE_CXX_COMPILER_ID      = ${CMAKE_CXX_COMPILER_ID}"     )
message(STATUS "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Configuring for ${CMAKE_BUILD_TYPE} mode:")

# TODO: Put this in the correct CMake project.
# Find Threads.
find_package (Threads)

# TODO: Put this in the correct CMake project.
# Find Boost.
find_package( Boost 1.46.0 REQUIRED COMPONENTS program_options filesystem system thread)
include_directories(${Boost_INCLUDE_DIR})

# TODO: Put this in the correct CMake project.
# Find Eigen.
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
#message("Eigen: ${EIGEN3_INCLUDE_DIR}")

# TODO: Put this in the correct CMake project.
# Find OMPL.
find_package( ompl REQUIRED )
include_directories(${OMPL_INCLUDE_DIRS})

# Add the thirdparty projects.
set(JSON_BuildTests OFF CACHE INTERNAL "") # No need to build the tests.
set(JSON_Install OFF CACHE INTERNAL "")    # No need to install the json lib.
add_subdirectory(thirdparty/json)

# Add the subdirectories.
add_subdirectory(esp_planner_factory)
add_subdirectory(esp_planning_contexts)
add_subdirectory(esp_experiments)
add_subdirectory(esp_obstacles)
add_subdirectory(esp_utilities)
