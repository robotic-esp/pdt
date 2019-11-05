# ESP OMPL TOOLS
# Authors: Jonathan Gammell, Marlin Strub
cmake_minimum_required(VERSION 3.5)
project(esp_ompl_tools)

# Enforce out-of-source builds.
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "Please build this porject out-of-source.")
endif()

# Set the standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Append the path to find installed third-party libraries.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Build in release if no build type is specified.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Put executables in a bin directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

# Display system info.
message(STATUS "CMAKE_SYSTEM_NAME          = ${CMAKE_SYSTEM_NAME}"         )
message(STATUS "CMAKE_CXX_COMPILER_ID      = ${CMAKE_CXX_COMPILER_ID}"     )
message(STATUS "CMAKE_CXX_COMPILER_VERSION = ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Configuring for ${CMAKE_BUILD_TYPE} mode:")

# Find Threads.
find_package(Threads REQUIRED)

# Find Boost.
find_package(Boost 1.58 REQUIRED COMPONENTS program_options thread)
include_directories(${Boost_INCLUDE_DIR})

# Find Eigen.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find OpenRAVE.
find_package(OpenRAVE REQUIRED)
include_directories(${OpenRAVE_INCLUDE_DIRS})

# Find OMPL.
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Add the thirdparty projects.
set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(thirdparty/json)
include_directories(thirdparty)

# Enable additional warnings and treat warnings as errors for our own code.
set(ESP_CXX_COMPILE_FLAGS "-Wall -Wextra -Werror -Wdisabled-optimization -fdiagnostics-color=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ESP_CXX_COMPILE_FLAGS}")

# Add the subdirectories.
add_subdirectory(esp_common)
add_subdirectory(esp_configuration)
add_subdirectory(esp_experiments)
add_subdirectory(esp_factories)
add_subdirectory(esp_obstacles)
add_subdirectory(esp_open_rave)
add_subdirectory(esp_optimization_objectives)
add_subdirectory(esp_performance_loggers)
add_subdirectory(esp_planning_contexts)
add_subdirectory(esp_statistics)
add_subdirectory(esp_tikz)
add_subdirectory(esp_time)
add_subdirectory(esp_utilities)
add_subdirectory(esp_visualization)
