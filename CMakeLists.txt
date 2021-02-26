# ESP OMPL TOOLS
# Authors: Jonathan Gammell, Marlin Strub
cmake_minimum_required(VERSION 3.10)
project(esp_ompl_tools)

# Enforce out-of-source builds.
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "Please build this project out-of-source.")
endif()

# Build in release mode if no build type is specified.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Put executables in a bin directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

# Display compiler and build info.
message(STATUS "Configuring ESP OMPL TOOLS for ${CMAKE_BUILD_TYPE} mode.")
message(STATUS "Compiler ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# Add the thirdparty projects.
set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(thirdparty/json)

# Collect all compile flags used for compiling internal libraries. This allows
# us to specify esp_ompl_tools as a dependency of our build targets and enables
# all the flags for these targets.
add_library(esp_ompl_tools INTERFACE)
target_compile_options(esp_ompl_tools INTERFACE 
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wconversion
#  -Wsign-conversion
  )
target_compile_features(esp_ompl_tools
  INTERFACE
  cxx_std_17)

# Add the local libraries & executables.
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
