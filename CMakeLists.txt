# ESP OMPL TOOLS
# Authors: Jonathan Gammell, Marlin Strub
message(STATUS "CMake ${CMAKE_VERSION}")
cmake_minimum_required(VERSION 3.10)
project(esp_ompl_tools)

# Display compiler and build info.
message(STATUS "Configuring ESP OMPL TOOLS for ${CMAKE_BUILD_TYPE} mode.")
message(STATUS "Compiler ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# CMake includes
include(FeatureSummary)
# include(GNUInstallDirs)

# CMake paths
set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

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

# Let the user specify the path to the OMPL library they want to compile with.
set_package_properties(ompl PROPERTIES
   URL "https://ompl.kavrakilab.org"
   PURPOSE "The Open Motion Plannig Library (OMPL) on which ESP_OMPL_TOOLS builds.")
if(NOT ESP_OMPL_TOOLS_OMPL_DIR)
  find_package(ompl REQUIRED)
else()
  find_package(ompl REQUIRED PATHS ${ESP_OMPL_TOOLS_OMPL_DIR} NO_DEFAULT_PATH)
endif()

# Add json as a thirdparty project.
set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(thirdparty/json)

# Collect all compile flags used for compiling internal libraries. Creating
# esp_ompl_tools as an empty interface library allows us to specify esp_ompl_tools
# as a private dependency for our build targets, which enables all the flags for these
# targets without propagating them to external targets that depend on them.
add_library(esp_ompl_tools INTERFACE)
target_compile_options(esp_ompl_tools
  INTERFACE 
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wconversion
  -Wsign-conversion)
target_compile_features(esp_ompl_tools
  INTERFACE
  cxx_std_17)

# Add open-rave integration if requested. This option must be processed before
# the local libraries as some of them depend on this option.
option(ESP_OMPL_TOOLS_OPEN_RAVE "Enable OpenRAVE integration." OFF)
if(ESP_OMPL_TOOLS_OPEN_RAVE)
  add_definitions(-DESP_OMPL_TOOLS_OPEN_RAVE)
  add_subdirectory(esp_open_rave)
endif()

# Add the local libraries & executables.
add_subdirectory(docs)
add_subdirectory(esp_common)
add_subdirectory(esp_configuration)
add_subdirectory(esp_experiments)
add_subdirectory(esp_factories)
add_subdirectory(esp_obstacles)
add_subdirectory(esp_optimization_objectives)
add_subdirectory(esp_performance_loggers)
add_subdirectory(esp_planning_contexts)
add_subdirectory(esp_spaces)
add_subdirectory(esp_statistics)
add_subdirectory(esp_tikz)
add_subdirectory(esp_time)
add_subdirectory(esp_utilities)
add_subdirectory(esp_visualization)

# Add the tests if requested.
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  # Include CMake's test utilities.
  include(CTest)
  
  # Add doctest as a thirdparty project.
  add_subdirectory(thirdparty/doctest)

  # Build the tests.
  add_subdirectory(test/esp_configuration)
  add_subdirectory(test/esp_optimization_objectives)
endif()

# Helpful, but too noisy to leave on by default at the moment
# feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
