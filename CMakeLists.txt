# Planner Developer Tools (PDT)
# Authors: Jonathan Gammell, Marlin Strub
message(STATUS "CMake ${CMAKE_VERSION}")
cmake_minimum_required(VERSION 3.10)
project(pdt)

# Build in release mode if no build type is specified.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Display compiler and build info.
message(STATUS "Configuring Planner Developer Tools (PDT) for ${CMAKE_BUILD_TYPE} mode.")
message(STATUS "Compiler ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# CMake includes
include(FeatureSummary)

# CMake paths
set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Enforce out-of-source builds.
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "Please build this project out-of-source.")
endif()

# Put executables in a bin directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

# Let the user specify the path to the OMPL library they want to compile with.
set_package_properties(ompl PROPERTIES
   URL "https://ompl.kavrakilab.org"
   PURPOSE "The Open Motion Plannig Library (OMPL) on which Planner Developer Tools (PDT) builds.")
if(NOT PDT_OMPL_DIR)
  find_package(ompl REQUIRED)
else()
  find_package(ompl REQUIRED PATHS ${PDT_OMPL_DIR} NO_DEFAULT_PATH)
endif()

# Add json as a thirdparty project.
set(JSON_BuildTests OFF CACHE INTERNAL "") # Don't build the tests for json.
set(JSON_Install OFF CACHE INTERNAL "")    # Don't install the json lib.
add_subdirectory(thirdparty/json)

# Collect all compile flags used for compiling internal libraries. Creating
# pdt as an empty interface library allows us to specify pdt
# as a private dependency for our build targets, which enables all the flags for these
# targets without propagating them to external targets that depend on them.
add_library(pdt INTERFACE)
target_compile_options(pdt
  INTERFACE 
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wconversion
  -Wsign-conversion)
target_compile_features(pdt
  INTERFACE
  cxx_std_17)

# Add open-rave integration if requested. This option must be processed
# first as some PDT libraries and/or source code depends on it.
option(PDT_OPEN_RAVE "Enable OpenRAVE integration." OFF)
if(PDT_OPEN_RAVE)
  message(STATUS "Configuring PDT for OpenRAVE support.")
  add_definitions(-DPDT_OPEN_RAVE)
  # See src/CMakeLists.txt and docs/DoxygenSettings.cmake for switches on this option.
endif()

# Add flag to disable features that require the ESP OMPL fork.
option(PDT_BASE_OMPL_ONLY "Configure PDT to use only mainline OMPL features." OFF)
if (PDT_BASE_OMPL_ONLY)
  # Disable optional parts by not setting them.
  message(STATUS "Configuring PDT to use only mainline OMPL features.")
else()
  add_definitions(-DPDT_EXTRA_SET_LOCAL_SEEDS)
  add_definitions(-DPDT_EXTRA_GET_BEST_COSTS)
  add_definitions(-DPDT_EXTRA_GOAL_SPACE)
  add_definitions(-DPDT_EXTRA_EITSTAR_PR)
endif()

# Add the documentation
add_subdirectory(docs)
# Add the libraries & executables.
add_subdirectory(src)

# Add the tests if requested.
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  message(STATUS "Configuring unit tests")
  # Include CMake's test utilities.
  include(CTest)
  # Add doctest as a thirdparty project.
  add_subdirectory(thirdparty/doctest)
  # Add the tests.
  add_subdirectory(test)
endif()

# Helpful, but too noisy to leave on by default at the moment
# feature_summary(DESCRIPTION INCLUDE_QUIET_PACKAGES WHAT ALL)
