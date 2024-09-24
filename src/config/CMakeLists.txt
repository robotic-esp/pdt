cmake_minimum_required(VERSION 3.10)
project(pdt_config)

# Get the git branch, commit hash, and status.
# See answers to https://stackoverflow.com/q/1435953/6333988
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
git_local_changes(GIT_STATUS)
configure_file(src/version.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/version.cpp @ONLY)

# Make the path to the source directory available in code.
configure_file(src/directory.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/directory.cpp @ONLY)

# Find the dependencies of this library.
set_package_properties(Boost PROPERTIES
  URL "https://boost.org"
  PURPOSE "Provides standardized libraries, such as program options, filesystem I/O, etc.")
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

# Specify the library as a target.
add_library(pdt_config
  STATIC
  src/configuration.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/directory.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_config
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_config SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_config
  PRIVATE
  Boost::program_options
  pdt
  PUBLIC
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  stdc++fs)


