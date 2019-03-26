cmake_minimum_required(VERSION 2.8)
project(esp_configuration)

# Get the git branch, commit hash, and status.
# See answers to https://stackoverflow.com/q/1435953/6333988
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
git_local_changes(GIT_STATUS)
configure_file(src/version.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/version.cpp @ONLY)

find_package(ompl)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_configuration
  ${OMPL_LIBRARIES}
  src/configuration.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)

# Link the library.
target_link_libraries(esp_configuration
  PUBLIC # Libraries exposed in API.
  stdc++fs
  nlohmann_json::nlohmann_json
  esp_planning_contexts
  esp_time)

# Specify the include directories exported by this library.
target_include_directories(esp_configuration PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
