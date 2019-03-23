cmake_minimum_required(VERSION 2.8)
project(esp_configuration)

# Enable more warnings and treat warnings as errors.
add_definitions(-std=c++17 -Wall -Wextra -Weffc++ -Werror
  -Wdisabled-optimization -fdiagnostics-color=auto)

# Include necessary header directories.
include_directories(
  ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/include)

# Build the library.
add_library(esp_configuration
  src/configuration.cpp)

# Link the library.
target_link_libraries(esp_configuration
  PUBLIC # Libraries exposed in API.
  stdc++fs
  nlohmann_json::nlohmann_json
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_configuration PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
