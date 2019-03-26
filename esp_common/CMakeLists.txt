cmake_minimum_required(VERSION 2.8)
project(esp_common)

# Include necessary header directories.
include_directories(
  ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_common INTERFACE)

target_link_libraries(esp_common INTERFACE
  nlohmann_json::nlohmann_json)

# Specify the include directories exported by this library.
target_include_directories(esp_common INTERFACE
  ${PROJECT_SOURCE_DIR}/include)
