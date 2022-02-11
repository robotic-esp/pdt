cmake_minimum_required(VERSION 3.10)
project(esp_common)

# Specify the library as a target.
add_library(esp_common INTERFACE)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_common SYSTEM
  INTERFACE
  ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
  ${PROJECT_SOURCE_DIR}/include)

# Specify the link targets for this target.
target_link_libraries(esp_common
  INTERFACE
  nlohmann_json::nlohmann_json)

# Specify the compile features for this target.
target_compile_features(esp_common
  INTERFACE
  cxx_std_17)

