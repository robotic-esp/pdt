cmake_minimum_required(VERSION 3.10)
project(test_esp_configuration)

# Find the dependencies of this library.
find_package(ompl REQUIRED)

# Specify the unit test as an executable target.
add_executable(test_esp_configuration
  unit_tests.cpp)

# Specify the link targets for this target.
target_link_libraries(test_esp_configuration
  PRIVATE
  doctest
  esp_ompl_tools
  esp_configuration)
  
