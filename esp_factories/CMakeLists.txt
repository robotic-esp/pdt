cmake_minimum_required(VERSION 3.10)
project(esp_factories)

# Find the dependencies of this library.
set_package_properties(Boost PROPERTIES
  URL "https://boost.org"
  PURPOSE "Provides standardized libraries, such as program options, filesystem I/O, etc.")
find_package(Boost 1.58 REQUIRED COMPONENTS system)

# Specify the library as a target.
add_library(esp_factories
  src/context_factory.cpp
  src/planner_factory.cpp)

# Specify our include directories for this target.
target_include_directories(esp_factories
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_factories SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_factories
  PRIVATE
  pdt
  PUBLIC
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_common
  esp_configuration
  esp_planning_contexts
  esp_spaces)

# Add esp_open_rave as a link targets if requested.
if(PDT_OPEN_RAVE)
  target_link_libraries(esp_factories
    PUBLIC
    esp_open_rave)
endif()
