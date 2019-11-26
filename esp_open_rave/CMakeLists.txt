cmake_minimum_required(VERSION 2.8)
project(esp_open_rave)

# Include necessary header directories.
include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

# Add the library as an interface.
add_library(esp_open_rave
  src/open_rave_base_context.cpp
  src/open_rave_base_validity_checker.cpp
  src/open_rave_manipulator.cpp
  src/open_rave_manipulator_validity_checker.cpp
  src/open_rave_se3.cpp
  src/open_rave_se3_validity_checker.cpp)

# Link the library.
target_link_libraries(esp_open_rave
  PRIVATE
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_planning_contexts)

# Specify the include directories exported by this library.
target_include_directories(esp_open_rave PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Add the gui executable.
add_executable(open_rave_gui
  src/open_rave_gui.cpp)

target_link_libraries(open_rave_gui
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_open_rave)
