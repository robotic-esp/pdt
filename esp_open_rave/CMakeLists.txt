cmake_minimum_required(VERSION 3.10)
project(esp_open_rave)

# Append the path to find installed third-party libraries.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find the dependencies of this library.
set_package_properties(OpenRAVE PROPERTIES
  URL "http://openrave.org/"
  PURPOSE "A 3D environment to simulate robotic systems, including manipulation arms and mobile platforms.")
find_package(OpenRAVE REQUIRED)
set_package_properties(Boost PROPERTIES
  URL "https://boost.org"
  PURPOSE "Provides standardized libraries, such as program options, filesystem I/O, etc.")
find_package(Boost 1.58 REQUIRED COMPONENTS system)
set_package_properties(Threads PROPERTIES
  URL "https://en.wikipedia.org/wiki/POSIX_Threads"
  PURPOSE "A standard multithreading library.")
find_package(Threads REQUIRED)

# Specify the library as a target.
add_library(esp_open_rave
  src/open_rave_base_context.cpp
  src/open_rave_base_validity_checker.cpp
  src/open_rave_knee_goal.cpp
  src/open_rave_manipulator.cpp
  src/open_rave_manipulator_validity_checker.cpp
  src/open_rave_r3.cpp
  src/open_rave_r3xso2.cpp
  src/open_rave_r3_validity_checker.cpp
  src/open_rave_r3xso2_validity_checker.cpp
  src/open_rave_se3.cpp
  src/open_rave_se3_validity_checker.cpp
  src/SE3WAxisAngleBoundStateSpace.cpp
  src/SO3WAxisAngleBoundStateSpace.cpp)

# Specify our include directories for this target.
target_include_directories(esp_open_rave
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(esp_open_rave SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(esp_open_rave
  PRIVATE
  esp_ompl_tools
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  Threads::Threads
  PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_planning_contexts)

# Specify the gui as a target.
add_executable(open_rave_gui
  src/open_rave_gui.cpp)

# Specify the link libraries for the gui target.
target_link_libraries(open_rave_gui
  PRIVATE
  esp_ompl_tools
  PUBLIC
  ${Boost_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_open_rave)
