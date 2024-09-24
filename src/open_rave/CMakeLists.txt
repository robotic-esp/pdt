cmake_minimum_required(VERSION 3.10)
project(pdt_open_rave)

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
find_package(Boost 1.58 REQUIRED COMPONENTS system thread)
set_package_properties(Threads PROPERTIES
  URL "https://en.wikipedia.org/wiki/POSIX_Threads"
  PURPOSE "A standard multithreading library.")
find_package(Threads REQUIRED)

# Specify the library as a target.
add_library(pdt_open_rave
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
  src/open_rave_se3_validity_checker.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_open_rave
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_open_rave SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_open_rave
  PRIVATE
  pdt
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  Threads::Threads
  PUBLIC
  Boost::thread
  ${OMPL_LIBRARIES}
  pdt_config
  pdt_planning_contexts)

# Specify the gui as a target.
add_executable(open_rave_gui
  src/open_rave_gui.cpp)

# Specify the link libraries for the gui target.
target_link_libraries(open_rave_gui
  PRIVATE
  pdt
  PUBLIC
  Boost::system
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  ${OMPL_LIBRARIES}
  pdt_config
  pdt_factories
  pdt_planning_contexts
  pdt_open_rave)
