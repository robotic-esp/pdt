cmake_minimum_required(VERSION 3.10)
project(pdt_factories)

# Find the dependencies of this library.
set_package_properties(Boost PROPERTIES
  URL "https://boost.org"
  PURPOSE "Provides standardized libraries, such as program options, filesystem I/O, etc.")
find_package(Boost 1.58 REQUIRED COMPONENTS system)

# Specify the library as a target.
add_library(pdt_factories
  src/context_factory.cpp
  src/planner_factory.cpp)

# Specify our include directories for this target.
target_include_directories(pdt_factories
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include)

# Specify third-party include directories as system includes to suppress warnings.
target_include_directories(pdt_factories SYSTEM
  PUBLIC
  ${OMPL_INCLUDE_DIRS})

# Specify the link targets for this target.
target_link_libraries(pdt_factories
  PRIVATE
  pdt
  PUBLIC
  Boost::system
  ${OMPL_LIBRARIES}
  pdt_common
  pdt_config
  pdt_planning_contexts
  pdt_spaces)

# Add pdt_open_rave as a link targets if requested.
if(PDT_OPEN_RAVE)
  target_link_libraries(pdt_factories
    PUBLIC
    pdt_open_rave)
endif()
