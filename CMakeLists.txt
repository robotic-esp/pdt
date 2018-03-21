# A random planning world in OMPL -- J. Gammell 2014
cmake_minimum_required( VERSION 3.5 )
project(random_world)

#Disallow in-source builds
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "In-source builds are not allowed.")
endif("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

##Set the GCC compiler options (This is mainly from OMPL)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
add_definitions(-std=c++11)
if( (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX) AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -Wno-variadic-macros") #-pedantic , variadic-macros are used in OMPL
else()
  message(SEND_ERROR "Only GCC is currently supported. Try: \"unset CC; export CXX=g++-5\" and then rerunning in a cleaned build folder.")
endif()
#if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
#  message("Clang")
#  add_definitions(-W -Wall -Wextra -Wno-missing-field-initializers -Wno-unused -Wno-unused-parameter -Wno-delete-non-virtual-dtor -Wno-overloaded-virtual -Wno-unknown-pragmas -Qunused-arguments -Wno-deprecated-register -Wno-mismatched-tags)
#  # prepend optimizion flag (in case the default setting doesn't include one)
#  set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
#endif()

# Configurations
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

message("Configuring for ${CMAKE_BUILD_TYPE} mode:")

# Threads
find_package (Threads)

# Boost
find_package( Boost 1.46.0 REQUIRED COMPONENTS program_options filesystem system thread)
include_directories(${Boost_INCLUDE_DIR})

# Eigen
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
#message("Eigen: ${EIGEN3_INCLUDE_DIR}")

# OMPL
#message("OMPL: $ENV{OMPL_PREFIX}")
find_package( ompl REQUIRED )
include_directories(${OMPL_INCLUDE_DIRS})

include_directories( include )
add_subdirectory( src )
