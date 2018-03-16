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

#Obstacle library
add_library(obstacles
  src/obstacles/BaseObstacle.cpp
  src/obstacles/HyperrectangleObstacles.cpp
  src/obstacles/RepeatingHyperrectangleObstacles.cpp
  src/obstacles/CutoutObstacles.cpp
  src/obstacles/CsvObstacle.cpp
)

#Experiments library
add_library(experiments
  src/experiments/BaseExperiment.cpp
  src/experiments/AsrlExperiment.cpp
  src/experiments/ObstacleFreeExperiment.cpp
  src/experiments/CentreSquareExperiment.cpp
  src/experiments/DeadEndExperiment.cpp
  src/experiments/DividingWallExperiment.cpp
  src/experiments/DoubleEnclosureExperiment.cpp
  src/experiments/FlankingGapExperiment.cpp
  src/experiments/MultiStartGoalExperiment.cpp
  src/experiments/RandomRectanglesExperiment.cpp
  src/experiments/RandomRectanglesWithDividingWallExperiment.cpp
  src/experiments/RegularRectanglesExperiment.cpp
  src/experiments/SpiralExperiment.cpp
  src/experiments/TightlyBoundingRectangle.cpp
  src/experiments/WallGapExperiment.cpp
  src/experiments/RRTsharpResponseExperiment.cpp
)

#Symlink to the map directory
add_custom_command(
    TARGET experiments PRE_BUILD
    COMMAND ln -sf ${CMAKE_SOURCE_DIR}/maps ${PROJECT_BINARY_DIR}/maps
)

#Plotting library
add_library(ompl_tools
  src/tools/planner_tools.cpp
  src/tools/recording_tools.cpp
  src/tools/plotting_tools.cpp
)

target_link_libraries(experiments
  obstacles
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

target_link_libraries(ompl_tools
  obstacles
  experiments
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


# Executables
add_executable(learn_binary_heap
  src/learn_binary_heap.cpp
)
add_executable(test_nn
  src/test_nn.cpp
)
add_executable(test_sampling
  src/test_sampling.cpp
)
add_executable(test_planners
  src/test_planners.cpp
)
add_executable(test_multigoal
  src/test_multigoal.cpp
)
add_executable(time_prune
  src/time_prune.cpp
)
add_executable(asrl_logo
  src/asrl_logo.cpp
)
target_link_libraries(learn_binary_heap
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(test_nn
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(test_sampling
  experiments
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(test_planners
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(test_multigoal
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(time_prune
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(asrl_logo
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#These require set-seed functions (?)
add_executable(bitstar_journal_random_single
  src/bitstar_journal/bitstar_journal_random_single.cpp
)
add_executable(bitstar_journal_regular
  src/bitstar_journal/bitstar_journal_regular.cpp
)
add_executable(bitstar_journal_double_enclosure
  src/bitstar_journal/bitstar_journal_double_enclosure.cpp
)
add_executable(bitstar_journal_batch_size
  src/bitstar_journal/bitstar_journal_batch_size.cpp
)
add_executable(bitstar_journal_rrtsharp_response
  src/bitstar_journal/bitstar_journal_rrtsharp_response.cpp
)
target_link_libraries(bitstar_journal_random_single
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(bitstar_journal_regular
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(bitstar_journal_double_enclosure
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(bitstar_journal_batch_size
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(bitstar_journal_rrtsharp_response
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


#add_executable(informed_journal
#  src/informed_journal/informed_journal.cpp
#)
#add_executable(informed_journal_targets
#  src/informed_journal/informed_journal_targets.cpp
#)
#add_executable(informed_journal_random_multiple
#  src/informed_journal/informed_journal_random_multiple.cpp
#)
#add_executable(informed_journal_random_single
#  src/informed_journal/informed_journal_random_single.cpp
#)
#add_executable(informed_journal_regular
#  src/informed_journal/informed_journal_regular.cpp
#)
#add_executable(informed_journal_converge_rate
#  src/informed_journal/informed_journal_converge_rate.cpp
#)
#add_executable(informed_journal_prune
#  src/informed_journal/informed_journal_prune.cpp
#)
#target_link_libraries(informed_journal
#  obstacles
#  experiments
#  ompl_tools
#  ${CMAKE_THREAD_LIBS_INIT}
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_targets
#  obstacles
#  experiments
#  ompl_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_random_multiple
#  obstacles
#  experiments
#  ompl_tools
#  ${CMAKE_THREAD_LIBS_INIT}
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_random_single
#  obstacles
#  experiments
#  ompl_tools
#  ${CMAKE_THREAD_LIBS_INIT}
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_regular
#  obstacles
#  experiments
#  ompl_tools
#  ${CMAKE_THREAD_LIBS_INIT}
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_converge_rate
#  obstacles
#  experiments
#  ompl_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(informed_journal_prune
#  obstacles
#  experiments
#  ompl_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
