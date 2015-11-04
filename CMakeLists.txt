# A random planning world in OMPL -- J. Gammell 2014
cmake_minimum_required( VERSION 2.8 )
project(random_world)

#Disallow in-source builds
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(SEND_ERROR "In-source builds are not allowed.")
endif("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

##Set the GCC compiler options
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -Wno-variadic-macros") #-pedantic , variadic-macros are used in OMPL
endif()

# Configurations
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

message("Configuring for ${CMAKE_BUILD_TYPE} mode:")

# Boost
find_package( Boost 1.46.0 REQUIRED COMPONENTS thread date_time program_options filesystem system)
include_directories(${Boost_INCLUDE_DIR})

# Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

# OMPL
message("$ENV{OMPL_PREFIX}")
find_package( OMPL REQUIRED )
include_directories(${OMPL_INCLUDE_DIRS})

include_directories( include )

#Obstacle library
add_library(obstacles
  src/obstacles/BaseObstacle.cpp
  src/obstacles/HyperrectangleObstacles.cpp
  src/obstacles/RepeatingHyperrectangleObstacles.cpp
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
  src/experiments/FlankingGapExperiment.cpp
  src/experiments/MultiStartGoalExperiment.cpp
  src/experiments/RandomRectanglesExperiment.cpp
  src/experiments/RandomRectanglesWithDividingWallExperiment.cpp
  src/experiments/RegularRectanglesExperiment.cpp
  src/experiments/SpiralExperiment.cpp
  src/experiments/TightlyBoundingRectangle.cpp
  src/experiments/WallGapExperiment.cpp
)

#Symlink to the map directory
add_custom_command(
    TARGET experiments PRE_BUILD
    COMMAND ln -sf ${CMAKE_SOURCE_DIR}/maps ${PROJECT_BINARY_DIR}/maps
)

#Plotting library
add_library(plotting_tools
  src/plotting_tools.cpp
)

target_link_libraries(experiments
  obstacles
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

target_link_libraries(plotting_tools
  obstacles
  experiments
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


# Executables
#add_executable(test_nn
#  src/test_nn.cpp
#)
#add_executable(test_sampling
#  src/test_sampling.cpp
#)
#add_executable(test_planners
#  src/test_planners.cpp
#)
#add_executable(test_multigoal
#  src/test_multigoal.cpp
#)
#add_executable(asrl_logo
#  src/asrl_logo.cpp
#)
add_executable(informed_journal
  src/informed_journal/informed_journal.cpp
)
add_executable(informed_journal_targets
  src/informed_journal/informed_journal_targets.cpp
)
add_executable(informed_journal_random_multiple
  src/informed_journal/informed_journal_random_multiple.cpp
)
add_executable(informed_journal_random_single
  src/informed_journal/informed_journal_random_single.cpp
)
add_executable(informed_journal_regular
  src/informed_journal/informed_journal_regular.cpp
)
add_executable(informed_journal_converge_rate
  src/informed_journal/informed_journal_converge_rate.cpp
)
add_executable(informed_journal_prune
  src/informed_journal/informed_journal_prune.cpp
)

#target_link_libraries(test_nn
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(test_sampling
#  experiments
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(test_planners
#  obstacles
#  experiments
#  plotting_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(test_multigoal
#  obstacles
#  experiments
#  plotting_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
#target_link_libraries(asrl_logo
#  obstacles
#  experiments
#  plotting_tools
#  ${Boost_LIBRARIES}
#  ${OMPL_LIBRARIES}
#)
target_link_libraries(informed_journal
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_targets
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_random_multiple
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_random_single
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_regular
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_converge_rate
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
target_link_libraries(informed_journal_prune
  obstacles
  experiments
  plotting_tools
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
