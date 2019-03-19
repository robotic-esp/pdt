#Experiments library
add_library(experiments
  BaseExperiment.cpp
  ObstacleFreeExperiment.cpp
  CentreSquareExperiment.cpp
  DeadEndExperiment.cpp
  GoalEnclosureExperiment.cpp
  StartEnclosureExperiment.cpp
  DividingWallExperiment.cpp
  DoubleEnclosureExperiment.cpp
  FlankingGapExperiment.cpp
  MultiStartGoalExperiment.cpp
  RandomRectanglesExperiment.cpp
  RandomRectanglesWithDividingWallExperiment.cpp
  RegularRectanglesExperiment.cpp
  SpiralExperiment.cpp
  TightlyBoundingRectangle.cpp
  WallGapExperiment.cpp
  RRTsharpResponseExperiment.cpp
)

target_link_libraries(experiments
  obstacles
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#Symlink to the map directory
add_custom_command(
    TARGET experiments PRE_BUILD
    COMMAND ln -sf ${CMAKE_SOURCE_DIR}/maps ${PROJECT_BINARY_DIR}/maps
)
