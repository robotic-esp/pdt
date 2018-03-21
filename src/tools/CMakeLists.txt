#Plotting library
add_library(ompl_tools
  planner_tools.cpp
  recording_tools.cpp
  plotting_tools.cpp
)

target_link_libraries(ompl_tools
  obstacles
  experiments
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
