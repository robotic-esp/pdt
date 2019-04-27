add_executable(planner_performance
  src/planner_performance.cpp
)
target_link_libraries(planner_performance PUBLIC
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_performance_loggers
  esp_tikz
  esp_time
  esp_utilities
)

add_executable(planner_performance_plots
  src/planner_performance_plots.cpp
)
target_link_libraries(planner_performance_plots PUBLIC
  esp_configuration
  esp_statistics
  esp_tikz
)

add_executable(planner_visualization
  src/planner_visualization.cpp
  )
target_link_libraries(planner_visualization PUBLIC
  ${Boost_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
  esp_time
  esp_utilities
  esp_visualization
)
