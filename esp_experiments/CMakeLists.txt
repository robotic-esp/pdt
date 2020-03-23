add_executable(benchmark
  src/benchmark.cpp
)
target_link_libraries(benchmark PUBLIC
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

add_executable(benchmark_report
  src/benchmark_report.cpp
)
target_link_libraries(benchmark_report PUBLIC
  esp_configuration
  esp_statistics
  esp_tikz
)

add_executable(collision_detection_test
  src/collision_detection_test
  )
target_link_libraries(collision_detection_test PUBLIC
  ${OMPL_LIBRARIES}
  esp_configuration
  esp_factories
  esp_planning_contexts
)

add_executable(visualization
  src/visualization.cpp
  )
target_link_libraries(visualization PUBLIC
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
