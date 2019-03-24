# Executables
# add_executable(learn_binary_heap
#   learn_binary_heap.cpp
# )
# target_link_libraries(learn_binary_heap
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(test_nn
#   test_nn.cpp
# )
# target_link_libraries(test_nn
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(test_sampling
#   test_sampling.cpp
# )
# target_link_libraries(test_sampling
#   experiments
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(test_planners
#   test_planners.cpp
# )
# target_link_libraries(test_planners
#   obstacles
#   experiments
#   ompl_tools
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(test_abitstar
#   src/test_abitstar.cpp
# )
# target_link_libraries(test_abitstar
#   esp_factories
#   esp_obstacles
#   esp_planning_contexts
#   esp_utilities
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

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
  esp_utilities
)
# add_executable(test_multigoal
#   test_multigoal.cpp
# )
# target_link_libraries(test_multigoal
#   obstacles
#   experiments
#   ompl_tools
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(time_prune
#   time_prune.cpp
# )
# target_link_libraries(time_prune
#   obstacles
#   experiments
#   ompl_tools
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )

# add_executable(asrl_logo
#   asrl_logo.cpp
# )
# target_link_libraries(asrl_logo
#   obstacles
#   experiments
#   ompl_tools
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_LIBRARIES}
#   ${OMPL_LIBRARIES}
# )
