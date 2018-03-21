#src
add_subdirectory( obstacles )
add_subdirectory( experiments )
add_subdirectory( tools )
add_subdirectory( bitstar_journal )
add_subdirectory( informed_journal )


# Executables
add_executable(learn_binary_heap
  learn_binary_heap.cpp
)
target_link_libraries(learn_binary_heap
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(test_nn
  test_nn.cpp
)
target_link_libraries(test_nn
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(test_sampling
  test_sampling.cpp
)
target_link_libraries(test_sampling
  experiments
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(test_planners
  test_planners.cpp
)
target_link_libraries(test_planners
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(test_multigoal
  test_multigoal.cpp
)
target_link_libraries(test_multigoal
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(time_prune
  time_prune.cpp
)
target_link_libraries(time_prune
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(asrl_logo
  asrl_logo.cpp
)
target_link_libraries(asrl_logo
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
