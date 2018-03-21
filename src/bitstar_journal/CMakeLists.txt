#These require set-seed functions (?)
add_executable(bitstar_journal_random_single
  bitstar_journal_random_single.cpp
)
target_link_libraries(bitstar_journal_random_single
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(bitstar_journal_regular
  bitstar_journal_regular.cpp
)
target_link_libraries(bitstar_journal_regular
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(bitstar_journal_double_enclosure
  bitstar_journal_double_enclosure.cpp
)
target_link_libraries(bitstar_journal_double_enclosure
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(bitstar_journal_batch_size
  bitstar_journal_batch_size.cpp
)
target_link_libraries(bitstar_journal_batch_size
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(bitstar_journal_initial_search
  bitstar_journal_initial_search_figure.cpp
)
target_link_libraries(bitstar_journal_initial_search
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)


add_executable(bitstar_journal_rrtsharp_response
  bitstar_journal_rrtsharp_response.cpp
)
target_link_libraries(bitstar_journal_rrtsharp_response
  obstacles
  experiments
  ompl_tools
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)
