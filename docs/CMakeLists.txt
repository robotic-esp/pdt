# Find Doxygen
set_package_properties(Doxygen PROPERTIES
  URL "https://doxygen.org"
  PURPOSE "To generate API documentation.")
find_package(Doxygen
             COMPONENTS dot)

if (TARGET Doxygen::doxygen)
  message(STATUS "Configuring documentation")

  # A catch-all docs build target into which we will append sub targets
  add_custom_target(docs
    COMMENT "Generating documentation")

  # API documentation using doxygen configured within CMake as per:
  # https://cmake.org/cmake/help/latest/module/FindDoxygen.html
  include("${PROJECT_SOURCE_DIR}/docs/DoxygenSettings.cmake")
  doxygen_add_docs(docs_doxygen
    "${PROJECT_SOURCE_DIR}"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Generating API documentation with Doxygen")
  add_custom_command(TARGET docs_doxygen POST_BUILD
    COMMAND echo
    COMMAND echo "  Doxygen API documentation: file://${CMAKE_CURRENT_BINARY_DIR}/html/index.html"
    COMMAND echo)

  # Add to high-level target
  add_dependencies(docs docs_doxygen)

  if(TARGET Doxygen::dot)
    # A build target to create a project dependency graph using CMake
    set(DEPENDENCY_GRAPHVIZ_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/dot/dependency_graph")
    add_custom_target(docs_graphviz
      COMMAND "${CMAKE_COMMAND}" --log-level=NOTICE "${PROJECT_SOURCE_DIR}" --graphviz="${DEPENDENCY_GRAPHVIZ_OUTPUT}.dot"
      COMMAND Doxygen::dot "${DEPENDENCY_GRAPHVIZ_OUTPUT}.dot" -Tsvg -o "${DEPENDENCY_GRAPHVIZ_OUTPUT}.svg" -Tpng -o "${DEPENDENCY_GRAPHVIZ_OUTPUT}.png"
      WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
      COMMENT "Generating project dependency graph with CMake")
    add_custom_command(TARGET docs_graphviz POST_BUILD
      COMMAND echo
      COMMAND echo "  Project dependency graph: file://${DEPENDENCY_GRAPHVIZ_OUTPUT}.dot"
      COMMAND echo "                            file://${DEPENDENCY_GRAPHVIZ_OUTPUT}.svg"
      COMMAND echo "                            file://${DEPENDENCY_GRAPHVIZ_OUTPUT}.png"
      COMMAND echo)
    
      # Add to high-level target
    add_dependencies(docs docs_graphviz)
  endif()
else (DOXYGEN_FOUND)
  message(STATUS "NOT configuring documentation")
endif()
