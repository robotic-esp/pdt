# Settings as (and organized and ordered) per: https://doxygen.org/manual/config.html
# recognizing that some values are set by FindDoxygen: https://cmake.org/cmake/help/latest/module/FindDoxygen.html#command:doxygen_add_docs
# Preferring defaults whenever in doubt.

# Project options https://doxygen.org/manual/config.html#config_project
set(DOXYGEN_PROJECT_NAME "Planner Developer Tools (PDT)")
set(DOXYGEN_PROJECT_BRIEF "Tools for developing and testing planning algorithms in OMPL")
set(DOXYGEN_PROJECT_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/esp.png")
set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
# set(DOXYGEN_NUM_PROC_THREADS 0) # Documented as experimental

# Build options https://doxygen.nl/manual/config.html#config_build
set(DOXYGEN_EXTRACT_ALL YES)
set(DOXYGEN_EXTRACT_PRIVATE YES)
set(DOXYGEN_EXTRACT_PRIV_VIRTUAL YES)
set(DOXYGEN_EXTRACT_PACKAGE YES)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_EXTRACT_LOCAL_CLASSES YES)
set(DOXYGEN_EXTRACT_ANON_NSPACES YES)
set(DOXYGEN_CASE_SENSE_NAMES YES)
set(DOXYGEN_SORT_MEMBER_DOCS NO)
set(DOXYGEN_SORT_BRIEF_DOCS NO)
set(DOXYGEN_SORT_MEMBERS_CTORS_1ST YES)
set(DOXYGEN_SORT_BY_SCOPE_NAME YES)

# Compilation message options https://doxygen.nl/manual/config.html#config_messages
set(DOXYGEN_QUIET YES)

# Input options https://doxygen.nl/manual/config.html#config_input
set(DOXYGEN_RECURSIVE YES) # Set by FindDoxygen, but really important.
set(DOXYGEN_EXCLUDE
  "${CMAKE_BINARY_DIR}/"
  "${PROJECT_SOURCE_DIR}/docs/"
  "${PROJECT_SOURCE_DIR}/thirdparty/"
  "${PROJECT_SOURCE_DIR}/scripts/matlab/")
if(NOT PDT_OPEN_RAVE)
  set(DOXYGEN_EXCLUDE
    ${DOXYGEN_EXCLUDE}
    "${PROJECT_SOURCE_DIR}/src/open_rave/")
endif()
set(DOXYGEN_EXAMPLE_PATH "${PROJECT_SOURCE_DIR}/parameters/demo/") # This won't work, but I wish it did
set(DOXYGEN_EXAMPLE_RECURSIVE YES)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${PROJECT_SOURCE_DIR}/readme.md")

# Source browsing options https://doxygen.nl/manual/config.html#config_source_browser
set(DOXYGEN_SOURCE_BROWSER YES)
set(DOXYGEN_REFERENCED_BY_RELATION YES)
set(DOXYGEN_REFERENCES_RELATION YES)
set(DOXYGEN_REFERENCES_LINK_SOURCE NO) # Link references to the documentation not source

# Class index options https://doxygen.nl/manual/config.html#config_index

# HTML output options https://doxygen.nl/manual/config.html#config_html
set(DOXYGEN_GENERATE_HTML YES)
# Generate the header/footer/stylesheet from your current settings, and then edit them as desired:
#   doxygen -w html new_header.html new_footer.html new_stylesheet.css YourDoxygenConfigFile
# Add a link back to home in the ESP logo.
set(DOXYGEN_HTML_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/esp_header.html")
# Append to the default CSS to make the header/footer Oxford blue with white text.
set(DOXYGEN_HTML_EXTRA_STYLESHEET "${CMAKE_CURRENT_SOURCE_DIR}/esp_stylesheet.css")
set(DOXYGEN_HTML_COLORSTYLE_HUE 212)
set(DOXYGEN_HTML_COLORSTYLE_SAT 100)
set(DOXYGEN_HTML_COLORSTYLE_GAMMA 100)
set(DOXYGEN_HTML_TIMESTAMP YES)
set(DOXYGEN_EXT_LINKS_IN_WINDOW YES)
set(DOXYGEN_FORMULA_FONTSIZE 13)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_MATHJAX_EXTENSIONS "TeX/AMSmath;TeX/AMSsymbols")

### Other output options ###
# LaTeX options https://doxygen.nl/manual/config.html#config_latex
# RTF options https://doxygen.nl/manual/config.html#config_rtf
# man page options https://doxygen.nl/manual/config.html#config_man
# XML options https://doxygen.nl/manual/config.html#config_xml
# DOCBOOK options https://doxygen.nl/manual/config.html#config_docbook
# Autogen options https://doxygen.nl/manual/config.html#config_autogen
# Sqlite3 options https://doxygen.nl/manual/config.html#config_sqlite3
# Perl module options https://doxygen.nl/manual/config.html#config_perlmod
### End other output options ###

# Preprocessor options https://doxygen.nl/manual/config.html#config_preprocessor
set(DOXYGEN_MACRO_EXPANSION YES)

# External refernece options https://doxygen.nl/manual/config.html#config_external
set(DOXYGEN_GENERATE_TAGFILE "${CMAKE_CURRENT_BINARY_DIR}/html/pdt.tag")

# Dot tool options https://doxygen.nl/manual/config.html#config_dot
set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
set(DOXYGEN_TEMPLATE_RELATIONS YES)
set(DOXYGEN_CALL_GRAPH YES)
set(DOXYGEN_CALLER_GRAPH YES)
set(DOXYGEN_DOT_IMAGE_FORMAT svg)
set(DOXYGEN_DOT_INTERACTIVE_SVG YES)
set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
set(DOXYGEN_DOT_TRANSPARENT YES)
