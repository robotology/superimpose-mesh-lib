#.rst:
# Findglew
# ---------
#
# Find the glew framework.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` target ``GLEW::glew``,
# if glew has been found.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   GLEW_INCLUDE_DIRS - include directories for glew
#   GLEW_LIBRARIES - libraries to link against glew
#   GLEW_FOUND - true if glew has been found and can be used
#
# Environment Variables
# ^^^^^^^^^^^^^^^^^^^^^
#
# If the library is not found on system paths, the ``GLEW_ROOT``
# environment variable can be used to locate the lbrary.

include(StandardFindModule)

set(STANDARD_FIND_MODULE_DEBUG_GLEW FALSE)

standard_find_module(GLEW
                     glew
                     TARGET glew
)

if(NOT GLFW3_FOUND)
  return()
endif()

# Create imported target GLEW::glew
if(TARGET glew)
  # If the upstream target glew exists, make GLEW::glew an alias of it
  # Note: ALIAS of imported targets are not supported, so we define an
  # imported interface target that links in a public way to glew
  add_library(GLEW::glew INTERFACE IMPORTED)
  set_target_properties(GLEW::glew PROPERTIES INTERFACE_LINK_LIBRARIES glew)
endif()
