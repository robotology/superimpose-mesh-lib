#===============================================================================
#
# Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(LIBRARY_TARGET_NAME SuperimposeMesh)


# List of CPP (source) library files
set(${LIBRARY_TARGET_NAME}_SRC
      src/Mesh.cpp
      src/Model.cpp
      src/Shader.cpp
      src/SICAD.cpp
      src/SISkeleton.cpp)

# List of HPP (header) library files
set(${LIBRARY_TARGET_NAME}_HDR
      include/SuperimposeMesh/Mesh.h
      include/SuperimposeMesh/Model.h
      include/SuperimposeMesh/Shader.h
      include/SuperimposeMesh/SICAD.h
      include/SuperimposeMesh/SISkeleton.h
      include/SuperimposeMesh/Superimpose.h)


# Depndencies
find_package(assimp REQUIRED CONFIG QUIET)
print_dependency(assimp)

find_package(GLEW REQUIRED QUIET)
print_dependency(GLEW)

find_package(glfw3 REQUIRED CONFIG QUIET)
print_dependency(glfw3)

find_package(glm REQUIRED QUIET)
print_dependency(glm)

find_package(OpenCV REQUIRED QUIET)
print_dependency(OpenCV)

find_package(OpenGL REQUIRED QUIET)
print_dependency(OpenGL)


# Workaround for flawed assimp package on Ubuntu: remove non-existing paths (like "/usr/lib/include")
if(UNIX AND assimp_FOUND AND ASSIMP_INCLUDE_DIRS)
  message(AUTHOR_WARNING "Calling ASSIMP workaround to remove non-existing paths")

  # Remove non-existing paths
  foreach(path ${ASSIMP_INCLUDE_DIRS})
    if(NOT EXISTS ${path})
      list(REMOVE_ITEM ASSIMP_INCLUDE_DIRS "${path}")
    endif()
  endforeach()

  # Check if ASSIMP_INCLUDE_DIRS contains at least one valid path
  set(VALID_ASSIMP_INCLUDE_DIRS False)
  foreach(path ${ASSIMP_INCLUDE_DIRS})
    if(EXISTS "${path}/assimp/ai_assert.h")
      set(VALID_ASSIMP_INCLUDE_DIRS True)
    endif()
  endforeach()

  # Find a valid include directory if necessary
  if(NOT ${VALID_ASSIMP_INCLUDE_DIRS})
    find_path(ASSIMP_FIXED_INCLUDE_DIR "assimp/ai_assert.h" PATHS "/usr/include")
    mark_as_advanced(ASSIMP_FIXED_INCLUDE_DIR)
    if(EXISTS ${ASSIMP_FIXED_INCLUDE_DIR})
      list(APPEND ASSIMP_INCLUDE_DIRS "${ASSIMP_FIXED_INCLUDE_DIR}")
    else()
      message(FATAL_ERROR "Could not find valid ASSIMP include directory, try to set ASSIMP_FIXED_INCLUDE_DIR to a valid path containing the ASSIMP header folder")
    endif()
  endif()

  message(STATUS "ASSIMP_INCLUDE_DIRS after workaround: " ${ASSIMP_INCLUDE_DIRS})
endif()

if(APPLE)
  message(AUTHOR_WARNING "Calling ASSIMP workaround for macOS until assimp/assimp#1517 (https://github.com/assimp/assimp/issues/1517) is not fixed.")
  message(STATUS "ASSIMP_LIBRARIES before workaround: " ${ASSIMP_LIBRARIES})

  find_library(assimp_LIBRARIES
               NAMES
               ${ASSIMP_LIBRARIES})
               set(ASSIMP_LIBRARIES ${assimp_LIBRARIES})

  message(STATUS "ASSIMP_LIBRARIES after workaround: " ${ASSIMP_LIBRARIES})
endif()

# Library main
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(SI::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_include_directories(${LIBRARY_TARGET_NAME}
                           PUBLIC
                             "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                             "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                             ${ASSIMP_INCLUDE_DIRS}
                             ${GLM_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_TARGET_NAME}
                      PUBLIC
                        ${ASSIMP_LIBRARIES}
                        GLEW::GLEW
                        glfw
                        ${OpenCV_LIBS}
                      PRIVATE
                        ${OPENGL_gl_LIBRARY})

set_target_properties(${LIBRARY_TARGET_NAME}
                      PROPERTIES
                      VERSION ${${PROJECT_NAME}_VERSION}
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)
