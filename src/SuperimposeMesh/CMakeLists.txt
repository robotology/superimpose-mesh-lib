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
find_package(assimp REQUIRED QUIET)
print_dependency(assimp)

find_package(GLEW REQUIRED QUIET)
print_dependency(GLEW)

find_package(glfw3 REQUIRED CONFIG QUIET)
print_dependency(glfw3)

set(GLM_VERBOSE 1)
find_package(GLM REQUIRED QUIET)
print_dependency(GLM)

find_package(OpenCV REQUIRED QUIET)
print_dependency(OpenCV)


# Library main
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(SI::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_include_directories(${LIBRARY_TARGET_NAME}
                           PUBLIC
                             "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                             "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${LIBRARY_TARGET_NAME}
                      PUBLIC
                        assimp::assimp
                        GLEW::GLEW
                        glfw
                        glm
                        ${OpenCV_LIBS})

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
