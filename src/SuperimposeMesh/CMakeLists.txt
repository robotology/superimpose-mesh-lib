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
find_package(assimp REQUIRED CONFIG)
find_package(glew   REQUIRED CONFIG)
find_package(glfw3  REQUIRED CONFIG)
find_package(glm    REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

# Library main
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                         ${ASSIMP_INCLUDE_DIRS}
                                                         ${GLM_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC  ${ASSIMP_LIBRARIES}
                                                     GLEW::GLEW
                                                     glfw
                                                     ${OpenCV_LIBS}
                                             PRIVATE ${OPENGL_gl_LIBRARY})

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)
