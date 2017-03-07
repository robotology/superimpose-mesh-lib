set(LIBRARY_TARGET_NAME SuperImpose)

# List of CPP (source) library files
set(${LIBRARY_TARGET_NAME}_SRC
        src/Mesh.cpp
        src/Model.cpp
        src/Shader.cpp
        src/SICAD.cpp
        src/SISkeleton.cpp)

# List of HPP (header) library files
set(${LIBRARY_TARGET_NAME}_HDR
        include/SuperImpose/Mesh.h
        include/SuperImpose/Model.h
        include/SuperImpose/Shader.h
        include/SuperImpose/SICAD.h
        include/SuperImpose/SISkeleton.h
        include/SuperImpose/SuperImpose.h)

# Depndencies
find_package(assimp     REQUIRED)
find_package(GLEW       REQUIRED)
find_package(GLFW3      REQUIRED)
find_package(glm        REQUIRED)
find_package(OpenCV     REQUIRED)
find_package(OpenGL     REQUIRED)

# Library main
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                  PRIVATE ${GLEW_INCLUDE_DIRS}
                                                          ${OPENGL_INCLUDE_DIR}
                                                          ${ASSIMP_INCLUDE_DIRS})

target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ${OpenCV_LIBS}
                                                     ${OPENGL_LIBRARIES}
                                                     ${GLEW_LIBRARIES}
                                                     glfw
                                                     -L${ASSIMP_LIBRARY_DIRS} ${ASSIMP_LIBRARIES})

target_compile_features(${LIBRARY_TARGET_NAME} PUBLIC cxx_nullptr cxx_constexpr)

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)
