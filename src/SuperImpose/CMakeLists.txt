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
find_package(ASSIMP     REQUIRED)
find_package(glew       REQUIRED)
find_package(glfw3      REQUIRED)
find_package(glm        REQUIRED)
find_package(OpenCV     REQUIRED)
find_package(OpenGL     REQUIRED)

# Library main
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                          ${GLM_INCLUDE_DIRS}
                                                          ${ASSIMP_INCLUDE_DIRS})


if(WIN32)
    set(LIBS glfw3 opengl32 glew32s SOIL assimp)
elseif(UNIX AND NOT APPLE)
    add_definitions(${OPENGL_DEFINITIONS})
    find_package(X11 REQUIRED)
    # note that the order is important for setting the libs
    # use pkg-config --libs $(pkg-config --print-requires --print-requires-private glfw3) in a terminal to confirm
    set(LIBS X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL dl pthread)
elseif(APPLE)
#    include_directories(/System/Library/Frameworks)
#    find_library(COCOA_LIBRARY Cocoa)
#    find_library(OpenGL_LIBRARY OpenGL)
#    find_library(IOKit_LIBRARY IOKit)
#    mark_as_advanced(COCOA_LIBRARY OpenGL_LIBRARY)
#    set(APPLE_LIBS ${COCOA_LIBRARY} ${IOKit_LIBRARY} ${OpenGL_LIBRARY})
#    set(APPLE_LIBS ${APPLE_LIBS} glfw3 GLEW assimp SOIL)
#    set(LIBS ${APPLE_LIBS})
    set(LIBS ${OpenGL_LIBRARY})
else()
    set(LIBS)
endif(WIN32)



target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${LIBS}
                                                    ${OpenCV_LIBS}
                                                    GLEW::glew
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
