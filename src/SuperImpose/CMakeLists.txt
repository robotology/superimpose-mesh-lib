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

message("GLEW_INCLUDE_DIRS: " ${GLEW_INCLUDE_DIRS})
message("GLEW_LIBRARY_DIRS: " ${GLEW_LIBRARY_DIRS})
message("GLEW_LIBRARIES: " ${GLEW_LIBRARIES})

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
                                                     glm
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

# EXTRA files
set(${LIBRARY_TARGET_NAME}_SHADERS_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.vert)

set(${LIBRARY_TARGET_NAME}_SHADERS_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model_simple.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.frag)

set(${LIBRARY_TARGET_NAME}_MESH
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_palm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ail0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ail1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ail2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ail3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_indexbase.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ml0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ml1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ml2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_ml3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_palm.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_tl0.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_tl1.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_tl2.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_tl3.obj
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh/icub_right_hand/r_tl4.obj)

add_custom_command(TARGET  ${LIBRARY_TARGET_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory                              $<TARGET_FILE_DIR:${LIBRARY_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${LIBRARY_TARGET_NAME}_SHADERS_VERT} $<TARGET_FILE_DIR:${LIBRARY_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${LIBRARY_TARGET_NAME}_SHADERS_FRAG} $<TARGET_FILE_DIR:${LIBRARY_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${LIBRARY_TARGET_NAME}_MESH}         $<TARGET_FILE_DIR:${LIBRARY_TARGET_NAME}>)

install(FILES ${${LIBRARY_TARGET_NAME}_SHADERS_VERT} ${${LIBRARY_TARGET_NAME}_SHADERS_FRAG} DESTINATION "${CMAKE_INSTALL_LIBDIR}")
