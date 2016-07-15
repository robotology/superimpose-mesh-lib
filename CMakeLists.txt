cmake_minimum_required(VERSION 3.1)

project(superimpose-hand)

set(SH_VERSION_MAJOR "0")
set(SH_VERSION_MINOR "1")
set(SH_VERSION_PATCH "0")
set(SH_VERSION_DEVEL "1")
if(SH_VERSION_DEVEL)
    set(SH_VERSION "${SH_VERSION_MAJOR}.${SH_VERSION_MINOR}.${SH_VERSION_PATCH}.${SH_VERSION_DEVEL}")
else()
    set(SH_VERSION "${SH_VERSION_MAJOR}.${SH_VERSION_MINOR}.${SH_VERSION_PATCH}")
endif()

#[[                            DEPENDENCIES BEGIN                             ]]
list(APPEND CMAKE_MODULE_PATH   "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(PkgConfig      REQUIRED)
find_package(YARP           REQUIRED)
find_package(ICUB           REQUIRED)
find_package(ICUBcontrib    REQUIRED)
find_package(OpenCV         REQUIRED)
find_package(OpenGL         REQUIRED)
find_package(GLEW           REQUIRED)
pkg_search_module(GLFW      REQUIRED    glfw3)
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    pkg_search_module(glm   REQUIRED    glm)
else()
    find_package(glm        REQUIRED)
endif()
find_package(assimp         REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH}
                                ${ICUBCONTRIB_MODULE_PATH})
#[[                             DEPENDENCIES END                              ]]


set(INCLUDE_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SHADERS_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(ICUB_MESH_EXT       obj)
set(ICUB_HAND_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/icub_right_hand/${ICUB_MESH_EXT})


set(INCLUDE_FILES       ${INCLUDE_DIR}/shader.h
                        ${INCLUDE_DIR}/mesh.h
                        ${INCLUDE_DIR}/model.h)
set(SOURCE_FILES        src/main.cpp
                        src/shader.cpp
                        src/mesh.cpp
                        src/model.cpp
                        src/idl.thrift)
set(THIRFT_FILES        src/idl.thrift)

set(PROGRAM_CONFIG      config/superimpose-hand_config.ini)
set(PROGRAM_SCRIPT      scripts/superimpose-hand.xml.template)
set(SIM_PROGRAM_CONFIG  config/SIM_superimpose-hand_config.ini
                        config/SIM_Gazebo_superimpose-hand_config.ini)
set(SIM_PROGRAM_SCRIPT  scripts/SIM_superimpose-hand.xml
                        scripts/SIM_Gazebo_superimpose-hand.xml
                        scripts/SIM_Background_Apps.xml
                        scripts/SIM_Gazebo_Background_Apps.xml)

set(SHADERS_VERT_FILES  ${SHADERS_DIR}/shader_model.vert
                        ${SHADERS_DIR}/shader_background.vert)
set(SHADERS_FRAG_FILES  ${SHADERS_DIR}/shader_model.frag
                        ${SHADERS_DIR}/shader_model_simple.frag
                        ${SHADERS_DIR}/shader_background.frag)

set(ICUB_HAND_FILES     ${ICUB_HAND_DIR}/r_palm.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ail0.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ail1.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ail2.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ail3.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_indexbase.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ml0.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ml1.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ml2.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_ml3.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_palm.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_tl0.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_tl1.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_tl2.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_tl3.${ICUB_MESH_EXT}
                        ${ICUB_HAND_DIR}/r_tl4.${ICUB_MESH_EXT})


set(PROGRAM_FILES       ${PROGRAM_CONFIG}       ${PROGRAM_SCRIPT}
                        ${SIM_PROGRAM_CONFIG}   ${SIM_PROGRAM_SCRIPT})
set(RESOURCE_FILES      ${SHADERS_VERT_FILES} ${SHADERS_FRAG_FILES}
                        ${ICUB_HAND_FILES})

option(IS_ICUB_SIM
       "Choose to use the iCub_SIM and the associated application files" ON)

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

include_directories(${INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${GLEW_INCLUDE_DIRS}
                    ${GLFW_INCLUDE_DIRS}
                    ${GLM_INCLUDE_DIRS}
                    ${ASSIMP_INCLUDE_DIRS})

set(LIBS    ${YARP_LIBRARIES}
            ${ICUB_LIBRARIES}
            ctrlLib
            iKin
            ${OpenCV_LIBRARIES}
            ${OPENGL_LIBRARIES}
            ${GLEW_LIBRARIES}
            ${GLFW_LIBRARIES}
            ${ASSIMP_LIBRARIES} -L${ASSIMP_LIBRARY_DIRS})

yarp_add_idl(IDL_GEN_FILES ${THIRFT_FILES})

if(${CMAKE_GENERATOR} MATCHES "Xcode")
    add_executable(${PROJECT_NAME}  ${INCLUDE_FILES}
                                    ${SOURCE_FILES}
                                    ${PROGRAM_FILES}
                                    ${RESOURCE_FILES}
                                    ${IDL_GEN_FILES})
    source_group("Resource\\Shader Files"       FILES ${SHADERS_VERT_FILES} ${SHADERS_FRAG_FILES})
    source_group("Thrift IDL"                   FILES ${THIRFT_FILES})
    source_group("Resource\\CAD"                FILES ${ICUB_HAND_FILES})
    source_group("YARP\\Configuration Files"    FILES ${PROGRAM_CONFIG} ${SIM_PROGRAM_CONFIG})
    source_group("YARP\\Script Files"           FILES ${PROGRAM_SCRIPT} ${SIM_PROGRAM_SCRIPT})
else()
    add_executable(${PROJECT_NAME}  ${INCLUDE_FILES}
                                    ${SOURCE_FILES}
                                    ${IDL_GEN_FILES})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_definitions(-D GLFW_MAC)
endif()

add_custom_command(TARGET  ${PROJECT_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory                       $<TARGET_FILE_DIR:${PROJECT_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy             ${RESOURCE_FILES}   $<TARGET_FILE_DIR:${PROJECT_NAME}>)


target_link_libraries(${PROJECT_NAME} ${LIBS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_nullptr cxx_constexpr)


install(TARGETS     ${PROJECT_NAME}     DESTINATION bin)
if(IS_ICUB_SIM)
    yarp_install(FILES  ${SIM_PROGRAM_SCRIPT}   DESTINATION     ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
    yarp_install(FILES  ${SIM_PROGRAM_CONFIG}   DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
else()
    yarp_install(FILES  ${PROGRAM_SCRIPT}       DESTINATION     ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
    yarp_install(FILES  ${PROGRAM_CONFIG}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
endif()
yarp_install(FILES      ${RESOURCE_FILES}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})

icubcontrib_add_uninstall_target()

message("IDL_GEN_FILES: " ${IDL_GEN_FILES})
message("CMAKE_CURRENT_BINARY_DIR: " ${CMAKE_CURRENT_BINARY_DIR})
