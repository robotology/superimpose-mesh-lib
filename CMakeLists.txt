cmake_minimum_required(VERSION 3.1)

project(superimpose-hand)

set(SH_VERSION_MAJOR "0")
set(SH_VERSION_MINOR "2")
set(SH_VERSION_PATCH "1")
set(SH_VERSION_DEVEL "1")
if(SH_VERSION_DEVEL)
    set(SH_VERSION "${SH_VERSION_MAJOR}.${SH_VERSION_MINOR}.${SH_VERSION_PATCH}.${SH_VERSION_DEVEL}")
else()
    set(SH_VERSION "${SH_VERSION_MAJOR}.${SH_VERSION_MINOR}.${SH_VERSION_PATCH}")
endif()


### --- FILES --- ###
set(INCLUDE_FILES)
set(SOURCE_FILES)
set(PROGRAM_FILES)
set(RESOURCE_FILES)
### --- INCLUDE & SOURCE FILES --- ###
set(INCLUDE_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SH_INCLUDE_FILES    ${INCLUDE_DIR}/Mesh.h
                        ${INCLUDE_DIR}/Model.h
                        ${INCLUDE_DIR}/Shader.h
                        ${INCLUDE_DIR}/SuperImpose.h
                        ${INCLUDE_DIR}/SICAD.h
                        ${INCLUDE_DIR}/SISkeleton.h
                        ${INCLUDE_DIR}/SuperimposerFactory.h
                        ${INCLUDE_DIR}/CADSuperimposer.h
                        ${INCLUDE_DIR}/SkeletonSuperimposer.h
                        ${INCLUDE_DIR}/ThreadControllerSHC.h)

set(SOURCE_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SH_SOURCE_FILES     ${SOURCE_DIR}/main.cpp
                        ${SOURCE_DIR}/Mesh.cpp
                        ${SOURCE_DIR}/Model.cpp
                        ${SOURCE_DIR}/Shader.cpp
                        ${SOURCE_DIR}/SuperimposerFactory.cpp
                        ${SOURCE_DIR}/CADSuperimposer.cpp
                        ${SOURCE_DIR}/SkeletonSuperimposer.cpp
                        ${SOURCE_DIR}/SICAD.cpp
                        ${SOURCE_DIR}/SISkeleton.cpp
                        ${SOURCE_DIR}/ThreadControllerSHC.cpp)

list(APPEND             INCLUDE_FILES   ${SH_INCLUDE_FILES} ${SH_INCLUDE_DEPR})
list(APPEND             SOURCE_FILES    ${SH_SOURCE_FILES}  ${SH_SOURCE_DEPR})


### --- EXTRA FILES --- ###
set(THIRFT_FILES        ${SOURCE_DIR}/idl_rfm.thrift
                        ${SOURCE_DIR}/idl_mesh.thrift)

set(SHADERS_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADERS_VERT_FILES  ${SHADERS_DIR}/shader_model.vert
                        ${SHADERS_DIR}/shader_background.vert)
set(SHADERS_FRAG_FILES  ${SHADERS_DIR}/shader_model.frag
                        ${SHADERS_DIR}/shader_model_simple.frag
                        ${SHADERS_DIR}/shader_background.frag)

set(PROGRAM_CONFIG      config/superimpose-hand_config.ini)
set(PROGRAM_APP         applications/superimpose-hand.xml.template
                        applications/superimpose-hand-dump.xml.template)
set(SIM_PROGRAM_CONFIG  config/SIM_superimpose-hand_config.ini
                        config/SIM_Gazebo_superimpose-hand_config.ini)
set(SIM_PROGRAM_APP     applications/SIM_superimpose-hand.xml
                        applications/SIM_Gazebo_superimpose-hand.xml
                        applications/SIM_Background_Apps.xml
                        applications/SIM_Gazebo_Background_Apps.xml)

set(ICUB_MESH_EXT       obj)
set(ICUB_HAND_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/icub_right_hand/${ICUB_MESH_EXT})
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

list(APPEND             PROGRAM_FILES   ${PROGRAM_CONFIG}
                                        ${PROGRAM_APP}
                                        ${SIM_PROGRAM_CONFIG}
                                        ${SIM_PROGRAM_APP})
list(APPEND             RESOURCE_FILES  ${SHADERS_VERT_FILES}
                                        ${SHADERS_FRAG_FILES}
                                        ${ICUB_HAND_FILES})


### --- INCLUDE & EXECUTABLE VARS --- ###
list(APPEND CMAKE_MODULE_PATH   "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(PkgConfig      REQUIRED)
find_package(YARP           REQUIRED)
find_package(ICUB           REQUIRED)
find_package(ICUBcontrib    REQUIRED)
find_package(OpenCV         REQUIRED)
find_package(OpenGL         REQUIRED)
find_package(GLEW           REQUIRED)
find_package(GLFW3          REQUIRED)
find_package(glm            REQUIRED)
find_package(assimp         REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH}
                                ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

include_directories(${INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${ASSIMP_INCLUDE_DIRS})


### --- PACKAGES --- ###
set(LIBS    ${YARP_LIBRARIES}
            ${ICUB_LIBRARIES}
            ctrlLib
            iKin
            ${OpenCV_LIBS}
            ${OPENGL_LIBRARIES}
            glew
            glm
            glfw
            -L${ASSIMP_LIBRARY_DIRS} ${ASSIMP_LIBRARIES})

yarp_add_idl(IDL_GEN_SOURCE ${THIRFT_FILES})

LIST(APPEND SOURCE_FILES ${IDL_GEN_SOURCE})


### --- MAIN --- ###
if(${CMAKE_GENERATOR} MATCHES "Xcode")
    add_executable(${PROJECT_NAME}                    ${INCLUDE_FILES}
                                                      ${SOURCE_FILES}
                                                      ${THIRFT_FILES}
                                                      ${PROGRAM_FILES}
                                                      ${RESOURCE_FILES})

    source_group("Thrift IDL"                   FILES ${THIRFT_FILES})
    source_group("Resource\\Shader Files"       FILES ${SHADERS_VERT_FILES} ${SHADERS_FRAG_FILES})
    source_group("Resource\\CAD Files"          FILES ${ICUB_HAND_FILES})
    source_group("YARP\\Configuration Files"    FILES ${PROGRAM_CONFIG} ${SIM_PROGRAM_CONFIG})
    source_group("YARP\\Application Files"      FILES ${PROGRAM_APP} ${SIM_PROGRAM_APP})
else()
    add_executable(${PROJECT_NAME}                    ${INCLUDE_FILES}
                                                      ${SOURCE_FILES})
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_nullptr cxx_constexpr)


### --- CUSTOM --- ###
option(IS_ICUB_SIM
       "Choose to use the iCub_SIM and the associated application files" ON)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    add_definitions(-D GLFW_MAC)
    option(IS_APPLE_RETINA
           "Select this option if the Apple machine has a Retina display" ON)
    if(IS_APPLE_RETINA)
        add_definitions(-D GLFW_RETINA)
    endif()
endif()

add_custom_command(TARGET  ${PROJECT_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory                       $<TARGET_FILE_DIR:${PROJECT_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy             ${RESOURCE_FILES}   $<TARGET_FILE_DIR:${PROJECT_NAME}>)

install(TARGETS     ${PROJECT_NAME}     DESTINATION bin)
if(IS_ICUB_SIM)
    yarp_install(FILES  ${SIM_PROGRAM_APP}      DESTINATION     ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
    yarp_install(FILES  ${SIM_PROGRAM_CONFIG}   DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
else()
    yarp_install(FILES  ${PROGRAM_APP}          DESTINATION     ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
    yarp_install(FILES  ${PROGRAM_CONFIG}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
endif()
yarp_install(FILES      ${RESOURCE_FILES}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})

icubcontrib_add_uninstall_target()
