cmake_minimum_required(VERSION 2.8.9)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH   "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(superimpose-hand)

#message("\nRequired packages list:")
#message("- GLFW\n    Debian: apt\n    MacOS: brew\n    WIN: TBD")
#message("- GLM\n    Debian: apt\n    MacOS: brew\n    WIN: TBD")
#message("- GLEW\n    Debian: apt\n    MacOS: brew\n    WIN: TBD")
#message("- ASSIMP\n    Debian: apt\n    MacOS: brew\n    WIN: TBD")
#message("- SOIL\n    Debian: apt\n    MacOS: http://www.lonesock.net/soil.html or custom github/brew repos)\n    WIN: TBD\n")

find_package(PkgConfig      REQUIRED)
find_package(YARP           REQUIRED)
find_package(ICUB           REQUIRED)
find_package(ICUBcontrib    REQUIRED)
find_package(OpenCV         REQUIRED)
find_package(OPENGL         REQUIRED)
find_package(GLEW           REQUIRED)
pkg_search_module(GLFW      REQUIRED    glfw3)
find_package(GLM            REQUIRED)
find_package(ASSIMP         REQUIRED)
find_package(SOIL           REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH}
                                ${ICUBCONTRIB_MODULE_PATH})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${GLEW_INCLUDE_DIRS}
                    ${GLFW_INCLUDE_DIRS}
                    ${GLM_INCLUDE_DIRS}
                    ${ASSIMP_INCLUDE_DIRS}
                    ${SOIL_INCLUDE_DIR})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

set(SOURCE_FILES        src/main.cpp
                        src/shader.h    src/shader.cpp
                        src/mesh.h      src/mesh.cpp
                        src/model.h     src/model.cpp)

set(PROGRAM_CONFIG      config/superimpose-hand_config.ini)
set(PROGRAM_SCRIPT      scripts/superimpose-hand.xml.template)

set(SIM_PROGRAM_CONFIG  config/SIM_superimpose-hand_config.ini
                        config/SIM_Gazebo_superimpose-hand_config.ini)
set(SIM_PROGRAM_SCRIPT  scripts/SIM_superimpose-hand.xml
                        scripts/SIM_Gazebo_superimpose-hand.xml
                        scripts/SIM_Background_Apps.xml
                        scripts/SIM_Gazebo_Background_Apps.xml)

set(PROGRAM_FILES       ${PROGRAM_CONFIG}   ${SIM_PROGRAM_CONFIG}
                        ${PROGRAM_SCRIPT}   ${SIM_PROGRAM_SCRIPT})

set(SHADERS_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADERS_VERT_FILES  ${SHADERS_DIR}/shader_model.vert
                        ${SHADERS_DIR}/shader_background.vert)
set(SHADERS_FRAG_FILES  ${SHADERS_DIR}/shader_model.frag
                        ${SHADERS_DIR}/shader_simple.frag
                        ${SHADERS_DIR}/shader_background.frag)

set(IMAGE_DIR           ${CMAKE_CURRENT_SOURCE_DIR}/img)
set(IMAGE_FILES         ${IMAGE_DIR}/skeleton_leftcam_02000.jpeg)

set(ICUB_MODEL_EXT      obj)
set(ICUB_HAND_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/icub_right_hand/${ICUB_MODEL_EXT})
set(ICUB_HAND_FILES     ${ICUB_HAND_DIR}/r_palm_cad.${ICUB_MODEL_EXT})

set(RESOURCE_FILES      ${SHADERS_VERT_FILES} ${SHADERS_FRAG_FILES}
                        ${ICUB_HAND_FILES}
                        ${IMAGE_FILES})

option(IS_ICUB_SIM
       "Choose to use the iCub_SIM and the associated application files" ON)

set(LIBS    ${YARP_LIBRARIES}
            ${ICUB_LIBRARIES}
            ctrlLib
            iKin
            ${OpenCV_LIBRARIES}
            ${OPENGL_LIBRARIES}
            ${GLEW_LIBRARIES}
            ${GLFW_LIBRARIES}
            ${ASSIMP_LIBRARIES} -L${ASSIMP_LIBRARY_DIRS}
            ${SOIL_LIBRARIES})

if(${CMAKE_GENERATOR} STREQUAL "Xcode")
    add_executable(${PROJECT_NAME}  ${SOURCE_FILES}
                                    ${PROGRAM_FILES}
                                    ${RESOURCE_FILES})
    source_group("Shader Files"                 FILES ${SHADERS_VERT_FILES} ${SHADERS_FRAG_FILES})
    source_group("Resource\\CAD"              FILES ${ICUB_HAND_FILES})
    source_group("Resource\\Image"                  FILES ${IMAGE_FILES})
    source_group("Resource\\Configuration Files"    FILES ${PROGRAM_CONFIG} ${SIM_PROGRAM_CONFIG})
    source_group("Resource\\Script Files"           FILES ${PROGRAM_SCRIPT} ${SIM_PROGRAM_SCRIPT})
else()
    add_executable(${PROJECT_NAME}  ${SOURCE_FILES})
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS})

if(APPLE)
    add_definitions(-DGLFW_MAC)
endif()
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${RESOURCE_FILES} $<TARGET_FILE_DIR:${PROJECT_NAME}>)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
if(NOT IS_ICUB_SIM)
    yarp_install(FILES  ${PROGRAM_SCRIPT}       DESTINATION     ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
    yarp_install(FILES  ${PROGRAM_CONFIG}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
else(IS_ICUB_SIM)
    yarp_install(FILES  ${SIM_PROGRAM_SCRIPT}   DESTINATION     ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
    yarp_install(FILES  ${SIM_PROGRAM_CONFIG}   DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
endif()

icubcontrib_add_uninstall_target()
