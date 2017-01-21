set(EXE_TARGET_NAME iCubLimbSuperimposer)

set(${EXE_TARGET_NAME}_SRC
        src/main.cpp
        src/CADSuperimposer.cpp
        src/SkeletonSuperimposer.cpp
        src/SuperimposerFactory.cpp
        src/ThreadControllerSHC.cpp)

set(${EXE_TARGET_NAME}_HDR
        include/iCubLimbSuperimposer/CADSuperimposer.h
        include/iCubLimbSuperimposer/SkeletonSuperimposer.h
        include/iCubLimbSuperimposer/SuperimposerFactory.h
        include/iCubLimbSuperimposer/ThreadControllerSHC.h)

set(${EXE_TARGET_NAME}_THRIFT_HDR
        thrift/idl_rfm.thrift
        thrift/idl_mesh.thrift)

set(${EXE_TARGET_NAME}_CONFIG
        config/superimpose-hand_config.ini)

set(${EXE_TARGET_NAME}_APP
        app/superimpose-hand.xml.template
        app/superimpose-hand-dump.xml.template)

set(${EXE_TARGET_NAME}_CONFIG_SIM
        config/SIM_superimpose-hand_config.ini)

set(${EXE_TARGET_NAME}_APP_SIM
        app/SIM_superimpose-hand.xml
        app/SIM_Background_Apps.xml)

# Dependencies
find_package(YARP        REQUIRED)
find_package(ICUB        REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV      REQUIRED)
find_package(OpenGL      REQUIRED)
find_package(GLEW        REQUIRED)
find_package(GLFW3       REQUIRED)

list(APPEND CMAKE_MODULE_PATH   ${YARP_MODULE_PATH}
                                ${ICUB_MODULE_PATH}
                                ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable main
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC}     ${${EXE_TARGET_NAME}_HDR}
                                  ${${EXE_TARGET_NAME}_THRIFT_SRC} ${${EXE_TARGET_NAME}_THRIFT})

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${YARP_INCLUDE_DIRS}
                                                      ${ICUB_INCLUDE_DIRS}
                                                      ${OPENGL_INCLUDE_DIR}
                                                      ${ASSIMP_INCLUDE_DIRS})

target_link_libraries(${EXE_TARGET_NAME} PRIVATE SuperImpose
                                                 ${YARP_LIBRARIES}
                                                 ${ICUB_LIBRARIES}
                                                 ctrlLib
                                                 iKin
                                                 ${OpenCV_LIBS}
                                                 ${OPENGL_LIBRARIES}
                                                 glew
                                                 glfw)

target_compile_features(${EXE_TARGET_NAME} PUBLIC cxx_nullptr cxx_constexpr)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Custom commands
if(${CMAKE_GENERATOR} MATCHES "Xcode")
    source_group("Thrift IDL"                   FILES ${${EXE_TARGET_NAME}_THRIFT_HDR} ${${EXE_TARGET_NAME}_THRIFT_SRC})
    source_group("YARP\\Configuration Files"    FILES ${${EXE_TARGET_NAME}_CONFIG} ${${EXE_TARGET_NAME}_CONFIG_SIM})
    source_group("YARP\\Application Files"      FILES ${${EXE_TARGET_NAME}_APP} ${${EXE_TARGET_NAME}_APP_SIM})
endif()

yarp_install(FILES  ${${EXE_TARGET_NAME}_APP_SIM}      DESTINATION     ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES  ${${EXE_TARGET_NAME}_CONFIG_SIM}   DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
yarp_install(FILES  ${${EXE_TARGET_NAME}_APP}          DESTINATION     ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES  ${${EXE_TARGET_NAME}_CONFIG}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})

icubcontrib_add_uninstall_target()
