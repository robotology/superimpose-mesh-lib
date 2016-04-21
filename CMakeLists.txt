cmake_minimum_required(VERSION 2.8.9)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(PROJECT_NAME        movefinger)

set(SOURCE_FILES        src/main.cpp)

set(PROGRAM_CONFIG      config/movefinger_config.ini)
set(PROGRAM_SCRIPT      scripts/movefinger.xml.template)

set(SIM_PROGRAM_CONFIG  config/SIM_movefinger_config.ini)
set(SIM_PROGRAM_SCRIPT  scripts/SIM_movefinger.xml
                        scripts/SIM_Background_Apps.xml)

project(${PROJECT_NAME})

find_package(YARP           REQUIRED)
find_package(ICUB           REQUIRED)
find_package(ICUBcontrib    REQUIRED)
find_package(OpenCV         REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

option(IS_ICUB_SIM
       "Choose to use the iCub_SIM and the associated application files" ON)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ctrlLib iKin ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
if (NOT IS_ICUB_SIM)
    yarp_install(FILES  ${PROGRAM_SCRIPT}       DESTINATION     ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
    yarp_install(FILES  ${PROGRAM_CONFIG}       DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
else (IS_ICUB_SIM)
    yarp_install(FILES  ${SIM_PROGRAM_SCRIPT}   DESTINATION     ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
    yarp_install(FILES  ${SIM_PROGRAM_CONFIG}   DESTINATION     ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
endif (NOT IS_ICUB_SIM)

icubcontrib_add_uninstall_target()
