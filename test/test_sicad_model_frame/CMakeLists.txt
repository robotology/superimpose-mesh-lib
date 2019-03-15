#===============================================================================
#
# Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TEST_TARGET_NAME test_sicad_model_frame)

set(${TEST_TARGET_NAME}_HDR
      ../common/utils.h
)

set(${TEST_TARGET_NAME}_SRC
      main.cpp
)


add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_HDR} ${${TEST_TARGET_NAME}_SRC})

target_link_libraries(${TEST_TARGET_NAME} SI::SuperimposeMesh)

target_include_directories(${TEST_TARGET_NAME}
                           PRIVATE
                             ${PROJECT_SOURCE_DIR}/test/common)

add_test(NAME ${TEST_TARGET_NAME}
         COMMAND ${TEST_TARGET_NAME}
         WORKING_DIRECTORY $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>)
