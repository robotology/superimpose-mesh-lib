set(TEST_TARGET_NAME test_sicad_frame)

set(${TEST_TARGET_NAME}_SRC
        main.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})

target_link_libraries(${TEST_TARGET_NAME} SuperimposeMesh)

add_test(NAME ${TEST_TARGET_NAME}
         COMMAND ${TEST_TARGET_NAME}
         WORKING_DIRECTORY $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>)

set(${TEST_TARGET_NAME}_SHADER_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/shader_frame.vert)

set(${TEST_TARGET_NAME}_SHADER_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/shader_frame.frag)

add_custom_command(TARGET  ${TEST_TARGET_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_SHADER_VERT} $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_SHADER_FRAG} $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>)
