set(TEST_TARGET_NAME test_scissors_background)

set(${TEST_TARGET_NAME}_SRC
        main.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_SRC})

target_link_libraries(${TEST_TARGET_NAME} SuperImpose)

add_test(NAME ${TEST_TARGET_NAME} COMMAND ${TEST_TARGET_NAME})

set(${TEST_TARGET_NAME}_SHADER_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_model.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_background.vert)

set(${TEST_TARGET_NAME}_SHADER_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_model.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_background.frag)

set(${TEST_TARGET_NAME}_MESH
        ${CMAKE_CURRENT_SOURCE_DIR}/Space_Invader.obj)

set(${TEST_TARGET_NAME}_IMG
        ${CMAKE_CURRENT_SOURCE_DIR}/space.png)

add_custom_command(TARGET  ${TEST_TARGET_NAME} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_SHADER_VERT} $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_SHADER_FRAG} $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_MESH}        $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>
                   COMMAND ${CMAKE_COMMAND} -E copy ${${TEST_TARGET_NAME}_IMG}         $<TARGET_FILE_DIR:${TEST_TARGET_NAME}>)
