add_subdirectory(test_hdpi)
add_subdirectory(test_initialization_control)
add_subdirectory(test_moving_object)
add_subdirectory(test_multiple_windows_moving_object)
add_subdirectory(test_scissors)
add_subdirectory(test_scissors_background)
add_subdirectory(test_scissors_moving_objects)
add_subdirectory(test_sicad_frame)
add_subdirectory(test_sicad_model)
add_subdirectory(test_sicad_model_frame)
add_subdirectory(test_thread_contexts)

set(TEST_SHADER_VERT
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_background.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_frame.vert
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_model.vert
)

set(TEST_SHADER_FRAG
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_background.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_frame.frag
        ${CMAKE_CURRENT_SOURCE_DIR}/common/shader_model.frag
)

set(TEST_MESH
        ${CMAKE_CURRENT_SOURCE_DIR}/common/Space_Invader.obj
)

set(TEST_IMG
        ${CMAKE_CURRENT_SOURCE_DIR}/common/space.png
)

add_custom_target(test_files ALL)
add_custom_command(TARGET test_files POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_SHADER_VERT} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_SHADER_FRAG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_MESH}        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_IMG}         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
