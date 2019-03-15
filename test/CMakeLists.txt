#===============================================================================
#
# Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. See the accompanying LICENSE file for details.
#
#===============================================================================

message(STATUS "Creating and configuring tests.")

add_subdirectory(test_hdpi)
add_subdirectory(test_moving_object)
add_subdirectory(test_multiple_windows_moving_object)
add_subdirectory(test_public_interface)
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

set(TEST_GROUND_TRUTH
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_scissors_background.png
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_scissors.png
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_sicad_alien_frame.png
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_sicad_alien_space.png
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_sicad_alien.png
      ${CMAKE_CURRENT_SOURCE_DIR}/common/gt_sicad_frame.png
)


set(SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX)
if("${CMAKE_GENERATOR}" MATCHES "^Visual Studio.+")
  set(SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX $<CONFIG>)
endif()



add_custom_target(test_files ALL)
add_custom_command(TARGET test_files POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_SHADER_VERT}  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_SHADER_FRAG}  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_MESH}         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_IMG}          ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_GROUND_TRUTH} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
)
