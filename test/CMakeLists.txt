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
add_subdirectory(test_sicad)
add_subdirectory(test_sicad_frame)
add_subdirectory(test_sicad_model_frame)
add_subdirectory(test_sicad_shader_path)
add_subdirectory(test_thread_contexts)


set(TEST_BACKGROUND
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/background/space.png
)

set(TEST_GROUND_TRUTH
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_scissors_background.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_scissors.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_alien_frame.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_alien_space.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_alien.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_frame.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_textured_alien.png
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/ground_truth/gt_sicad_textured_alien_space.png
)

set(TEST_MESH
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/mesh/spaceinvader.obj
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/mesh/spaceinvader_textured.obj
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/mesh/spaceinvader_textured.mtl
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/mesh/spaceinvader_texture.jpg
)

set(TEST_SHADER
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_background.frag
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_background.vert
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_frame.frag
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_frame.vert
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_model_texture.frag
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_model.frag
      ${CMAKE_CURRENT_SOURCE_DIR}/misc/shader/shader_model.vert
)


set(SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX)
if("${CMAKE_GENERATOR}" MATCHES "^Visual Studio.+" OR "${CMAKE_GENERATOR}" MATCHES "Xcode")
  set(SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX $<CONFIG>)
endif()

add_custom_target(test_files ALL)
add_custom_command(TARGET test_files POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory             ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_BACKGROUND}    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_GROUND_TRUTH}  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_MESH}          ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
                   COMMAND ${CMAKE_COMMAND} -E copy ${TEST_SHADER}        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SI_RUNTIME_OUTPUT_DIRECTORY_SUFFIX}
)
