cmake_minimum_required(VERSION 3.5)

set(ORIGINAL_PROJECT_NAME "SuperimposeMesh")
get_filename_component(ORIGINAL_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

project(${ORIGINAL_PROJECT_NAME}Docs)

include(GNUInstallDirs)

find_package(Doxygen QUIET)
print_dependency(Doxygen)

if(DOXYGEN_FOUND)
    # This variables are used inside the Doxyfile.in template
    set(DOXYGEN_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}"
        CACHE STRING "Build directory for Doxygen html output (absolute).")

    set(DOXYGEN_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/doc/${ORIGINAL_PROJECT_NAME}"
        CACHE STRING "Installation directory for Doxygen html output (relative to CMAKE_INSTALL_PREFIX).")

    set(DOXYGEN_SOURCE_DIR "${ORIGINAL_PROJECT_DIR}/src")

    set(DOXYGEN_DOC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")


    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)


    add_custom_target(doc COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                          COMMENT "Generating doxygen documentation in ${DOXYGEN_BUILD_DIR}"
    )

    install(DIRECTORY ${DOXYGEN_BUILD_DIR}/html
                      ${DOXYGEN_BUILD_DIR}/latex
            DESTINATION ${DOXYGEN_INSTALL_DIR}
            OPTIONAL
    )
endif()
