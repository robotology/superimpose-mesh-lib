#===============================================================================
#
# Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. See the accompanying LICENSE file for details.
#
#===============================================================================

if(WIN32)
  cmake_minimum_required(VERSION 3.4)
else()
  cmake_minimum_required(VERSION 3.0)
endif()


### Define the project
project(SuperimposeMesh
        LANGUAGES CXX
        VERSION 0.10.0)


### Define project specific options
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(GNUInstallDirs)
include(AddDependenciesPrintUtils)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
  if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(CMAKE_CXX_FLAGS "/MP /EHsc")
    set(CMAKE_C_FLAGS "/MP /EHsc")
  endif()
endif()


### Use YCM to improve and simplify CMake maintenence
find_package(YCM 0.10.2 REQUIRED QUIET)
print_dependency(YCM)


### Configure project
# Is macOS?
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_definitions(-D GLFW_MAC)
endif()

# Shared/Dynamic or Static library?
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Build test related commands?
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
  enable_testing()
endif()

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Default Build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()


### Compile- and install-related commands
add_subdirectory(src)

# Install the files necessary to call find_package(LibTemplateCMake) in CMake projects
include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY ExactVersion
                            NAMESPACE SI::
                            NO_SET_AND_CHECK_MACRO
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            UPPERCASE_FILENAMES
                            DEPENDENCIES YCM assimp GLEW "glfw3 CONFIG" GLM OpenCV)

# Add the uninstall target
include(AddUninstallTarget)


### Add integration tests (unit tests for each library should be in each sublibrary directory)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()


# Add targets related to doxygen documention generation
add_subdirectory(doc EXCLUDE_FROM_ALL)
