#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.0)

project(RobotsViz
        LANGUAGES CXX
        VERSION 0.0.100)

set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES)
    if(MSVC)
        # Since MSVC produces both release and debug
        set(CMAKE_DEBUG_POSTFIX "d")
        # Enable parallel compile and optimized handling of exception filters
        set(CMAKE_CXX_FLAGS "/MP /EHsc")
        set(CMAKE_C_FLAGS "/MP /EHsc")
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

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

# options for YARP and SuperimposeMesh
option(USE_SUPERIMPOSE "Use SuperimposeMesh" OFF)
option(USE_YARP        "Use YARP"            OFF)
option(USE_OTL         "Use ObjectTrackingLibrary" OFF)
option(BUILD_VIEWER    "Build basic 3D viewer" OFF)

# Library sources
add_subdirectory(src)

# Install the files necessary to call find_package(LibTemplateCMake) in CMake projects

set(DEPENDENCIES "Eigen3" "RobotsIO" "VTK")
if (USE_YARP)
  set(DEPENDENCIES ${DEPENDENCIES} "YARP COMPONENTS eigen os sig")
endif()
if (USE_SUPERIMPOSE)
  set(DEPENDENCIES ${DEPENDENCIES} "SuperimposeMesh")
endif()
if (USE_OTL)
  set(DEPENDENCIES ${DEPENDENCIES} "OTL")
endif()

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY ExactVersion
                            EXPORT ${PROJECT_NAME}
                            NO_SET_AND_CHECK_MACRO
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            UPPERCASE_FILENAMES
                            DEPENDENCIES ${DEPENDENCIES}
                            INCLUDE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ForceVTKUseFile.cmake)

# Add the uninstall target
include(AddUninstallTarget)
