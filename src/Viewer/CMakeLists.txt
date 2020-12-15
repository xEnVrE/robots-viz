#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME robots-viz-viewer)

find_package(RobotsViz REQUIRED)

find_package(YARP REQUIRED COMPONENTS os)

set(${EXE_TARGET_NAME}_HDR
    include/Viewer.h
)

set(${EXE_TARGET_NAME}_SRC
    src/Viewer.cpp
    src/main.cpp
)

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      RobotsViz::RobotsViz
                      YARP::YARP_init
                      YARP::YARP_os
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
