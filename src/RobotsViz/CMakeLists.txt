#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_YARP        "Use YARP"            OFF)

set(LIBRARY_TARGET_NAME RobotsViz)

include(CMakeRC)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# RobotsIO
find_package(RobotsIO REQUIRED)

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

if (USE_SUPERIMPOSE)
  # Superimposemesh
  find_package(SuperimposeMesh 0.11.100 REQUIRED)
endif()

if (USE_YARP)
    # YARP
    find_package(YARP CONFIG REQUIRED
                 COMPONENTS
                 eigen
                 os
                 sig
    )
endif()

# Header files
list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/RobotsViz/MeshResources.h
    include/RobotsViz/VtkContainer.h
    include/RobotsViz/VtkContent.h
    include/RobotsViz/VtkMeshOBJ.h
    include/RobotsViz/VtkPointCloud.h
    include/RobotsViz/VtkUpdateHandler.h
    include/RobotsViz/iCubForwardKinematics.h
    include/RobotsViz/vtkOBJResource.h
)

# Source files
list(APPEND ${LIBRARY_TARGET_NAME}_SRC
    src/MeshResources.cpp
    src/VtkContainer.cpp
    src/VtkContent.cpp
    src/VtkMeshOBJ.cpp
    src/VtkPointCloud.cpp
    src/VtkUpdateHandler.cpp
    src/iCubForwardKinematics.cpp
    src/vtkOBJResource.cpp
)

if (USE_YARP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
         include/RobotsViz/VtkiCubHand.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
         src/VtkiCubHand.cpp
    )
endif()

if (USE_SUPERIMPOSE AND USE_YARP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
         include/RobotsViz/SIiCubHand.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
         src/SIiCubHand.cpp
    )
endif()

# List of shader files
# TODO: To be moved somewhere else
cmrc_add_resource_library(${LIBRARY_TARGET_NAME}_RCShader
                          ALIAS ${LIBRARY_TARGET_NAME}::RCShader
                          NAMESPACE meshes
                          PREFIX __prc
                          meshes/full_LeftHandPalm_red.obj
                          meshes/full_LeftThumb0.obj
                          meshes/full_LeftThumbFinger1.obj
                          meshes/full_LeftThumbFinger2.obj
                          meshes/full_LeftThumbFingerTip.obj
                          meshes/full_LeftIndexFinger0.obj
                          meshes/full_LeftIndexFinger1.obj
                          meshes/full_LeftIndexFinger2.obj
                          meshes/full_LeftIndexFingerTip_b.obj
                          meshes/full_LeftMiddleFinger0.obj
                          meshes/full_LeftMiddleFinger1.obj
                          meshes/full_LeftMiddleFinger2.obj
                          meshes/full_LeftMiddleFingerTip_b.obj
                          meshes/full_LeftRingFinger0.obj
                          meshes/full_LeftRingFinger1.obj
                          meshes/full_LeftRingFinger2.obj
                          meshes/full_LeftRingFingerTip_b.obj
                          meshes/full_LeftLittleFinger0.obj
                          meshes/full_LeftLittleFinger1.obj
                          meshes/full_LeftLittleFinger2.obj
                          meshes/full_LeftLittleFingerTip_b.obj
)

# Create library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(SI::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_include_directories(${LIBRARY_TARGET_NAME}
                           PUBLIC
                             "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                             "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${LIBRARY_TARGET_NAME}
                      PRIVATE
                      ${LIBRARY_TARGET_NAME}::RCShader
                      PUBLIC
                      Eigen3::Eigen
                      RobotsIO::RobotsIO
                      ${OpenCV_LIBS}
                      ${VTK_LIBRARIES}
)

if (USE_SUPERIMPOSE)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          SI::SuperimposeMesh
                          )
endif()

if (USE_YARP)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          YARP::YARP_eigen
                          YARP::YARP_init
                          YARP::YARP_os
                          YARP::YARP_sig
                          )
endif()

set_target_properties(${LIBRARY_TARGET_NAME}
                      PROPERTIES
                      VERSION ${${PROJECT_NAME}_VERSION}
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)
