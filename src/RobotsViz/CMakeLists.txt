#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

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

if (USE_OTL)
  # ObjectTrackingLibrary
  find_package(OTL REQUIRED)
endif()

# Header files
list(APPEND ${LIBRARY_TARGET_NAME}_HDR
    include/RobotsViz/MeshResources.h
    include/RobotsViz/Resources.h
    include/RobotsViz/VtkContainer.h
    include/RobotsViz/VtkContent.h
    include/RobotsViz/VtkMeshOBJ.h
    include/RobotsViz/VtkObject.h
    include/RobotsViz/VtkOrientationCube.h
    include/RobotsViz/VtkPointCloud.h
    include/RobotsViz/VtkReferenceFrame.h
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
    src/VtkObject.cpp
    src/VtkOrientationCube.cpp
    src/VtkPointCloud.cpp
    src/VtkReferenceFrame.cpp
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

if (USE_OTL)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR
         include/RobotsViz/MeshResourcesOTL.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC
         src/MeshResourcesOTL.cpp
    )
endif()

# List of shader files
# TODO: To be moved somewhere else
cmrc_add_resource_library(${LIBRARY_TARGET_NAME}_RCShader
                          ALIAS ${LIBRARY_TARGET_NAME}::RCShader
                          NAMESPACE meshes
                          PREFIX __prc
                          meshes/full_LeftTopCover.obj
                          meshes/full_LeftHandPalm.obj
                          meshes/full_LeftThumb0.obj
                          meshes/full_LeftThumb1.obj
                          meshes/full_LeftThumb2.obj
                          meshes/full_LeftThumb3.obj
                          meshes/full_LeftIndex0.obj
                          meshes/full_LeftIndex1.obj
                          meshes/full_LeftIndex2.obj
                          meshes/full_LeftIndex3.obj
                          meshes/full_LeftMiddle0.obj
                          meshes/full_LeftMiddle1.obj
                          meshes/full_LeftMiddle2.obj
                          meshes/full_LeftMiddle3.obj
                          meshes/full_LeftRing0.obj
                          meshes/full_LeftRing1.obj
                          meshes/full_LeftRing2.obj
                          meshes/full_LeftRing3.obj
                          meshes/full_LeftLittle0.obj
                          meshes/full_LeftLittle1.obj
                          meshes/full_LeftLittle2.obj
                          meshes/full_LeftLittle3.obj
                          # meshes/full_RightTopCover.obj
                          meshes/full_RightHandPalm.obj
                          meshes/full_RightThumb0.obj
                          meshes/full_RightThumb1.obj
                          meshes/full_RightThumb2.obj
                          meshes/full_RightThumb3.obj
                          meshes/full_RightIndex0.obj
                          meshes/full_RightIndex1.obj
                          meshes/full_RightIndex2.obj
                          meshes/full_RightIndex3.obj
                          meshes/full_RightMiddle0.obj
                          meshes/full_RightMiddle1.obj
                          meshes/full_RightMiddle2.obj
                          meshes/full_RightMiddle3.obj
                          meshes/full_RightRing0.obj
                          meshes/full_RightRing1.obj
                          meshes/full_RightRing2.obj
                          meshes/full_RightRing3.obj
                          meshes/full_RightLittle0.obj
                          meshes/full_RightLittle1.obj
                          meshes/full_RightLittle2.obj
                          meshes/full_RightLittle3.obj
                          meshes/cube.obj
)

# Create library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

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

if (USE_OTL)
  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                        OTL::OTL
                        )
endif()

if (USE_OTL)
    target_compile_definitions(${LIBRARY_TARGET_NAME} PUBLIC ROBOTSVIZ_USES_OTL)
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
