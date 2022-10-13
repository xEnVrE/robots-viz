/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/MeshResources.h>
#include <RobotsViz/VtkiCubHand.h>


#include <RobotsIO/Hand/iCubHand.h>

#include <unordered_map>
#include "RobotsViz/MeshResources.h"

#include <yarp/eigen/Eigen.h>


using namespace Eigen;
using namespace RobotsIO::Hand;
using namespace RobotsViz;
using namespace yarp::eigen;


VtkiCubHand::VtkiCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_fingers, const bool& use_analogs, const std::tuple<double, double, double>& color, const double& opacity) :
    use_fingers_(use_fingers)
{
    if ((laterality != "left") && (laterality != "right"))
        throw(std::runtime_error(log_name_ + "::ctor. Invalid laterality specified."));

    std::string laterality_key = ((laterality == "right") ? "Right" : "Left");

    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Open input port. */
    if(!hand_pose_port_in_.open("/" + port_prefix + "/vtk-icub-hand/" + laterality + "/state:i"))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot open hand pose input port."));

    /* Add meshes of hand parts. */
    meshes_.emplace("palm", VtkMeshOBJ(MeshResources("full_" +  laterality_key + "HandPalm.obj"), color, opacity));
    meshes_.emplace("top_cover", VtkMeshOBJ(MeshResources("full_" +  laterality_key + "TopCover.obj"), color, opacity));

    meshes_.emplace("thumb0", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Thumb0.obj"), color, opacity));
    meshes_.emplace("thumb1", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Thumb1.obj"), color, opacity));
    meshes_.emplace("thumb2", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Thumb2.obj"), color, opacity));
    meshes_.emplace("thumb3", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Thumb3.obj"), color, opacity));

    meshes_.emplace("index0", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Index0.obj"), color, opacity));
    meshes_.emplace("index1", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Index1.obj"), color, opacity));
    meshes_.emplace("index2", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Index2.obj"), color, opacity));
    meshes_.emplace("index3", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Index3.obj"), color, opacity));

    meshes_.emplace("middle0", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Middle0.obj"), color, opacity));
    meshes_.emplace("middle1", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Middle1.obj"), color, opacity));
    meshes_.emplace("middle2", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Middle2.obj"), color, opacity));
    meshes_.emplace("middle3", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Middle3.obj"), color, opacity));

    meshes_.emplace("ring0", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Ring0.obj"), color, opacity));
    meshes_.emplace("ring1", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Ring1.obj"), color, opacity));
    meshes_.emplace("ring2", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Ring2.obj"), color, opacity));
    meshes_.emplace("ring3", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Ring3.obj"), color, opacity));

    meshes_.emplace("little0", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Little0.obj"), color, opacity));
    meshes_.emplace("little1", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Little1.obj"), color, opacity));
    meshes_.emplace("little2", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Little2.obj"), color, opacity));
    meshes_.emplace("little3", VtkMeshOBJ(MeshResources("full_" + laterality_key + "Little3.obj"), color, opacity));

    /* Configure forward kinematics. */
    forward_kinematics_ = std::unique_ptr<iCubForwardKinematics>
    (
        new iCubForwardKinematics(laterality + "_hand")
    );

    /* Configure fingers encoders. */
    if (use_fingers_)
    {
        fingers_encoders_ = std::unique_ptr<iCubHand>
        (
            new iCubHand(robot_name, laterality, port_prefix + "/vtk-icub-hand", "icub-fingers-encoders", use_analogs)
        );
    }

    transform_.setIdentity();
}


VtkiCubHand::~VtkiCubHand()
{
    hand_pose_port_in_.close();
}


void VtkiCubHand::add_to_renderer(vtkRenderer& renderer)
{
    /* Add all the parts to the renderer. */
    for (auto& mesh : meshes_)
        mesh.second.add_to_renderer(renderer);
}


bool VtkiCubHand::update(const bool& blocking)
{
    yarp::sig::Vector* pose_in = hand_pose_port_in_.read(blocking);

    if (pose_in != nullptr)
    {
        VectorXd pose = toEigen(*pose_in);
        Transform<double, 3, Affine> transform;
        transform = Translation<double, 3>(pose.head<3>());
        transform.rotate(AngleAxisd(pose(6), pose.segment<3>(3)));
        transform_ = transform;
    }

    std::unordered_map<std::string, VectorXd> encoders;
    if (use_fingers_)
    {
        bool valid_encoders = false;
        std::tie(valid_encoders, encoders) = fingers_encoders_->encoders(blocking);

        if (!valid_encoders)
            return false;
    }

    /* Palm. */
    for (auto& mesh : meshes_)
        if (use_fingers_)
            mesh.second.set_pose(transform_ * forward_kinematics_->map("ee", mesh.first, encoders));
        else
            mesh.second.set_pose(transform_ * forward_kinematics_->map("ee", mesh.first));

    return true;
}

void RobotsViz::VtkiCubHand::setTransform(const Eigen::Matrix4d &transform)
{
    transform_ = transform;
}
