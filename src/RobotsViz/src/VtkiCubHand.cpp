/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkiCubHand.h>

#include <RobotsIO/Hand/iCubHand.h>

#include <unordered_map>

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
    meshes_.emplace("palm", VtkMeshOBJ("full_" +  laterality_key + "HandPalm_red.obj", true, color, opacity));

    meshes_.emplace("thumb0", VtkMeshOBJ("full_" + laterality_key + "Thumb0.obj", true, color, opacity));
    meshes_.emplace("thumb1", VtkMeshOBJ("full_" + laterality_key + "ThumbFinger1.obj", true, color, opacity));
    meshes_.emplace("thumb2", VtkMeshOBJ("full_" + laterality_key + "ThumbFinger2.obj", true, color, opacity));
    meshes_.emplace("thumb3", VtkMeshOBJ("full_" + laterality_key + "ThumbFingerTip.obj", true, color, opacity));

    meshes_.emplace("index0", VtkMeshOBJ("full_" + laterality_key + "IndexFinger0.obj", true, color, opacity));
    meshes_.emplace("index1", VtkMeshOBJ("full_" + laterality_key + "IndexFinger1.obj", true, color, opacity));
    meshes_.emplace("index2", VtkMeshOBJ("full_" + laterality_key + "IndexFinger2.obj", true, color, opacity));
    meshes_.emplace("index3", VtkMeshOBJ("full_" + laterality_key + "IndexFingerTip_b.obj", true, color, opacity));

    meshes_.emplace("middle0", VtkMeshOBJ("full_" + laterality_key + "MiddleFinger0.obj", true, color, opacity));
    meshes_.emplace("middle1", VtkMeshOBJ("full_" + laterality_key + "MiddleFinger1.obj", true, color, opacity));
    meshes_.emplace("middle2", VtkMeshOBJ("full_" + laterality_key + "MiddleFinger2.obj", true, color, opacity));
    meshes_.emplace("middle3", VtkMeshOBJ("full_" + laterality_key + "MiddleFingerTip_b.obj", true, color, opacity));

    meshes_.emplace("ring0", VtkMeshOBJ("full_" + laterality_key + "RingFinger0.obj", true, color, opacity));
    meshes_.emplace("ring1", VtkMeshOBJ("full_" + laterality_key + "RingFinger1.obj", true, color, opacity));
    meshes_.emplace("ring2", VtkMeshOBJ("full_" + laterality_key + "RingFinger2.obj", true, color, opacity));
    meshes_.emplace("ring3", VtkMeshOBJ("full_" + laterality_key + "RingFingerTip_b.obj", true, color, opacity));

    meshes_.emplace("little0", VtkMeshOBJ("full_" + laterality_key + "LittleFinger0.obj", true, color, opacity));
    meshes_.emplace("little1", VtkMeshOBJ("full_" + laterality_key + "LittleFinger1.obj", true, color, opacity));
    meshes_.emplace("little2", VtkMeshOBJ("full_" + laterality_key + "LittleFinger2.obj", true, color, opacity));
    meshes_.emplace("little3", VtkMeshOBJ("full_" + laterality_key + "LittleFingerTip_b.obj", true, color, opacity));

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
}


VtkiCubHand::~VtkiCubHand()
{
    hand_pose_port_in_.close();
}


void VtkiCubHand::add_to_renderer(vtkRenderer& renderer)
{
    /* Add all the parts to the renderer. */
    for (auto mesh : meshes_)
        mesh.second.add_to_renderer(renderer);
}


bool VtkiCubHand::update(const bool& blocking)
{
    yarp::sig::Vector* pose_in = hand_pose_port_in_.read(blocking);

    if (pose_in == nullptr)
        return false;

    std::unordered_map<std::string, VectorXd> encoders;
    if (use_fingers_)
    {
        bool valid_encoders = false;
        std::tie(valid_encoders, encoders) = fingers_encoders_->encoders(blocking);

        if (!valid_encoders)
            return false;
    }

    VectorXd pose = toEigen(*pose_in);
    Transform<double, 3, Affine> transform;
    transform = Translation<double, 3>(pose.head<3>());
    transform.rotate(AngleAxisd(pose(6), pose.segment<3>(3)));

    /* Palm. */
    for (auto mesh : meshes_)
        if (use_fingers_)
            mesh.second.set_pose(transform * forward_kinematics_->map("ee", mesh.first, encoders));
        else
            mesh.second.set_pose(transform * forward_kinematics_->map("ee", mesh.first));

    return true;
}
