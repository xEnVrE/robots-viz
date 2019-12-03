/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/SIiCubHand.h>
#include <RobotsViz/MeshResources.h>

#include <RobotsIO/Hand/iCubHand.h>

#include <sstream>

#include <yarp/eigen/Eigen.h>

using namespace Eigen;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Hand;
using namespace RobotsViz;
using namespace yarp::eigen;


SIiCubHand::SIiCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_analogs, const bool& use_camera_pose, std::shared_ptr<Camera> camera) :
    use_camera_pose_(use_camera_pose),
    camera_(camera)
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
    if(!hand_pose_port_in_.open("/" + port_prefix + "/" + laterality + "/state:i"))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot open hand pose input port."));

    /* Load meshes. */
    std::istringstream top_cover_model_stream(MeshResources("full_" +  laterality_key + "TopCover.obj").get_data());
    std::istringstream palm_model_stream(MeshResources("full_" +  laterality_key + "HandPalm_red.obj").get_data());

    std::istringstream thumb0_model_stream(MeshResources("full_" +  laterality_key + "Thumb0.obj").get_data());
    std::istringstream thumb1_model_stream(MeshResources("full_" +  laterality_key + "ThumbFinger1.obj").get_data());
    std::istringstream thumb2_model_stream(MeshResources("full_" +  laterality_key + "ThumbFinger2.obj").get_data());
    std::istringstream thumb3_model_stream(MeshResources("full_" +  laterality_key + "ThumbFingerTip.obj").get_data());

    std::istringstream index0_model_stream(MeshResources("full_" +  laterality_key + "IndexFinger0.obj").get_data());
    std::istringstream index1_model_stream(MeshResources("full_" +  laterality_key + "IndexFinger1.obj").get_data());
    std::istringstream index2_model_stream(MeshResources("full_" +  laterality_key + "IndexFinger2.obj").get_data());
    std::istringstream index3_model_stream(MeshResources("full_" +  laterality_key + "IndexFingerTip_b.obj").get_data());

    std::istringstream middle0_model_stream(MeshResources("full_" +  laterality_key + "MiddleFinger0.obj").get_data());
    std::istringstream middle1_model_stream(MeshResources("full_" +  laterality_key + "MiddleFinger1.obj").get_data());
    std::istringstream middle2_model_stream(MeshResources("full_" +  laterality_key + "MiddleFinger2.obj").get_data());
    std::istringstream middle3_model_stream(MeshResources("full_" +  laterality_key + "MiddleFingerTip_b.obj").get_data());

    std::istringstream ring0_model_stream(MeshResources("full_" +  laterality_key + "RingFinger0.obj").get_data());
    std::istringstream ring1_model_stream(MeshResources("full_" +  laterality_key + "RingFinger1.obj").get_data());
    std::istringstream ring2_model_stream(MeshResources("full_" +  laterality_key + "RingFinger2.obj").get_data());
    std::istringstream ring3_model_stream(MeshResources("full_" +  laterality_key + "RingFingerTip_b.obj").get_data());

    std::istringstream little0_model_stream(MeshResources("full_" +  laterality_key + "LittleFinger0.obj").get_data());
    std::istringstream little1_model_stream(MeshResources("full_" +  laterality_key + "LittleFinger1.obj").get_data());
    std::istringstream little2_model_stream(MeshResources("full_" +  laterality_key + "LittleFinger2.obj").get_data());
    std::istringstream little3_model_stream(MeshResources("full_" +  laterality_key + "LittleFingerTip_b.obj").get_data());

    meshes_["top_cover"] = &top_cover_model_stream;
    meshes_["palm"] = &palm_model_stream;

    meshes_["thumb0"] = &thumb0_model_stream;
    meshes_["thumb1"] = &thumb1_model_stream;
    meshes_["thumb2"] = &thumb2_model_stream;
    meshes_["thumb3"] = &thumb3_model_stream;

    meshes_["index0"] = &index0_model_stream;
    meshes_["index1"] = &index1_model_stream;
    meshes_["index2"] = &index2_model_stream;
    meshes_["index3"] = &index3_model_stream;

    meshes_["middle0"] = &middle0_model_stream;
    meshes_["middle1"] = &middle1_model_stream;
    meshes_["middle2"] = &middle2_model_stream;
    meshes_["middle3"] = &middle3_model_stream;

    meshes_["ring0"] = &ring0_model_stream;
    meshes_["ring1"] = &ring1_model_stream;
    meshes_["ring2"] = &ring2_model_stream;
    meshes_["ring3"] = &ring3_model_stream;

    meshes_["little0"] = &little0_model_stream;
    meshes_["little1"] = &little1_model_stream;
    meshes_["little2"] = &little2_model_stream;
    meshes_["little3"] = &little3_model_stream;

    /* Get camera parameters. */
    CameraParameters parameters;
    std::tie(std::ignore, parameters) = camera_->parameters();

    /* Initialize rendering engine. */
    renderer_ = std::unique_ptr<SICAD>
    (
        new SICAD(meshes_, parameters.width, parameters.height, parameters.fx, parameters.fy, parameters.cx, parameters.cy)
    );
    renderer_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});

    /* Configure forward kinematics. */
    forward_kinematics_ = std::unique_ptr<iCubForwardKinematics>
    (
        new iCubForwardKinematics(laterality + "_hand")
    );

    /* Configure fingers encoders. */
    fingers_encoders_ = std::unique_ptr<iCubHand>
    (
        new iCubHand(robot_name, laterality, port_prefix, "icub-fingers-encoders", use_analogs)
    );
}


SIiCubHand:: ~SIiCubHand()
{
    hand_pose_port_in_.close();
}


std::pair<bool, cv::Mat> SIiCubHand::render_image(const bool& blocking)
{
    yarp::sig::Vector* pose_in = hand_pose_port_in_.read(blocking);

    if (pose_in == nullptr)
        return std::make_pair(false, cv::Mat());

    bool valid_encoders = false;
    std::unordered_map<std::string, VectorXd> encoders;
    std::tie(valid_encoders, encoders) = fingers_encoders_->encoders(blocking);

    if (!valid_encoders)
        return std::make_pair(false, cv::Mat());

    VectorXd pose = toEigen(*pose_in);
    Transform<double, 3, Affine> transform;
    transform = Translation<double, 3>(pose.head<3>());
    transform.rotate(AngleAxisd(pose(6), pose.segment<3>(3)));

    Superimpose::ModelPoseContainer poses;
    for (auto mesh : meshes_)
        poses.emplace(mesh.first, transform_to_vector(transform * forward_kinematics_->map("ee", mesh.first, encoders)));


    double cam_x [3] = {0.0, 0.0, 0.0};
    double cam_o [4] = {1.0, 0.0, 0.0, 0.0};
    cv::Mat render_image;
    if (use_camera_pose_)
    {
        bool valid_pose = false;
        Transform<double, 3, Affine> pose;
        std::tie(valid_pose, pose) = camera_->pose(true);
        if (!valid_pose)
            std::make_pair(false, cv::Mat());

        VectorXd axis_angle(4);
        AngleAxisd angle_axis(pose.rotation());
        axis_angle.head<3>() = angle_axis.axis();
        axis_angle(3) = angle_axis.angle();
        renderer_->superimpose(poses, pose.translation().data(), axis_angle.data(), render_image);
    }
    else
        renderer_->superimpose(poses, cam_x, cam_o, render_image);

    return std::make_pair(true, render_image);
}


std::vector<double> SIiCubHand::transform_to_vector(const Eigen::Transform<double, 3, Eigen::Affine>& transform)
{
    std::vector<double> pose(7);
    pose[0] = transform.translation()(0);
    pose[1] = transform.translation()(1);
    pose[2] = transform.translation()(2);

    AngleAxisd angle_axis(transform.rotation());
    pose[3] = angle_axis.axis()(0);
    pose[4] = angle_axis.axis()(1);
    pose[5] = angle_axis.axis()(2);
    pose[6] = angle_axis.angle();

    return pose;
}
