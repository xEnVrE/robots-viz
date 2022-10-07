/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKICUBHAND_H
#define ROBOTSVIZ_VTKICUBHAND_H

#include <Eigen/Dense>

#include <RobotsIO/Hand/iCubHand.h>

#include <RobotsViz/iCubForwardKinematics.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkMeshOBJ.h>

#include <memory>
#include <unordered_map>

#include <vtkRenderer.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

namespace RobotsViz {
    class VtkiCubHand;
}


class RobotsViz::VtkiCubHand : public RobotsViz::VtkContent
{
public:
    VtkiCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_fingers, const bool& use_analogs, const std::tuple<double, double, double>& color, const double& opacity);

    virtual ~VtkiCubHand();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

private:
    yarp::os::Network yarp_;

    std::unordered_map<std::string, VtkMeshOBJ> meshes_;

    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in_;

    std::unique_ptr<iCubForwardKinematics> forward_kinematics_;

    std::unique_ptr<RobotsIO::Hand::iCubHand> fingers_encoders_;

    const bool use_fingers_;

    const std::string log_name_ = "VtkiCubHand";

    yarp::sig::Vector pose_;
};

#endif /* ROBOTSVIZ_VTKICUBHAND_H */
