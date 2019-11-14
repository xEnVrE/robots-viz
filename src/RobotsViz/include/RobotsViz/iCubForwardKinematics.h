/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBFORWARDKINEMATICS_H
#define ICUBFORWARDKINEMATICS_H

#include <Eigen/Dense>

#include <string>
#include <unordered_map>

using T = Eigen::Transform<double, 3, Eigen::Affine>;

namespace RobotsViz {
    class iCubForwardKinematics;
}


class RobotsViz::iCubForwardKinematics
{
public:
    iCubForwardKinematics(const std::string& part_name);

    virtual ~iCubForwardKinematics();

    T map(const std::string& from, const std::string& to, const std::unordered_map<std::string, Eigen::VectorXd>& encoders);

    T map(const std::string& from, const std::string& to);

private:
    T DH(const double& d, const double& theta, const double& a, const double& alpha);

    std::unordered_map<std::string, std::unordered_map<std::string, T>> maps_;

    const std::string log_name_ = "iCubForwardKinematics";
};

#endif
