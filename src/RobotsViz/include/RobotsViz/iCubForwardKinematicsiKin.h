/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBFORWARDKINEMATICSIKIN_H
#define ICUBFORWARDKINEMATICSIKIN_H

#include <Eigen/Dense>

#include <string>
#include <unordered_map>

#include <iCub/iKin/iKinFwd.h>


using T = Eigen::Transform<double, 3, Eigen::Affine>;

namespace RobotsViz {
    class iCubForwardKinematicsiKin;
}


class RobotsViz::iCubForwardKinematicsiKin
{
public:
    iCubForwardKinematicsiKin(const std::string& part_name);

    virtual ~iCubForwardKinematicsiKin();

    T map(const std::string& from, const std::string& to, const std::unordered_map<std::string, Eigen::VectorXd>& encoders);

    T map(const std::string& from, const std::string& to);

private:
    T DH(const double& d, const double& theta, const double& a, const double& alpha);

    std::unordered_map<std::string, std::unordered_map<std::string, T>> maps_;

    std::unordered_map<std::string, iCub::iKin::iCubFinger> fingers_fwd_kin_;

    const std::string part_name_;

    const std::string log_name_ = "iCubForwardKinematicsiKin";
};

#endif
