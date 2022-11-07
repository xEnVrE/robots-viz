/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_POINTCLOUDSOURCE_H
#define ROBOTSVIZ_POINTCLOUDSOURCE_H

#include <Eigen/Dense>

#include <utility>

namespace RobotsViz {
    class PointCloudSource;
}


class RobotsViz::PointCloudSource
{
public:
    virtual bool freeze(const bool& blocking);

    virtual Eigen::MatrixXd points() = 0;

    virtual Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> colors() = 0;
};

#endif /* ROBOTSVIZ_POINTCLOUDSOURCE_H */
