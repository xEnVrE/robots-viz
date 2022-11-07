/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/PointCloudSource.h>

using namespace Eigen;
using namespace RobotsViz;


bool PointCloudSource::freeze(const bool& blocking)
{
    return true;
}


std::tuple<bool, Transform<double, 3, Affine>> PointCloudSource::pose()
{
    /* By default, the point cloud source do not provide any pose. */
    Transform<double, 3, Affine> empty;

    return std::make_tuple(false, empty);
}
