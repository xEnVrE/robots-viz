/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <Eigen/Dense>

#include <RobotsViz/VtkContainer.h>

#include <string>

#include <yarp/os/ResourceFinder.h>


class Viewer
{
public:
    Viewer(const yarp::os::ResourceFinder& resource_finder);

    void run();

private:
    std::unique_ptr<RobotsViz::VtkContainer> vtk_container_;

    const std::string log_name_ = "ObjectViewer";
};

#endif /* VIEWER_H */
