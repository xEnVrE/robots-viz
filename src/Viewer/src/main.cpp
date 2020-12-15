/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Viewer.h>

#include <cstdlib>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

using namespace Eigen;
using namespace yarp::os;


int main(int argc, char** argv)
{
    const std::string log_name = "robots-viz-viewer";

    Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout << log_name << "::main. Unable to find YARP." << std::endl;

        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("robots-viz-viewer");
    rf.configure(argc, argv);

    Viewer viewer(rf);
    viewer.run();

    return EXIT_SUCCESS;
}
