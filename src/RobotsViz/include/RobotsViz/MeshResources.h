/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_MESHRESOURCES_H
#define ROBOTSVIZ_MESHRESOURCES_H

#include <cmrc/cmrc.hpp>
CMRC_DECLARE(meshes);

#include <string>

namespace RobotsViz {
    class MeshResources;
}


class RobotsViz::MeshResources
{
public:
    MeshResources(const std::string& name);

    virtual ~MeshResources();

    const std::string& get_data() const;

private:
    std::string data_;

    const std::string log_name_ = "MeshResources";
};

#endif /* ROBOTSVIZ_MESHRESOURCES_H */
