/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_MESHRESOURCES_H
#define ROBOTSVIZ_MESHRESOURCES_H

#include <RobotsViz/Resources.h>

#include <string>

namespace RobotsViz {
    class MeshResources;
}


class RobotsViz::MeshResources : public RobotsViz::Resources
{
public:
    MeshResources(const std::string& name, const std::string& set_name = "meshes");

    virtual ~MeshResources();

    virtual const std::string& as_string() const override;

private:
    std::string data_;

    const std::string log_name_ = "MeshResources";
};

#endif /* ROBOTSVIZ_MESHRESOURCES_H */
