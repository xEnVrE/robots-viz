/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_MESHRESOURCESOTL_H
#define ROBOTSVIZ_MESHRESOURCESOTL_H

#include <RobotsViz/Resources.h>

#include <OTL/ModelParameters.h>

#include <string>

namespace RobotsViz {
    class MeshResourcesOTL;
}


class RobotsViz::MeshResourcesOTL : public RobotsViz::Resources
{
public:
    MeshResourcesOTL(const std::string& name, const std::string& set);

    MeshResourcesOTL(const OTL::ModelParameters& model_parameters);

    virtual ~MeshResourcesOTL();

    virtual const std::string& as_string() const override;

private:
    std::string data_;

    const std::string log_name_ = "MeshResourcesOTL";
};

#endif /* ROBOTSVIZ_MESHRESOURCESOTL_H */
