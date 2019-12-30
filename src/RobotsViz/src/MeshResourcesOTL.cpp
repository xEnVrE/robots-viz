/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <OTL/MeshResource.h>
#include <OTL/ModelParameters.h>
#include <RobotsViz/MeshResourcesOTL.h>

using namespace OTL;

using namespace RobotsViz;


MeshResourcesOTL::MeshResourcesOTL(const std::string& name, const std::string& set)
{
    OTL::MeshResource mesh_resource(name, set);
    data_ = mesh_resource.as_string();
}


MeshResourcesOTL::MeshResourcesOTL(const ModelParameters& model_parameters)
{
    OTL::MeshResource mesh_resource(model_parameters);
    data_ = mesh_resource.as_string();
}


MeshResourcesOTL::~MeshResourcesOTL()
{}


const std::string& MeshResourcesOTL::as_string() const
{
    return data_;
}
