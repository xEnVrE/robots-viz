/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/MeshResources.h>

using namespace RobotsViz;


MeshResources::MeshResources(const std::string& name)
{
    const std::string virtual_path = "__prc/meshes/" + name;

    auto cmrc_fs = cmrc::meshes::get_filesystem();

    if (!(cmrc_fs.exists(virtual_path) && cmrc_fs.is_file(virtual_path)))
        throw(std::runtime_error(log_name_ + "::ctor. Cannot find requested mesh among available resources."));

    auto mesh_cmrc_file = cmrc_fs.open(virtual_path);
    data_.assign(mesh_cmrc_file.cbegin(), mesh_cmrc_file.cend());
}


MeshResources::~MeshResources()
{}


const std::string& MeshResources::get_data() const
{
    return data_;
}
