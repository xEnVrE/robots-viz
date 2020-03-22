/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkObject.h>

#include <Eigen/Dense>

using namespace RobotsIO::Utils;
using namespace RobotsViz;

#ifdef ROBOTSVIZ_USES_OTL
#include <RobotsViz/MeshResourcesOTL.h>

using namespace OTL;
#endif


VtkObject::VtkObject
(
    const std::string& mesh_path,
    std::unique_ptr<RobotsIO::Utils::Transform> object_transform,
    const std::tuple<double, double, double>& color,
    const double& opacity
) :
    VtkObject(std::move(object_transform), color, opacity)
{
    vtk_mesh_ = std::unique_ptr<VtkMeshOBJ>(new VtkMeshOBJ(mesh_path, color, opacity));
}


VtkObject::VtkObject
(
    const Resources& mesh_resource,
    std::unique_ptr<RobotsIO::Utils::Transform> object_transform,
    const std::tuple<double, double, double>& color,
    const double& opacity
) :
    VtkObject(std::move(object_transform), color, opacity)
{
    vtk_mesh_ = std::unique_ptr<VtkMeshOBJ>(new VtkMeshOBJ(mesh_resource, color, opacity));
}


#ifdef ROBOTSVIZ_USES_OTL
VtkObject::VtkObject
(
    const ModelParameters& object_parameters,
    std::unique_ptr<Transform> object_transform,
    const std::tuple<double, double, double>& color,
    const double& opacity
) :
    VtkObject(std::move(object_transform), color, opacity)
{

    MeshResourcesOTL mesh_resource(object_parameters);
    vtk_mesh_ = std::unique_ptr<VtkMeshOBJ>(new VtkMeshOBJ(mesh_resource, color, opacity));
}
#endif


VtkObject::VtkObject
(
    std::unique_ptr<Transform> object_transform,
    const std::tuple<double, double, double>& color,
    const double& opacity
) :
    transform_(std::move(object_transform))
{
    vtk_frame_ = std::unique_ptr<VtkReferenceFrame>(new VtkReferenceFrame(0.0));
    vtk_frame_->set_visibility(false);
}


VtkObject::~VtkObject()
{}


void VtkObject::add_to_renderer(vtkRenderer& renderer)
{
    vtk_mesh_->add_to_renderer(renderer);

    if (vtk_frame_ != nullptr)
        vtk_frame_->add_to_renderer(renderer);
}


bool VtkObject::update(const bool& blocking)
{
    bool valid_transform = transform_->freeze(blocking);

    if (valid_transform)
    {
        auto transform = transform_->transform();
        vtk_mesh_->set_pose(transform);

        if (vtk_frame_ != nullptr)
        {
            vtk_frame_->set_transform(transform);
            vtk_frame_->update(false);
        }
    }

    return valid_transform;
}


VtkReferenceFrame& VtkObject::get_frame()
{
    /* Dereferencing is safe as the frame is always constructed in the ctor. */
    return *vtk_frame_;
}
