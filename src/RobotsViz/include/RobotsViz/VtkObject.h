/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKOBJECT_H
#define ROBOTSVIZ_VTJOBJECT_H

#ifdef ROBOTSVIZ_USES_OTL
#include <OTL/ModelParameters.h>
#endif

#include <RobotsIO/Utils/Transform.h>

#include <RobotsViz/Resources.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/VtkMeshOBJ.h>


namespace RobotsViz {
    class VtkObject;
}


class RobotsViz::VtkObject : public RobotsViz::VtkContent
{
public:
    VtkObject(const std::string& mesh_path, std::unique_ptr<RobotsIO::Utils::Transform> object_transform, const std::tuple<double, double, double>& color, const double& opacity);

    VtkObject(const Resources& resource, std::unique_ptr<RobotsIO::Utils::Transform> object_transform, const std::tuple<double, double, double>& color, const double& opacity);

#ifdef ROBOTSVIZ_USES_OTL
    VtkObject(const OTL::ModelParameters& object_parameters, std::unique_ptr<RobotsIO::Utils::Transform> object_transform, const std::tuple<double, double, double>& color, const double& opacity);
#endif

    virtual ~VtkObject();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

private:
    const std::string log_name_ = "VtkObject";

    std::unique_ptr<RobotsIO::Utils::Transform> transform_;

    std::unique_ptr<VtkMeshOBJ> vtk_mesh_;
};

#endif /* ROBOTSVIZ_VTJOBJECT_H */
