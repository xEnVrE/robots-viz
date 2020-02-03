/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKORIENTATIONCUBE_H
#define ROBOTSVIZ_VTKORIENTATIONCUBE_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/SpatialVelocity.h>

#include <RobotsViz/MeshResources.h>
#include <RobotsViz/VtkContent.h>
#include <RobotsViz/vtkOBJResource.h>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>

#include <memory>

namespace RobotsViz {
    class VtkOrientationCube;
}


class RobotsViz::VtkOrientationCube : public RobotsViz::VtkContent
{
public:
    VtkOrientationCube(std::unique_ptr<RobotsIO::Utils::SpatialVelocity> velocity, const double& sample_time, const std::tuple<double, double, double>& color, const double& opacity);

    virtual ~VtkOrientationCube();

    void add_to_renderer(vtkRenderer& renderer);

    bool update(const bool& blocking) override;

private:
    Eigen::Transform<double, 3, Eigen::Affine> transform_ = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    std::unique_ptr<RobotsIO::Utils::SpatialVelocity> velocity_;

    const double sample_time_;

    vtkSmartPointer<vtkOBJResource> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;
};

#endif /* ROBOTSVIZ_VTKMESHOBJ_H */
