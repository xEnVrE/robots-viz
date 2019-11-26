/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKREFERENCEFRAME_H
#define ROBOTSVIZ_VTKREFERENCEFRAME_H

#include <RobotsViz/VtkContent.h>

#include <Eigen/Dense>

#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>

#include <string>

namespace RobotsViz {
    class VtkReferenceFrame;
}


class RobotsViz::VtkReferenceFrame : public VtkContent
{
public:
    VtkReferenceFrame(const double& length);

    virtual ~VtkReferenceFrame();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

    void set_transform(const Eigen::Transform<double, 3, Eigen::Affine>& transform);

private:
    const std::string log_name_ = "VtkReferenceFrame";

    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    vtkSmartPointer<vtkAxesActor> axes_;
};

#endif /* ROBOTSVIZ_VTKREFERENCEFRAME_H */
