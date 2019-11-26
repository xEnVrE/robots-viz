/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkReferenceFrame.h>

#include <vtkAxesActor.h>
#include <vtkTransform.h>

using namespace Eigen;
using namespace RobotsViz;


VtkReferenceFrame::VtkReferenceFrame(const double& length)
{
    axes_ = vtkSmartPointer<vtkAxesActor>::New();
    axes_->SetTotalLength(length, length, length);
    axes_->VisibilityOn();
    axes_->SetXAxisLabelText(std::string("").c_str());
    axes_->SetYAxisLabelText(std::string("").c_str());
    axes_->SetZAxisLabelText(std::string("").c_str());
}


VtkReferenceFrame::~VtkReferenceFrame()
{}


void VtkReferenceFrame::add_to_renderer(vtkRenderer& renderer)
{
    renderer.AddActor(this->axes_);
}


bool VtkReferenceFrame::update(const bool& blocking)
{
    /* Create vtk transform. */
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    /* Set translation. */
    transform->Translate(transform_.translation().data());

    /* Set rotation. */
    AngleAxisd angle_axis(transform_.rotation());
    transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                          angle_axis.axis()(0), angle_axis.axis()(1), angle_axis.axis()(2));

    /* Apply transform. */
    axes_->SetUserTransform(transform);

    return true;
}


void VtkReferenceFrame::set_transform(const Eigen::Transform<double, 3, Eigen::Affine>& transform)
{
    transform_ = transform;
}
