/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkPointCloud.h>
#include <RobotsViz/poisson_disk_sampling.h>

#include <vtkProperty.h>

#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace RobotsViz;


VtkPointCloud::VtkPointCloud(std::unique_ptr<PointCloudSource> source) :
    source_(std::move(source))
{
    /* Configure VTK. */
    points_ = vtkSmartPointer<vtkPoints>::New();

    polydata_ = vtkSmartPointer<vtkPolyData>::New();
    polydata_->SetPoints(points_);

    glyph_filter_ = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyph_filter_->SetInputData(polydata_);
    glyph_filter_->Update();

    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(glyph_filter_->GetOutputPort());

    actor_ = vtkSmartPointer<vtkActor>::New();
    actor_->SetMapper(mapper_);
    actor_->GetProperty()->SetPointSize(2);

    frame_ = std::unique_ptr<VtkReferenceFrame>(new VtkReferenceFrame(1.0));
    frame_->set_visibility(false);
}


VtkPointCloud::~VtkPointCloud()
{}


void VtkPointCloud::add_to_renderer(vtkRenderer& renderer)
{
    renderer.AddActor(actor_);
    frame_->add_to_renderer(renderer);
}


bool VtkPointCloud::update(const bool& blocking)
{
    if (!source_->freeze(blocking))
        return false;

    MatrixXd points = source_->points();
    Matrix<unsigned char, Dynamic, Dynamic> colors = source_->colors();

    /* Set new points and colors. */
    set_points(points);
    set_colors(colors);

    /* Set new pose. */
    bool valid_pose;
    Transform<double, 3, Affine> pose;
    std::tie(valid_pose, pose) = source_->pose();
    if (valid_pose)
    {
        frame_->set_transform(pose);
        frame_->update(blocking);
    }

    return true;
}


void VtkPointCloud::set_points(const Ref<const MatrixXd>& points)
{
    /* Set new points. */
    points_ = vtkSmartPointer<vtkPoints>::New();

    for (std::size_t i = 0; i < points.cols(); i++)
        points_->InsertNextPoint(points(0, i), points(1, i), points(2, i));

    polydata_->SetPoints(points_);
}


void VtkPointCloud::set_colors(const Ref<const Matrix<unsigned char, Dynamic, Dynamic>>& colors)
{
    /* Set new colors. */
    colors_ = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors_->SetNumberOfComponents(3);

    for (std::size_t i = 0; i < colors.cols(); i++)
    {
        std::vector<unsigned char> color = {colors(2, i), colors(1, i), colors(0, i)};

        colors_->InsertNextTypedTuple(color.data());
    }

    polydata_->GetPointData()->SetScalars(colors_);
}


VtkReferenceFrame& VtkPointCloud::get_reference_frame()
{
    return *frame_;
}
