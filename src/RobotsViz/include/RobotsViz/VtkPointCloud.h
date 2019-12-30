/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKPOINTCLOUD_H
#define ROBOTSVIZ_VTKPOINTCLOUD_H

#include <Eigen/Dense>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <RobotsViz/VtkContent.h>

#include <vtkActor.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>

#include <limits>
#include <memory>
#include <string>

namespace RobotsViz {
    class VtkPointCloud;
}


class RobotsViz::VtkPointCloud : public RobotsViz::VtkContent
{
public:
    VtkPointCloud(std::unique_ptr<RobotsIO::Camera::Camera> camera, const double& far_plane_ = std::numeric_limits<double>::infinity());

    virtual ~VtkPointCloud();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

private:
    void set_points(const Eigen::Ref<const Eigen::MatrixXd>& points);

    void set_colors(const Eigen::Ref<const Eigen::VectorXi>& valid_coordinates, const cv::Mat& rgb_image);

    vtkSmartPointer<vtkActor> actor_;

    vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkPolyData> polydata_;

    vtkSmartPointer<vtkPoints> points_;

    vtkSmartPointer<vtkUnsignedCharArray> colors_;

    std::unique_ptr<RobotsIO::Camera::Camera> camera_;

    RobotsIO::Camera::CameraParameters camera_parameters_;

    std::vector<cv::Point> image_coordinates_;

    const double far_plane_;

    const std::string log_name_ = "VtkPointCloud";
};

#endif /* VTKPOINTCLOUD_H */
