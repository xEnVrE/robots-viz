/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKPOINTCLOUD_H
#define ROBOTSVIZ_VTKPOINTCLOUD_H

#include <Eigen/Dense>

#include <RobotsViz/PointCloudSource.h>
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
    VtkPointCloud(std::unique_ptr<RobotsViz::PointCloudSource> source);

    virtual ~VtkPointCloud();

    void add_to_renderer(vtkRenderer& renderer) override;

    bool update(const bool& blocking) override;

private:
    void set_points(const Eigen::Ref<const Eigen::MatrixXd>& points);

    void set_colors(const Eigen::Ref<const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>& colors);

    vtkSmartPointer<vtkActor> actor_;

    vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkPolyData> polydata_;

    vtkSmartPointer<vtkPoints> points_;

    vtkSmartPointer<vtkUnsignedCharArray> colors_;

    std::unique_ptr<RobotsViz::PointCloudSource> source_;

    const std::string log_name_ = "VtkPointCloud";
};

#endif /* VTKPOINTCLOUD_H */
