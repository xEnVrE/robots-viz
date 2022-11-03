/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_POINTCLOUDCAMERA_H
#define ROBOTSVIZ_POINTCLOUDCAMERA_H

#include <RobotsViz/PointCloudSource.h>

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <memory>

namespace RobotsViz {
    class PointCloudCamera;
}


class RobotsViz::PointCloudCamera : public RobotsViz::PointCloudSource
{
public:
    PointCloudCamera(std::unique_ptr<RobotsIO::Camera::Camera> camera, const double& far_plane_ = std::numeric_limits<double>::infinity(), const double& subsampling_radius = -1);

    std::tuple<bool, Eigen::MatrixXd, Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>> points(const bool& blocking) override;

private:
    std::unique_ptr<RobotsIO::Camera::Camera> camera_;

    RobotsIO::Camera::CameraParameters camera_parameters_;

    std::vector<cv::Point> image_coordinates_;

    const double far_plane_;
};

#endif /* ROBOTSVIZ_POINTCLOUDCAMERA_H */
