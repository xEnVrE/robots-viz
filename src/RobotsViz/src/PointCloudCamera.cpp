/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/PointCloudCamera.h>
#include <RobotsViz/poisson_disk_sampling.h>

using namespace RobotsViz;
using namespace RobotsIO::Camera;
using namespace Eigen;


PointCloudCamera::PointCloudCamera(std::unique_ptr<Camera> camera, const double& far_plane, const double& subsampling_radius) :
    camera_(std::move(camera)),
    far_plane_(far_plane)
{
    /* Cache camera parameters. */
    bool valid_parameters = false;
    std::tie(valid_parameters, camera_parameters_) = camera_->parameters();

    /* Cache image coordinates. */
    if (subsampling_radius > 0)
    {
        /* Subsample image coordinates if require. */
        auto min_coordinates = std::array<float, 2>{{float(0.0), float(0.0)}};
        auto max_coordinates = std::array<float, 2>{{float(camera_parameters_.width()), float(camera_parameters_.height())}};
        auto coordinates = thinks::PoissonDiskSampling(float(subsampling_radius), min_coordinates, max_coordinates);

        for (const auto& pixel : coordinates)
            image_coordinates_.push_back(cv::Point(pixel[0], pixel[1]));
    }
    else
    {
        /* Take all the image coordinates otherwise. */
        for (std::size_t u = 0; u < camera_parameters_.width(); u++)
            for (std::size_t v = 0; v < camera_parameters_.height(); v++ )
                image_coordinates_.push_back(cv::Point(u, v));
    }
}



bool PointCloudCamera::freeze(const bool& blocking)
{
    /* Step frame in case of an offline camera. */
    if (!camera_->step_frame())
        return false;

    bool valid_data = false;

    /* Get RGB. */
    cv::Mat rgb;
    std::tie(valid_data, rgb) = camera_->rgb(blocking);
    if (!valid_data)
        return false;

    /* Get D. */
    valid_data = false;
    MatrixXf depth;
    std::tie(valid_data, depth) = camera_->depth(blocking);
    if (!valid_data)
        return false;

    /* Get camera pose. */
    valid_data = false;
    Transform<double, 3, Affine> pose;
    std::tie(valid_data, pose) = camera_->pose(blocking);
    if (!valid_data)
        return false;

    /* Find valid points. */
    VectorXi valid_points(image_coordinates_.size());
    for (std::size_t i = 0; i < valid_points.size(); i++)
    {
        valid_points(i) = 0;

        float depth_u_v = depth(image_coordinates_.at(i).y, image_coordinates_.at(i).x);

        if ((depth_u_v > 0) && (depth_u_v < far_plane_))
            valid_points(i) = 1;
    }

    std::size_t num_valids = valid_points.sum();
    if (num_valids == 0)
        return false;

    /* Get deprojection matrix. */
    valid_data = false;
    MatrixXd deprojection_matrix;
    std::tie(valid_data, deprojection_matrix) = camera_->deprojection_matrix();
    if (!valid_data)
        return false;

    /* Store only valid 3D points. */
    points_ = MatrixXd(3, num_valids);
    colors_ = Matrix<unsigned char, -1, -1>(3, num_valids);
    for (std::size_t i = 0, j = 0; i < image_coordinates_.size(); i++)
    {
        if (valid_points(i) == 1)
        {
            const int& u = image_coordinates_.at(i).x;
            const int& v = image_coordinates_.at(i).y;

            points_.col(j) = deprojection_matrix.col(u * camera_parameters_.height() + v) * depth(v, u);

            cv::Vec3b cv_color = rgb.at<cv::Vec3b>(image_coordinates_.at(i));
            Matrix<unsigned char, -1, -1> color(3, 1);
            colors_(0, j) = cv_color[0];
            colors_(1, j) = cv_color[1];
            colors_(2, j) = cv_color[2];

            j++;
        }
    }

    points_ = pose * points_.colwise().homogeneous();

    return true;
}



MatrixXd PointCloudCamera::points()
{
    return points_;
}


Matrix<unsigned char, Dynamic, Dynamic> PointCloudCamera::colors()
{
    return colors_;
}
