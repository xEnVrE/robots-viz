/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Eigen/Dense>

#include <RobotsViz/MeshResources.h>
#include <RobotsViz/VtkOrientationCube.h>

#include <unsupported/Eigen/MatrixFunctions>

#include <vtkProperty.h>
#include <vtkTransform.h>

using namespace Eigen;
using namespace RobotsViz;


VtkOrientationCube::VtkOrientationCube(std::unique_ptr<RobotsIO::Utils::SpatialVelocity> velocity, const double& sample_time, const std::tuple<double, double, double>& color, const double& opacity) :
    velocity_(std::move(velocity)),
    sample_time_(sample_time)
{
    /* Load cube mesh. */
    auto cube_mesh = MeshResources("cube.obj");

    /* Load as VTK mesh. */
    reader_ = vtkSmartPointer<vtkOBJResource>::New();
    reader_->SetFileData(cube_mesh.as_string());
    reader_->Update();

    /* Connect to PolyDataMapper. */
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    /* Initialize actor. */
    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);
    mesh_actor_->GetProperty()->SetColor(std::get<0>(color), std::get<1>(color), std::get<2>(color));
    mesh_actor_->GetProperty()->SetOpacity(opacity);

    /* FIXME: This will be removed, for testing only. */

    Vector3d axis(0.443360717982344, -0.0995417780829392, -0.890798915674165);
    AngleAxisd aa(1.8710993448310, axis);
    transform_.linear() = aa.toRotationMatrix();

    /* Create vtk transform. */
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    /* Set translation. */
    transform->Translate(transform_.translation().data());

    /* Set rotation. */
    AngleAxisd angle_axis(transform_.rotation());
    transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                          angle_axis.axis()(0), angle_axis.axis()(1), angle_axis.axis()(2));

    /* Apply transform. */
    mesh_actor_->SetUserTransform(transform);

    /* */
}


VtkOrientationCube::~VtkOrientationCube()
{}


void VtkOrientationCube::add_to_renderer(vtkRenderer& renderer)
{
    renderer.AddActor(mesh_actor_);
}


bool VtkOrientationCube::update(const bool& blocking)
{
    bool valid_velocity = false;
    Vector3d angular_velocity;
    double elapsed;
    std::tie(valid_velocity, std::ignore, std::ignore, angular_velocity, elapsed) = velocity_->velocity(blocking);

    if (!valid_velocity)
        return false;

    Vector3d angular_velocity_body = angular_velocity;

    /* Propagate the orientation using the current angular velocity. */
    Matrix3d angular_screw;
    angular_screw << 0.0, -angular_velocity_body(2), angular_velocity_body(1),
                     angular_velocity_body(2), 0.0, -angular_velocity_body(0),
                     -angular_velocity_body(1), angular_velocity_body(0), 0.0;
    Matrix3d delta = (angular_screw * elapsed).exp();
    transform_.linear() = delta * transform_.rotation();

    /* Create vtk transform. */
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    /* Set translation. */
    transform->Translate(transform_.translation().data());

    /* Set rotation. */
    AngleAxisd angle_axis(transform_.rotation());
    transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                          angle_axis.axis()(0), angle_axis.axis()(1), angle_axis.axis()(2));

    /* Apply transform. */
    mesh_actor_->SetUserTransform(transform);

    return true;
}
