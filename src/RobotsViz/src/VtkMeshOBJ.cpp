/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkMeshOBJ.h>
#include <RobotsViz/MeshResources.h>

#include <vtkProperty.h>
#include <vtkTransform.h>

using namespace Eigen;
using namespace RobotsViz;


VtkMeshOBJ::VtkMeshOBJ(const std::string& mesh_path, const bool& use_mesh_resources, const std::tuple<double, double, double>& color, const double& opacity)
{
    /* Initialize mesh reader. */
    reader_ = vtkSmartPointer<vtkOBJResource>::New();
    if (use_mesh_resources)
    {
        /* Use internal mesh resources system. */
        MeshResources mesh_resource(mesh_path);
        reader_->SetFileData(mesh_resource.get_data());
    }
    else
    {
        /* Open file from local file system. */
        reader_->SetFileName(mesh_path.c_str());
    }
    reader_->Update();

    /* Connect to PolyDataMapper. */
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    /* Initialize actor. */
    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);
    mesh_actor_->GetProperty()->SetColor(std::get<0>(color), std::get<1>(color), std::get<2>(color));
    mesh_actor_->GetProperty()->SetOpacity(opacity);
}


VtkMeshOBJ::~VtkMeshOBJ()
{}


void VtkMeshOBJ::add_to_renderer(vtkRenderer& renderer)
{
    renderer.AddActor(mesh_actor_);
}


void VtkMeshOBJ::set_pose(const Transform<double, 3, Affine>& pose)
{
    /* Create vtk transform. */
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    /* Set translation. */
    transform->Translate(pose.translation().data());

    /* Set rotation. */
    AngleAxisd angle_axis(pose.rotation());
    transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                          angle_axis.axis()(0), angle_axis.axis()(1), angle_axis.axis()(2));

    /* Apply transform. */
    mesh_actor_->SetUserTransform(transform);
}
