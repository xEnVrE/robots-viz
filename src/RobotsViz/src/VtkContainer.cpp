/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkUpdateHandler.h>

using namespace RobotsViz;


VtkContainer::VtkContainer(const int& period, const int& width, const int& height, const bool& blocking) :
    period_(period),
    blocking_(blocking)
{
    /* Configure axes. */
    axes_ = vtkSmartPointer<vtkAxesActor>::New();
    orientation_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientation_widget_->SetOrientationMarker(axes_);

    /* Configure camera. */
    vtk_camera_ = vtkSmartPointer<vtkCamera>::New();
    vtk_camera_->SetPosition(0.0, 0.0, 0.5);
    vtk_camera_->SetViewUp(-1.0, 0.0, -1.0);

    /* Configure interactor style. */
    interactor_style_ = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
    interactor_style_->SetCurrentStyleToTrackballCamera();

    /* Configure renderer. */
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetActiveCamera(vtk_camera_);
    renderer_->SetBackground(0.8, 0.8, 0.8);

    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_->AddRenderer(renderer_);
    render_window_->SetSize(width, height);

    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor_->SetRenderWindow(render_window_);
    render_window_interactor_->SetInteractorStyle(interactor_style_);

    /* Configure orientation widget. */
    orientation_widget_->SetInteractor(render_window_interactor_);
    orientation_widget_->SetEnabled(1);
    orientation_widget_->InteractiveOn();
}


VtkContainer::~VtkContainer()
{}


void VtkContainer::add_content(const std::string& key, std::unique_ptr<VtkContent> content)
{
    contents_.emplace(key, std::move(content));
}


void VtkContainer::run()
{
    /* Add contents to the renderer. */
    for (auto& content : contents_)
        content.second->add_to_renderer(*renderer_);

    /* Initialize renderer. */
    render_window_->Render();
    render_window_interactor_->Initialize();
    render_window_interactor_->CreateRepeatingTimer(period_);

    /* Setup update handler. */
    vtkSmartPointer<vtkUpdateHandler> update_handler = vtkSmartPointer<vtkUpdateHandler>::New();
    update_handler->set_container(this);
    render_window_interactor_->AddObserver(vtkCommand::TimerEvent, update_handler);

    /* Start the actual interaction .*/
    render_window_interactor_->Start();
}

void VtkContainer::update()
{
    for (auto& content : contents_)
        content.second->update(blocking_);
}
