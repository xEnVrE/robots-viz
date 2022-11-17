/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkContainer.h>
#include <RobotsViz/VtkManualUpdateHandler.h>
#include <RobotsViz/VtkUpdateHandler.h>

using namespace RobotsViz;


VtkContainer::VtkContainer(const double& period, const int& width, const int& height, const bool& blocking) :
    VtkContainer(width, height, true, blocking)
{
    period_ = period;
}


VtkContainer::VtkContainer(const int& width, const int& height) :
    VtkContainer(width, height, false, false)
{}


VtkContainer::VtkContainer(const int& width, const int& height, const bool& online, const bool& blocking) :
    blocking_(blocking),
    online_(online)
{
    /* Configure axes. */
    axes_ = vtkSmartPointer<vtkAxesActor>::New();
    orientation_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientation_widget_->SetOrientationMarker(axes_);

    /* Configure camera. */
    vtk_camera_ = vtkSmartPointer<vtkCamera>::New();
    vtk_camera_->SetPosition(0.0, 0.0, 0.5);
    vtk_camera_->SetViewUp(-1.0, 0.0, -1.0);

    /* Configure renderer. */
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetActiveCamera(vtk_camera_);
    renderer_->SetBackground(0.8, 0.8, 0.8);

    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_->AddRenderer(renderer_);
    render_window_->SetSize(width, height);

    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor_->SetRenderWindow(render_window_);

    /* Configure interactor style. */
    if (online_)
    {
        interactor_style_ = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        interactor_style_->SetCurrentStyleToTrackballCamera();

        render_window_interactor_->SetInteractorStyle(interactor_style_);
    }
    else
    {
        vtkSmartPointer<vtkManualUpdateHandler> manual_update_handler = vtkSmartPointer<vtkManualUpdateHandler>::New();
        manual_update_handler->SetCurrentRenderer(renderer_);
        manual_update_handler->set_container(this);

        render_window_interactor_->SetInteractorStyle(manual_update_handler);
    }

    /* Configure orientation widget. */
    orientation_widget_->SetInteractor(render_window_interactor_);
    orientation_widget_->SetEnabled(1);
    orientation_widget_->InteractiveOn();
}


VtkContainer::~VtkContainer()
{}


void VtkContainer::add_content(const std::string& key, std::shared_ptr<VtkContent> content)
{
    contents_.emplace(key, content);
}


void VtkContainer::set_position(const std::size_t& x, const std::size_t& y)
{
    render_window_->SetPosition(x, y);
}


void VtkContainer::initialize()
{
    /* Add contents to the renderer. */
    for (auto& content : contents_)
        content.second->add_to_renderer(*renderer_);
}


void VtkContainer::run()
{

    initialize();
    /* Initialize renderer. */
    render_window_->Render();
    render_window_interactor_->Initialize();

    /* Setup online update handler. */
    if (online_)
    {
        render_window_interactor_->CreateRepeatingTimer(period_);

        vtkSmartPointer<vtkUpdateHandler> update_handler = vtkSmartPointer<vtkUpdateHandler>::New();
        update_handler->set_container(this);
        render_window_interactor_->AddObserver(vtkCommand::TimerEvent, update_handler);
    }

    /* Start the actual interaction .*/
    render_window_interactor_->Start();
}

void VtkContainer::update()
{
    updateContent();
    render();
}

void VtkContainer::updateContent()
{
    /* Update each content .*/
    for (auto& content : contents_)
        content.second->update(blocking_);
}

void VtkContainer::render()
{
    /* Trigger the renderer. */
    render_window_->Render();
}

vtkSmartPointer<vtkRenderer> VtkContainer::renderer()
{
    return renderer_;
}

vtkSmartPointer<vtkRenderWindow> VtkContainer::render_window()
{
    return render_window_;
}

vtkSmartPointer<vtkCamera> VtkContainer::camera()
{
    return vtk_camera_;
}

void VtkContainer::setOrientationWidgetEnabled(bool enabled)
{
    orientation_widget_->SetEnabled(enabled);
}
