/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKCONTAINER_H
#define ROBOTSVIZ_VTKCONTAINER_H

#include <RobotsViz/VtkContent.h>

#include <unordered_map>

#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <memory>

namespace RobotsViz {
    class VtkContainer;
}

class RobotsViz::VtkContainer
{
public:
    VtkContainer(const double& period, const int& width, const int& height, const bool& blocking);

    VtkContainer(const int& width, const int& height);

    virtual ~VtkContainer();

    void add_content(const std::string& key, std::shared_ptr<VtkContent> content);

    void set_position(const std::size_t& x, const std::size_t& y);

    void initialize();

    void run();

    void update(); //updateContent() + render()

    void updateContent();

    void render();

    vtkSmartPointer<vtkRenderer> renderer();

    vtkSmartPointer<vtkRenderWindow> render_window();

    vtkSmartPointer<vtkCamera> camera();

    void setOrientationWidgetEnabled(bool enabled);

private:
    VtkContainer(const int& width, const int& height, const bool& online, const bool& blocking);

    std::unordered_map<std::string, std::shared_ptr<VtkContent>> contents_;

    vtkSmartPointer<vtkAxesActor> axes_;

    vtkSmartPointer<vtkCamera> vtk_camera_;

    vtkSmartPointer<vtkInteractorStyleSwitch> interactor_style_;

    vtkSmartPointer<vtkOrientationMarkerWidget> orientation_widget_;

    vtkSmartPointer<vtkRenderer> renderer_;

    vtkSmartPointer<vtkRenderWindow> render_window_;

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

    const bool blocking_;

    const bool online_;

    int period_;
};

#endif /* ROBOTSVIZ_VTKCONTAINER_H */
