/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkContainer.h>

#include <vtkInteractorStyleTrackballCamera.h>

namespace RobotsViz {
    class vtkManualUpdateHandler;
}


class RobotsViz::vtkManualUpdateHandler : public vtkInteractorStyleTrackballCamera
{
public:
    static vtkManualUpdateHandler* New();

    vtkTypeMacro(vtkManualUpdateHandler, vtkInteractorStyleTrackballCamera);

    void set_container(VtkContainer* container);

    virtual void OnKeyPress() override;

private:
    VtkContainer* container_;
};
