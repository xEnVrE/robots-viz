/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkManualUpdateHandler.h>

#include <vtkObjectFactory.h>

using namespace RobotsViz;


vtkStandardNewMacro(vtkManualUpdateHandler);


void vtkManualUpdateHandler::set_container(VtkContainer* container)
{
    container_ = container;
}


void vtkManualUpdateHandler::OnKeyPress()
{
    /* Get the pressed key. */
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    /* If right pressed update the view. */
    if(key == "Right")
    {
        container_->update();
    }

    /* Forward events. */
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}
