/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsViz/VtkUpdateHandler.h>

using namespace RobotsViz;


vtkUpdateHandler* vtkUpdateHandler::New()
{
    return new vtkUpdateHandler;
}


void vtkUpdateHandler::set_container(VtkContainer* container)
{
    container_ = container;
}


void vtkUpdateHandler::Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData))
{
    container_->update();

    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
    iren->GetRenderWindow()->Render();
}
