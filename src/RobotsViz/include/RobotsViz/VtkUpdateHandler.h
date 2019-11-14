/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSVIZ_VTKUPDATEHANDLER_H
#define ROBOTSVIZ_VTKUPDATEHANDLER_H

#include <RobotsViz/VtkContainer.h>

#include <vtkCommand.h>

namespace RobotsViz {
    class vtkUpdateHandler;
}


class RobotsViz::vtkUpdateHandler : public vtkCommand
{
public:
    static vtkUpdateHandler* New();

    void set_container(VtkContainer* container);

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData));

private:
    VtkContainer* container_;
};

#endif /* ROBOTSVIS_VTKUPDATEHANDLER_H */
