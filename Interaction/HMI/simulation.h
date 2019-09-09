#ifndef SIMULATION_H
#define SIMULATION_H

#include <QMainWindow>
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"

class Simulation
{
public:
    Simulation();

    void Update();
};

#endif // SIMULATION_H
