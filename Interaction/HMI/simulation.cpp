#include "simulation.h"

Simulation::Simulation()
{

}

void Simulation::Update(VehicleController *c,MessageManager *m)
{
    if(c->APAEnable)
    {
        m->WheelSpeedRearLeft = c->Velocity;
    }
}
