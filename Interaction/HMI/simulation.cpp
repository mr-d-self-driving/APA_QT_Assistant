#include "simulation.h"

Simulation::Simulation()
{

}

void Simulation::Update(VehicleController *c,MessageManager *m)
{
    if(c->APAEnable)
    {
        m->WheelSpeedRearLeft  = c->Velocity;
        m->WheelSpeedRearRight = c->Velocity;

        m->SteeringAngle = c->SteeringAngle;
        m->SteeringAngleRate = c->SteeringAngleRate;

        m->Gear = c->Gear;

        if(Drive == c->Gear){
            m->WheelSpeedDirection = Forward;
        }
        else if (Reverse == c->Gear) {
            m->WheelSpeedDirection = Backward;
        }
        else {
            m->WheelSpeedDirection = StandStill;
        }
    }
}
