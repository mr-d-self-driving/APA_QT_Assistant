#include "ompl_space.h"

OMPL_Space::OMPL_Space()
{
    _obstacle_boundary.setLow(0, BOUNDARY_LEFT);
    _obstacle_boundary.setLow(1, BOUNDARY_DOWN);

    _obstacle_boundary.setHigh(0, BOUNDARY_RIGHT);
    _obstacle_boundary.setHigh(1, BOUNDARY_TOP);

    _boundary_size_x = BOUNDARY_X_SIZE;
    _boundary_size_y = BOUNDARY_Y_SIZE;

    _space = std::make_shared<HC_ReedsSheppStateSpace>(0.2, 0.215, 0.02);
//    _space = std::make_shared<ob::ReedsSheppStateSpace>(5.0);
//    _space = std::make_shared<ob::DubinsStateSpace>();
    _space->as<ob::SE2StateSpace>()->setBounds(_obstacle_boundary);
    _si = std::make_shared<ob::SpaceInformation>(_space);
    _ompl_obstacle = std::make_shared<OMPL_Obstacle>(_si);
    _si->setStateValidityChecker(_ompl_obstacle);
    _si->setStateValidityCheckingResolution(0.005);
    _si->setup();
}


