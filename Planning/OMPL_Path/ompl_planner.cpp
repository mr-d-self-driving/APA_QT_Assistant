#include "ompl_planner.h"

OMPL_Planner::OMPL_Planner()
{
    _ompl_space = new OMPL_Space();
    _path       = new og::PathGeometric(_ompl_space->getSpaceInformation());
}

/** Return an optimization objective which attempts to steer the robot
    away from obstacles. */
ob::OptimizationObjectivePtr OMPL_Planner::getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

void OMPL_Planner::ompl_motion_planner(State set_start, State set_end)
{
    ob::ScopedState<> start(_ompl_space->getSpace()), goal(_ompl_space->getSpace());

    start[0] = set_start.x;
    start[1] = set_start.y;
    start[2] = set_start.psi;

    goal[0] = set_end.x;
    goal[1] = set_end.y;
    goal[2] = set_end.psi;

    _ss = std::make_shared<og::SimpleSetup>(_ompl_space->getSpaceInformation());

    _ss->setStartAndGoalStates(start, goal);

//    _ss->setPlanner( std::make_shared<og::RRTstar>(_ompl_space->getSpaceInformation()) );

    _ss->setup();

    _ss->print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = _ss->solve(20.0);

    if(solved)
    {
        _ss->simplifySolution();
        *_path = _ss->getSolutionPath();
        _path->interpolate(1000);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}
