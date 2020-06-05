#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ScopedState.h"

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/RRTsharp.h"
#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include "ompl/geometric/planners/rrt/LBTRRT.h"

#include <boost/program_options.hpp>

#include "Planning/OMPL_Path/ompl_obstacle.h"
#include "Planning/OMPL_Path/ompl_space.h"

#include "Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

class OMPL_Planner
{
public:
    OMPL_Planner();

    void ompl_motion_planner(State set_start, State set_end);

    /** Return an optimization objective which attempts to steer the robot
        away from obstacles. */
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);


    OMPL_Space    *getOmplSpace(void){ return _ompl_space; }
    og::PathGeometric *getPath(void){ return _path; }
private:
    og::SimpleSetupPtr _ss;
    og::PathGeometric *_path;
    OMPL_Space    *_ompl_space;
};

#endif // OMPL_PLANNER_H
