#ifndef OMPL_SPACE_H
#define OMPL_SPACE_H

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>
#include <fstream>

#include "Planning/OMPL_Path/ompl_obstacle.h"
#include "Planning/HC_CC_StateSpace/hc_reeds_shepp_state_space.h"


namespace ob = ompl::base;

class OMPL_Space
{
public:
    OMPL_Space();

    ob::RealVectorBounds getObstacleBoundary(){ return _obstacle_boundary; }
    uint16_t getBoundarySizeX(){ return _boundary_size_x; }
    uint16_t getBoundarySizeY(){ return _boundary_size_y; }

    ob::StateSpacePtr getSpace() { return _space; }
    ob::SpaceInformationPtr getSpaceInformation(void) { return _si; }
    std::shared_ptr<OMPL_Obstacle> getOmplObstacle(void){ return _ompl_obstacle; }
private:
    ob::RealVectorBounds _obstacle_boundary = ob::RealVectorBounds(2);

    uint16_t _boundary_size_x;
    uint16_t _boundary_size_y;

    ob::StateSpacePtr       _space;
    ob::SpaceInformationPtr _si;

    std::shared_ptr<OMPL_Obstacle> _ompl_obstacle;
};

#endif // OMPL_SPACE_H
