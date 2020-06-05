#ifndef OMPL_OBSTACLE_H
#define OMPL_OBSTACLE_H

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

//#include "Planning/OMPL_Path/ompl_space.h"

namespace ob = ompl::base;

/**
 * @brief 车位边界配置
 */
#define BOUNDARY_LEFT  (-18.0)
#define BOUNDARY_RIGHT ( 18.0)
#define BOUNDARY_TOP   ( 12.0)
#define BOUNDARY_DOWN  (-12.0)

#define BOUNDARY_X_SIZE     (36)
#define BOUNDARY_Y_SIZE     (24)

class OMPL_Obstacle : public ob::StateValidityChecker
{
public:
    OMPL_Obstacle(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {
        for(int i = 0; i< BOUNDARY_X_SIZE; i++)
        {
            for (int j = 0; j < BOUNDARY_Y_SIZE; j++)
            {
                ObstacleArray[i * BOUNDARY_Y_SIZE + j] = 0;
            }
        }
    }

    /**
     * @brief isValid
     * @param state
     * @return Returns whether the given state's position overlaps the obstacle
     */
    bool isValid(const ob::State* state) const override;

    /**
     * @brief clearance
     * @param state
     * @return Returns the distance from the given state's position to the boundary of the obstacle.
     */
    double clearance(const ob::State* state) const override;

    /**
     * @brief getDiscreteStateIndex
     * @param x :the x axis continuous position
     * @param y :the y axis continuous position
     * @param ind_x :the x axis discrete state index
     * @param ind_y :the y axis discrete state index
     */
    void getDiscreteStateIndex(double x, double y,
                               int16_t &ind_x, int16_t &ind_y)const;

    /**
     * @brief getObstacleIndex
     * @param state :the input state
     * @return return the onstacle array value
     */
    uint8_t getObstacleIndex(const ob::State* state)const;

    /**
     * @brief getObstacleIndex
     * @param state :the input state
     * @return return the obstacle result
     */
    bool ObstacleChecker(const ob::State* state)const;

    /**
     * @brief base on the current point return the grid colour value
     * @param x :the x axis grid position
     * @param y :the y axis grid position
     * @return the colour value
     */
    uint8_t getCurrentGridColour(double x, double y)const;

    /**
     * @brief base on the position reverse the colour value
     * @param x :the x axis position
     * @param y :the y axis position
     */
    void GridReverse(double x, double y);

    /**
     * @brief setGridWhite
     * @param x
     * @param y
     */
    void setGridWhite(double x, double y);

    /**
     * @brief setGridBlack
     * @param x
     * @param y
     */
    void setGridBlack(double x, double y);

    /**
     * @brief setGridValue
     * @param x
     * @param y
     * @param val
     */
    void setGridValue(double x, double y, uint8_t val);

    uint8_t *getObstacleArray(){ return ObstacleArray; }
private:
    uint8_t ObstacleArray[BOUNDARY_X_SIZE * BOUNDARY_Y_SIZE];
};




#endif // OMPL_OBSTACLE_H
