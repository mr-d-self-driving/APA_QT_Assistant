#include "ompl_obstacle.h"


/**
 * @brief isValid
 * @param state
 * @return Returns whether the given state's position overlaps the obstacle
 */
bool OMPL_Obstacle::isValid(const ob::State* state) const
{
    return getObstacleIndex(state) == 0;
}

/**
 * @brief clearance
 * @param state
 * @return Returns the distance from the given state's position to the boundary of the obstacle.
 */
double OMPL_Obstacle::clearance(const ob::State* state) const
{
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const auto* state2D =
        state->as<ob::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    // Distance formula between two points, offset by the circle's
    // radius
    return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
}

/**
 * @brief getDiscreteStateIndex
 * @param x :the x axis continuous position
 * @param y :the y axis continuous position
 * @param ind_x :the x axis discrete state index
 * @param ind_y :the y axis discrete state index
 */
void OMPL_Obstacle::getDiscreteStateIndex(double x, double y,
                                          int16_t &ind_x, int16_t &ind_y)const
{
    if(x < BOUNDARY_LEFT || x > BOUNDARY_RIGHT || y < BOUNDARY_DOWN || y > BOUNDARY_TOP)
    {
        return;
    }

    ind_x = static_cast<int16_t>(std::floor( (x + BOUNDARY_RIGHT) * BOUNDARY_X_SIZE / (BOUNDARY_RIGHT - BOUNDARY_LEFT)));
    ind_x = ind_x < BOUNDARY_X_SIZE ? ind_x : BOUNDARY_X_SIZE;

    ind_y = static_cast<int16_t>(std::floor( (y + BOUNDARY_TOP) * BOUNDARY_Y_SIZE / (BOUNDARY_TOP - BOUNDARY_DOWN)));
    ind_y = ind_y < BOUNDARY_Y_SIZE ? ind_y : BOUNDARY_Y_SIZE;
}


/**
 * @brief getObstacleIndex
 * @param state :the input state
 * @return return the onstacle array value
 */
uint8_t OMPL_Obstacle::getObstacleIndex(const ob::State* state)const
{
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const auto* stateE2 = state->as<ob::SE2StateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = stateE2->getX();
    double y = stateE2->getY();

    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);

    // Distance formula between two points, offset by the circle's
    // radius
    return ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index];
}
/**
 * @brief base on the current point return the grid colour value
 * @param x :the x axis grid position
 * @param y :the y axis grid position
 * @return the colour value
 */
uint8_t OMPL_Obstacle::getCurrentGridColour(double x, double y)const
{
    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);
    return ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index];
}

/**
 * @brief base on the position reverse the colour value
 * @param x :the x axis position
 * @param y :the y axis position
 */
void OMPL_Obstacle::GridReverse(double x, double y)
{
    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);
    ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index] = ~
    ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index];
}

/**
 * @brief setGridWhite
 * @param x
 * @param y
 */
void OMPL_Obstacle::setGridWhite(double x, double y)
{
    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);
    ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index] = 0;
}

/**
 * @brief setGridBlack
 * @param x
 * @param y
 */
void OMPL_Obstacle::setGridBlack(double x, double y)
{
    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);
    ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index] = 255;
}

/**
 * @brief setGridValue
 * @param x
 * @param y
 * @param val
 */
void OMPL_Obstacle::setGridValue(double x, double y, uint8_t val)
{
    int16_t x_index, y_index;
    this->getDiscreteStateIndex(x, y, x_index, y_index);
    ObstacleArray[x_index * BOUNDARY_Y_SIZE + y_index] = val;
//    for (int16_t i = -1; i < 1; i++) {
//        for (int16_t j = -1; j < 1; j++){
//            int16_t temp_x = x_index + i < 0 ? 0 : x_index + i >= BOUNDARY_X_SIZE ? BOUNDARY_X_SIZE - 1 : x_index + i;
//            int16_t temp_y = y_index + j < 0 ? 0 : y_index + j >= BOUNDARY_Y_SIZE ? BOUNDARY_Y_SIZE - 1 : y_index + j;
//            ObstacleArray[temp_x][temp_y] = val;
//        }
//    }
}
