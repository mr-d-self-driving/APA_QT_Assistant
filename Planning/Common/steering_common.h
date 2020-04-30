#ifndef STEERINGCOMMON_H
#define STEERINGCOMMON_H

namespace steering {

/**
 * @brief The State struct: Description of a kinematic car's state
 */
struct State
{
    /**
     * @brief x :Position in x of the robot
     */
    double x;

    /**
     * @brief y :Position in y of the robot
     */
    double y;

    /**
     * @brief theta :Orientation of the robot
     */
    double theta;

    /**
     * @brief kappa :Curvature at position (x,y)
     */
    double kappa;

    /**
     * @brief d :Driving direction {-1,0,1}
     */
    double d;
};

/**
 * @brief The Control struct :Description of a path segment with its corresponding control inputs
 */
struct Control
{
    /**
     * @brief delta_s :Signed arc length of a segment
     */
    double delta_s;

    /**
     * @brief kappa :Curvature at the beginning of a segment
     */
    double kappa;

    /**
     * @brief sigma :Sharpness (derivative of curvature with respect to arc length) of a segment
     */
    double sigma;
};

}

#endif // STEERINGCOMMON_H
