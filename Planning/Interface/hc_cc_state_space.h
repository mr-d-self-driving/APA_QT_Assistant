#ifndef HC_CC_STATESPACE_H
#define HC_CC_STATESPACE_H

#include <vector>
#include <cmath>

#include "Planning/Common/steering_common.h"

using namespace std;
using namespace steering;

class HC_CC_StateSpace
{
public:
    /**
     * @brief HC_CC_StateSpace :Constructor
     * @param kappa : the curvature of the circle
     * @param sigma : the sharpness of the clothoid
     * @param discretization : the coefficient of discretization
     */
    HC_CC_StateSpace(double kappa, double sigma, double discretization);

    /**
     * @brief getControls :Virtual function that return controls of the shortest path from stat1 to state2
     * @param state1 :the start state
     * @param state2 :the end state
     * @return the controls
     */
    virtual vector<Control> getControls(const State &state1, const State &state2) const = 0;

    vector<State> getPath(const State &state1, const State &state2) const;
private:

    /**
     * @brief _kappa :the curvature of clothoid
     */
    double _kappa;

    /**
     * @brief _sigma :the sharpness of clothoid
     */
    double _sigma;

    /**
     * @brief _discretization :the coefficient of the discretization path
     */
    double _discretization;

};

#endif // HC_CC_STATESPACE_H
