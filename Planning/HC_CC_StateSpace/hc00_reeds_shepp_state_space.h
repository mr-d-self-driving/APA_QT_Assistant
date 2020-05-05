#ifndef HC00_REEDSSHEPPSTATESPACE_H
#define HC00_REEDSSHEPPSTATESPACE_H

#include <limits>
#include <memory>
#include <vector>

#include "Planning/Interface/hc_cc_state_space.h"
#include "Planning/Path/hc_cc_circle.h"

#include "Common/Configure/Configs/system_config.h"

using namespace std;
using namespace steering;

#define CC_REGULAR false

class HC00_ReedsSheppStateSpace : public HC_CC_StateSpace
{
public:
    /**
     * @brief HC00_ReedsSheppStateSpace Constructor
     * @param kappa :the max curvature of the path
     * @param sigma :the max sharpness of the path
     * @param discretization :the discretization step
     */
    HC00_ReedsSheppStateSpace(double kappa, double sigma, double discretization = 0.1);

    /**
     * @brief Destructor
     */
    virtual ~HC00_ReedsSheppStateSpace();

private:

    /**
     * @brief class that contains functions to compute the families
     */
    class HC00_ReedsShepp;

    /**
     * @brief unique pointer on class with families
     */
    unique_ptr<HC00_ReedsShepp> _hc00_reeds_shepp;

    /**
     * @brief Param of a rs-circle
     */
    HC_CC_Circle_Param _rs_circle_param;
};

#endif // HC00_REEDSSHEPPSTATESPACE_H
