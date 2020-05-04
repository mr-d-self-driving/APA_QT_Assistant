#ifndef HC00_REEDSSHEPPSTATESPACE_H
#define HC00_REEDSSHEPPSTATESPACE_H

#include "Planning/Interface/hc_cc_state_space.h"

class HC00_ReedsSheppStateSpace : public HC_CC_StateSpace
{
public:

//    HC00_ReedsSheppStateSpace();

    virtual ~HC00_ReedsSheppStateSpace();

    HC00_ReedsSheppStateSpace(double kappa, double sigma, double discretization = 0.1);
};

#endif // HC00_REEDSSHEPPSTATESPACE_H
