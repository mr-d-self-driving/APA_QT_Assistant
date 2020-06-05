#ifndef SH2STATESPACE_H
#define SH2STATESPACE_H

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"

namespace ob = ompl::base;
/** \brief A state space representing SH(2) */
class SH2StateSpace : public ob::CompoundStateSpace
{
public:
    /** \brief A state in SH(2): (x, y, yaw) */
    class StateType : public CompoundStateSpace::StateType
    {
    public:
        StateType() = default;

        /** \brief Get the X component of the state */
        double getX() const
        {
            return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        }

        /** \brief Get the Y component of the state */
        double getY() const
        {
            return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        }

        /** \brief Get the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        double getYaw() const
        {
            return as<ob::SO2StateSpace::StateType>(1)->value;
        }

        double getKappa() const
        {
            return as<ob::RealVectorStateSpace::StateType>(2)->values[0];
        }

        double getDirection() const
        {
            return as<ob::RealVectorStateSpace::StateType>(2)->values[1];
        }

        /** \brief Set the X component of the state */
        void setX(double x)
        {
            as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
        }

        /** \brief Set the Y component of the state */
        void setY(double y)
        {
            as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
        }

        /** \brief Set the X and Y components of the state */
        void setXY(double x, double y)
        {
            setX(x);
            setY(y);
        }

        /** \brief Set the yaw component of the state. This is
            the rotation in plane, with respect to the Z
            axis. */
        void setYaw(double yaw)
        {
            as<ob::SO2StateSpace::StateType>(1)->value = yaw;
        }

        void setKappa(double kappa)
        {
            as<ob::RealVectorStateSpace::StateType>(2)->values[0] = kappa;
        }

        void setDirection(double d)
        {
            as<ob::RealVectorStateSpace::StateType>(2)->values[1] = d;
        }
    };
    SH2StateSpace()
    {
        setName("SH2" + getName());
        type_ = ob::STATE_SPACE_SH2;
        addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);
        addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);
        addSubspace(std::make_shared<ob::RealVectorStateSpace>(2), 1.0);
        lock();
    }

    ~SH2StateSpace() override = default;

    /** \copydoc RealVectorStateSpace::setBounds() */
    void setBounds(const ob::RealVectorBounds &bounds)
    {
        as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    /** \copydoc RealVectorStateSpace::getBounds() */
    const ob::RealVectorBounds &getBounds() const
    {
        return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    ob::State *allocState() const override;
    void freeState(ob::State *state) const override;

    void registerProjections() override;
};

#endif // SH2STATESPACE_H
