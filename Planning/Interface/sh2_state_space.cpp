#include "sh2_state_space.h"

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>


ob::State *SH2StateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void SH2StateSpace::freeState(ob::State *state) const
{
    CompoundStateSpace::freeState(state);
}

void SH2StateSpace::registerProjections()
{
    class SH2DefaultProjection : public ob::ProjectionEvaluator
    {
    public:
        SH2DefaultProjection(const ob::StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 2;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<SH2StateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<SH2StateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 2);
        }
    };

    registerDefaultProjection(std::make_shared<SH2DefaultProjection>(this));
}
