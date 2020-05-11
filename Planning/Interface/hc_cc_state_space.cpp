#include "hc_cc_state_space.h"

using namespace math;
/**
 * @brief HC_CC_StateSpace :Constructor
 * @param kappa : the curvature of the circle
 * @param sigma : the sharpness of the clothoid
 * @param discretization : the coefficient of discretization
 */
HC_CC_StateSpace::HC_CC_StateSpace(double kappa, double sigma, double discretization)
    : _kappa(kappa), _sigma(sigma), _discretization(discretization)
{
    Q_ASSERT(kappa > 0.0 && sigma > 0.0 && discretization > 0.0);

    double lenght_min = kappa / sigma;
    double x_i, y_i, psi_i, kappa_i;
    if(lenght_min > math::getEpsilon())
    {
        math::clothoid_to_end(0.0, 0.0, 0.0, 0.0, sigma, 1, lenght_min, &x_i, &y_i, &psi_i, &kappa_i);
    }
    else
    {
        x_i = 0.0;
        y_i = 0.0;
        psi_i = 0.0;
        kappa_i = 0.0;
    }
    // calculate the radius
    // step1: the center of circle
    double x_c, y_c;
    x_c = x_i - sin(psi_i) / kappa;
    y_c = y_i + cos(psi_i) / kappa;
    // step2: base on the center of circle, calulate the radius
    double radius = math::PointDistance(x_c, y_c, 0.0, 0.0);

    // calculate the mu
    double mu = atan(fabs(x_c / y_c));

    _hc_cc_circle_param.setParam(kappa, sigma, radius, mu);
}

HC_CC_StateSpace::~HC_CC_StateSpace() = default;
/**
 * @brief get the path from state1 to state2
 * @param state1 :start state
 * @param state2 :end state
 * @return return the path from state1 to state2
 */
vector<State> HC_CC_StateSpace::getPath(const State &state1, const State &state2) const
{
    vector<Control> controls = getControls(state1, state2);
    return integrate(state1, controls);
}

/**
 * @brief Computation of the integrate states given a start state and controls
 * @param state : the start state
 * @param controls :controls commond
 * @return return integrated states given a start state and controls
 */
vector<State> HC_CC_StateSpace::integrate(const State &state, const vector<Control> &controls) const
{
    vector<State> path;
    State state_curr, state_next;

    // calculate the capacity of path
    uint16_t n_states = 0;
    for(const auto &control : controls)
    {
        double abs_delta_s = fabs(control.delta_s);
        n_states += static_cast<uint16_t>(ceil(abs_delta_s / _discretization));
    }
    path.reserve(n_states + 3);

    // push back first state
    state_curr.x = state.x;
    state_curr.y = state.y;
    state_curr.psi = state.psi;
    state_curr.kappa = controls.front().kappa;
    state_curr.d = math::sgn(controls.front().delta_s);
    path.push_back(state_curr);

    for(const auto &control : controls)
    {
        double delta_s = control.delta_s;
        double abs_delta_s = fabs(delta_s);
        double kappa = control.kappa;
        double s_seg = 0.0;
        double integration_step = 0.0;

        if( fabs( kappa - state_curr.kappa) > math::getEpsilon())
        {
            state_curr.kappa = kappa;
            state_curr.d = math::sgn(delta_s);
            path.push_back(state_curr);
        }

        for(uint16_t i = 0, n = static_cast<uint16_t>(ceil(abs_delta_s / _discretization)); i < n; i++)
        {
            // get integration step
            s_seg += _discretization;
            if( s_seg > abs_delta_s )
            {
                integration_step = _discretization - ( s_seg - abs_delta_s);
                s_seg = abs_delta_s;
            }
            else
            {
                integration_step = _discretization;
            }
            state_next = integrate_ODE(state_curr, control, integration_step);
            path.push_back(state_next);
            state_curr = state_next;
        }
    }
    return path;
}

/**
 * @brief Computation the interpolate state at distance t in [0, 1](percentage if total path lenght)
 * @param state :the start state
 * @param controls :the control commond
 * @param t :the distance
 * @return return the interpolate state at distance t
 */
State HC_CC_StateSpace::interpolate(const State &state, const vector<Control> &controls, double t) const
{
    State state_curr, state_next;

    // get first state
    state_curr.x = state.x;
    state_curr.y = state.y;
    state_curr.psi = state.psi;
    state_curr.kappa = controls.front().kappa;
    state_curr.d = math::sgn(controls.front().delta_s);

    // get sarc lenght at t
    double s_path = 0.0;
    double s_inter = 0.0;
    for(const auto &control : controls)
    {
        s_path += fabs(control.delta_s);
    }
    if( t <= 0.0 )
    {
        return state_curr;
    }
    else if( t > 1.0 )
    {
        s_inter = s_path;
    }
    else
    {
        s_inter = t * s_path;
    }

    double s = 0.0;
    bool interpolated = false;
    for( const auto &control : controls)
    {
        if(interpolated)
            break;

        double delta_s = control.delta_s;
        double abs_delta_s = fabs(delta_s);
        double kappa = control.kappa;
        double s_seg = 0.0;
        double integration_step = 0.0;
        // update current state if curvature discontinuity
        if( fabs(kappa - state_curr.kappa) > math::getEpsilon() )
        {
            state_curr.kappa = kappa;
            state_curr.d = math::sgn(delta_s);
        }

        s += abs_delta_s;
        if( s > s_inter )
        {
            abs_delta_s = abs_delta_s - ( s - s_inter );
            interpolated = true;
        }

        for(uint16_t i = 0, n = static_cast<uint16_t>(ceil(abs_delta_s / _discretization)); i < n; i++)
        {
            // get integration step
            s_seg += _discretization;
            if( s_seg > abs_delta_s )
            {
                integration_step = _discretization - (s_seg - abs_delta_s);
                s_seg = abs_delta_s;
            }
            else
            {
                integration_step = _discretization;
            }
            state_next = integrate_ODE(state_curr, control, integration_step);
            state_curr = state_next;
        }
    }
    return state_curr;
}

/**
 * @brief Computation of integrated state given a start state, a control and a integration step
 * @param state :start state
 * @param control :control commond
 * @param integration_step :integration step
 * @return return the integrated state
 */
inline State HC_CC_StateSpace::integrate_ODE(const State &state, const Control &control, double integration_step) const
{
    State state_next;
    double sigma = control.sigma;
    double d = math::sgn(control.delta_s);
    if( fabs(sigma) > math::getEpsilon() ) // 角速度不为零，动打方向盘
    {
        math::clothoid_to_end(state.x, state.y, state.psi, state.kappa,
                              sigma, d, integration_step,
                              &state_next.x, &state_next.y, &state_next.psi, &state_next.kappa);
        state_next.d = d;
    }
    else
    {
        if( fabs(state.kappa) > math::getEpsilon()) // 曲率不为零，固定圆周运动
        {
            math::circular_arc_to_end(state.x, state.y, state.psi, state.kappa,
                                      d, integration_step,
                                      &state_next.x, &state_next.y, &state_next.psi);
            state_next.kappa = state.kappa;
            state_next.d = d;
        }
        else // 曲率为零，直线运动
        {
            math::straight_line_to_end(state.x, state.y, state.psi,
                                       d, integration_step,
                                       &state_next.x, &state_next.y);
            state_next.psi = state.psi;
            state_next.kappa = state.kappa;
            state_next.d = d;
        }
    }
    return state_next;
}
