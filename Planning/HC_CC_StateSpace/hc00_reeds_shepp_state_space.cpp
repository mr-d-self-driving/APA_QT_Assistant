#include "hc00_reeds_shepp_state_space.h"

class HC00_ReedsSheppStateSpace::HC00_ReedsShepp
{
public:
    explicit HC00_ReedsShepp(HC00_ReedsSheppStateSpace *parent)
    {
        _parent = parent;
    }

    /************************ TT ************************/
    /**
     * @brief Judge TT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return fabs(_distance - 2 * c1.getRadius()) < math::getEpsilon();
        }
    }

    /**
     * @brief Computation of the tangent point of outer circles
     * @param c1 :one circle
     * @param c2 :another circle
     * @param q :the tangent point
     */
    void TT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2, Configuration **q) const
    {
        double x = 0.5 * (c1.getCenter_x() + c2.getCenter_x());
        double y = 0.5 * (c1.getCenter_y() + c2.getCenter_y());
        double angle = atan2(c2.getCenter_y() - c1.getCenter_y(),
                             c2.getCenter_x() - c1.getCenter_x());

        double q_psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                q_psi = angle + MV_PI2 - c1.getMu();
            }
            else
            {
                q_psi = angle + MV_PI2 + c1.getMu();
            }
        }
        else
        {
            if(c1.getForward())
            {
                q_psi = angle - MV_PI2 + c1.getMu();
            }
            else
            {
                q_psi = angle - MV_PI2 - c1.getMu();
            }
        }
        *q = new Configuration(x, y, q_psi, 0.0);
    }

    /**
     * @brief Computation of the TT path
     * @param c1 :one circle
     * @param c2 :another circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q :the configuration tangent point
     * @return the length of path
     */
    double TT_Path(HC_CC_Circle &c1, HC_CC_Circle &c2,
                   HC_CC_Circle **cstart, HC_CC_Circle **cend, Configuration **q) const
    {
        TT_TangentCircles(c1, c2, q);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2.getStart(), c2.getLeft(), c2.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        return (*cstart)->cc_turn_lenght(**q) + (*cend)->cc_turn_lenght(**q);
    }

    /************************ TcT ************************/
    /**
     * @brief Judge TcT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return fabs(_distance - 2 * fabs(c1.getKappaInv())) < math::getEpsilon();
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :one circle
     * @param c2 :another circle
     * @param q  :the tangent point
     */
    void TcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2, Configuration **q) const
    {
        double distance = c1.CenterDistance(c2);
        double delta_x = 0.5 * distance;
        double delta_y = 0.0;
        double angle = atan2(c2.getCenter_y() - c1.getCenter_y(),
                             c2.getCenter_x() - c1.getCenter_x());
        double x,y,psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                psi = angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x,  delta_y, &x, &y);
            }
            else
            {
                psi = angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x, -delta_y, &x, &y);
            }
        }
        else
        {
            if(c1.getForward())
            {
                psi = angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x, -delta_y, &x, &y);
            }
            else
            {
                psi = angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x,  delta_y, &x, &y);
            }
        }
        *q = new Configuration(x, y, psi, c1.getKappa());
    }

    /**
     * @brief Computation of the TcT path
     * @param c1 :one circle
     * @param c2 :another circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q :the configuration tangent point
     * @return the length of path
     */
    double TcT_Path(HC_CC_Circle &c1, HC_CC_Circle &c2,
                    HC_CC_Circle **cstart, HC_CC_Circle **cend, Configuration **q) const
    {
        TcT_TangentCircles(c1, c2, q);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return (*cstart)->hc_turn_lenght(**q) + (*cend)->hc_turn_lenght(**q);
    }

    /************************ Reeds-Shepp families: ************************/

    /************************ TcTcT ************************/
    /**
     * @brief Judge TcTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance <= 4 * fabs(c1.getKappaInv());
        }
    }

    /**
     * @brief Computation of the tangent point of circles
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TcTcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **qa1, Configuration **qa2,
                              Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double two_r = 2 * fabs(c1.getKappaInv());
        double delta_x = 0.5 * _distance;
        double delta_y = sqrt(pow(two_r,2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TcT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TcT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TcT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TcTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param ci : the middle tangent circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @return the length of the TcTcT Path
     */
    double TcTcT_Path(HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      HC_CC_Circle **ci, Configuration **q1, Configuration **q2) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTcT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection
        double length1 = (*cstart)->hc_turn_lenght(*qa1)
                       + middle_tangent_circle1->rs_turn_lenght(*qa2)
                       + (*cend)->hc_turn_lenght(*qa2);

        double length2 = (*cstart)->hc_turn_lenght(*qb1)
                       + middle_tangent_circle1->rs_turn_lenght(*qb2)
                       + (*cend)->hc_turn_lenght(*qb2);

        if(length1 < length2)
        {
            *q1 = qa1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            return length1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete middle_tangent_circle1;
            return length2;
        }
    }

    /************************ TcTT ************************/
    /**
     * @brief Judge TcTT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 2 * ( c1.getRadius() + fabs(c1.getKappaInv()) ))
                && (_distance >= 2 * ( c1.getRadius() - fabs(c1.getKappaInv()) ));
        }
    }

    /**
     * @brief Computation of the tangent point of TcTT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TcTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r1 = 2 * fabs(c1.getKappaInv());
        double r2 = 2 * c1.getRadius();
        double delta_x = (pow(r1, 2) + pow(_distance, 2) - pow(r2, 2)) / ( 2 * _distance );
        double delta_y = sqrt(pow(r1, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TcT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TcTT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param ci : the middle tangent circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @return the length of the TcTcT Path
     */
    double TcTT_Path(HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     HC_CC_Circle **ci, Configuration **q1, Configuration **q2) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2.getStart(), c2.getLeft(), c2.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);

        // select shortest connection
        double length1 = (*cstart)->hc_turn_lenght(*qa1)
                       + middle_tangent_circle1->hc_turn_lenght(*qa1)
                       + (*cend)->cc_turn_lenght(*qa2);

        double length2 = (*cstart)->hc_turn_lenght(*qb1)
                       + middle_tangent_circle1->hc_turn_lenght(*qb1)
                       + (*cend)->cc_turn_lenght(*qb2);

        if(length1 < length2)
        {
            *q1 = qa1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            return length1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete middle_tangent_circle1;
            return length2;
        }
    }

    /************************ TTcT ************************/
    /**
     * @brief Judge TTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 2 * ( c1.getRadius() + fabs(c1.getKappaInv()) ))
                && (_distance >= 2 * ( c1.getRadius() - fabs(c1.getKappaInv()) ));
        }
    }

    /**
     * @brief Computation of the tangent point of TTcT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TTcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r1 = 2 * c1.getRadius();
        double r2 = 2 * fabs(c1.getKappaInv());
        double delta_x = (pow(r1, 2) + pow(_distance, 2) - pow(r2, 2)) / ( 2 * _distance );
        double delta_y = sqrt(pow(r1, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TcT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TcT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param ci : the middle tangent circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @return the length of the TcTcT Path
     */
    double TTcT_Path(HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     HC_CC_Circle **ci, Configuration **q1, Configuration **q2) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TTcT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection
        double length1 = (*cstart)->cc_turn_lenght(*qa1)
                       + middle_tangent_circle1->hc_turn_lenght(*qa2)
                       + (*cend)->hc_turn_lenght(*qa2);

        double length2 = (*cstart)->cc_turn_lenght(*qb1)
                       + middle_tangent_circle1->hc_turn_lenght(*qb2)
                       + (*cend)->hc_turn_lenght(*qb2);

        if(length1 < length2)
        {
            *q1 = qa1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            return length1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete middle_tangent_circle1;
            return length2;
        }
    }

    /************************ TST ************************/
    /************************ TiST ************************/

    /************************ TeST ************************/

    /************************ TTcT ************************/


    /************************ private variable interface ************************/
    double getDistance(void){ return _distance; }
    double getAngle(void) { return _angle; }
private:
    HC00_ReedsSheppStateSpace *_parent;

    double _distance = 0.0;
    double _angle = 0.0;
};

/**
 * @brief HC00_ReedsSheppStateSpace Constructor
 * @param kappa :the max curvature of the path
 * @param sigma :the max sharpness of the path
 * @param discretization :the discretization step
 */
HC00_ReedsSheppStateSpace::HC00_ReedsSheppStateSpace(double kappa, double sigma, double discretization)
    :HC_CC_StateSpace(kappa,sigma,discretization)
    ,_hc00_reeds_shepp{ unique_ptr<HC00_ReedsShepp>(new HC00_ReedsShepp(this)) }
{
    _rs_circle_param.setParam(_kappa, numeric_limits<double>::max(), 1/_kappa,0.0);
}

/**
 * @brief Destructor
 */
HC00_ReedsSheppStateSpace::~HC00_ReedsSheppStateSpace() = default;




