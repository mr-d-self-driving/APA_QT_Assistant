#include "hc0pm_reeds_shepp_state_space.h"

class HC0PM_ReedsSheppStateSpace::HC0PM_ReedsShepp
{
public:
    explicit HC0PM_ReedsShepp(HC0PM_ReedsSheppStateSpace *parent)
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

        double q_psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                q_psi = _angle + MV_PI2 - c1.getMu();
            }
            else
            {
                q_psi = _angle + MV_PI2 + c1.getMu();
            }
        }
        else
        {
            if(c1.getForward())
            {
                q_psi = _angle - MV_PI2 + c1.getMu();
            }
            else
            {
                q_psi = _angle - MV_PI2 - c1.getMu();
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
    double TT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                    HC_CC_Circle **cstart, HC_CC_Circle **cend,
                    Configuration **q1, Configuration **q2) const
    {
        TT_TangentCircles(c1, c2, q1);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q1, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q2 = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        return (*cstart)->cc_turn_lenght(**q1) + (*cend)->hc_turn_lenght(**q2);
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
        double delta_x = 0.5 * _distance;
        double delta_y = 0.0;

        double x,y,psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                psi = _angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), _angle,
                                             delta_x,  delta_y, &x, &y);
            }
            else
            {
                psi = _angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), _angle,
                                             delta_x, -delta_y, &x, &y);
            }
        }
        else
        {
            if(c1.getForward())
            {
                psi = _angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), _angle,
                                             delta_x, -delta_y, &x, &y);
            }
            else
            {
                psi = _angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), _angle,
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
    double TcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend, Configuration **q) const
    {
        TcT_TangentCircles(c1, c2, q);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return (*cstart)->hc_turn_lenght(**q) + (*cend)->rs_turn_lenght(**q);
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
    void TcTcT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
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
        HC_CC_Circle middle_tangent_circle1(x, y, !c2.getLeft(), c2.getForward(), true, _parent->_rs_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c2.getLeft(), c2.getForward(), true, _parent->_rs_circle_param);

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
    double TcTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTcT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c2.getLeft(), c2.getForward(), true, _parent->_rs_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c2.getLeft(), c2.getForward(), true, _parent->_rs_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection
        double length1 = (*cstart)->hc_turn_lenght(*qa1)
                       + middle_tangent_circle1->rs_turn_lenght(*qa2)
                       + (*cend)->rs_turn_lenght(*qa2);

        double length2 = (*cstart)->hc_turn_lenght(*qb1)
                       + middle_tangent_circle1->rs_turn_lenght(*qb2)
                       + (*cend)->rs_turn_lenght(*qb2);

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
     * @return the length of the TcTT Path
     */
    double TcTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        HC_CC_Circle *end_circle1, *end_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle1            = new HC_CC_Circle(*qa2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        end_circle2            = new HC_CC_Circle(*qb2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *q2     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        // select shortest connection
        double length1 = (*cstart)->hc_turn_lenght(*qa1)
                       + middle_tangent_circle1->hc_turn_lenght(*qa1)
                       + end_circle1->hc_turn_lenght(**q2);

        double length2 = (*cstart)->hc_turn_lenght(*qb1)
                       + middle_tangent_circle2->hc_turn_lenght(*qb1)
                       + end_circle2->hc_turn_lenght(**q2);

        if(length1 < length2)
        {
            *cend = end_circle1;
            *q1 = qa1;
            *ci = middle_tangent_circle1;
            delete qa2;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            delete end_circle2;
            return length1;
        }
        else
        {
            *cend = end_circle2;
            *q1 = qb1;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete qb2;
            delete middle_tangent_circle1;
            delete end_circle1;
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
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @param ci : the middle tangent circle
     * @return the length of the TTcT Path
     */
    double TTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
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
                       + (*cend)->rs_turn_lenght(*qa2);

        double length2 = (*cstart)->cc_turn_lenght(*qb1)
                       + middle_tangent_circle2->hc_turn_lenght(*qb2)
                       + (*cend)->rs_turn_lenght(*qb2);

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
    /**
     * @brief Judge TiST path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TiST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * c1.getRadius());
        }
    }

    /**
     * @brief Judge TeST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return (_distance >= 2 * c1.getRadius() * c1.getSinMu());
        }
    }

    /**
     * @brief Judge TST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TiST_Exist(c1, c2) || TeST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TiST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TiST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **q1, Configuration **q2) const
    {
        double alpha = asin(2 * c1.getRadius() * c1.getCosMu() / _distance);
        double delta_x = c1.getRadius() * c1.getSinMu();
        double delta_y = c1.getRadius() * c1.getCosMu();
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TiST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TeST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **q1, Configuration **q2) const
    {
        double psi = _angle;
        double delta_x = c1.getRadius() * c1.getSinMu();
        double delta_y = c1.getRadius() * c1.getCosMu();
        double x, y;

        if(c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TiST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TiST Path
     */
    double TiST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TiST_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q3     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q3);
    }

    /**
     * @brief Computation of the length of TeST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeST Path
     */
    double TeST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TeST_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q3     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q3);

    }

    /**
     * @brief Computation of the length of TST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TST Path
     */
    double TST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        if(TiST_Exist(c1, c2))
        {
            return TiST_Path(c1, c2, cstart, cend, q1, q2, q3);
        }
        else if(TeST_Exist(c1, c2))
        {
            return TeST_Path(c1,c2, cstart, cend, q1, q2, q3);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }

    /************************ TSTcT ************************/
    /**
     * @brief Judge TiSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TiSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return (_distance >= sqrt(pow(2 * c1.getRadius() * c1.getSinMu() + 2 *fabs(c1.getKappaInv()), 2) +
                                      pow(2 * c1.getRadius() * c1.getCosMu(), 2)));
        }
    }

    /**
     * @brief Judge TeSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * (fabs(c1.getKappaInv()) + c1.getRadius() * c1.getSinMu()));
        }
    }

    /**
     * @brief Judge TSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TSTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TiSTcT_Exist(c1, c2) || TeSTcT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TiSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TiSTcT Path
     */
    double TiSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2, Configuration **q3, HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_y = (4 * c1.getRadius() * c1.getCosMu()) / (fabs(c1.getKappa()) * _distance);
        double delta_x = sqrt(pow(2 * c1.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);


        TiST_TangentCircles(c1, middle_circle, q1, q2);
        TcT_TangentCircles(middle_circle, c2, q3);

        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        *ci     = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*ci)->hc_turn_lenght(**q3) +
                (*cend)->rs_turn_lenght(**q3);

    }

    /**
     * @brief Computation of the length of TeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TeSTcT Path
     */
    double TeSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2, Configuration **q3, HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c2.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TeST_TangentCircles(c1, middle_circle, q1, q2);
        TcT_TangentCircles(middle_circle, c2, q3);

        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        *ci     = new HC_CC_Circle(**q2, c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*ci)->hc_turn_lenght(**q3) +
                (*cend)->rs_turn_lenght(**q3);

    }

    /**
     * @brief Computation of the length of TeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TeSTcT Path
     */
    double TSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3,
                       HC_CC_Circle **ci) const
    {
        if( TiSTcT_Exist(c1, c2) )
        {
            return TiSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, ci);
        }
        else if( TeSTcT_Exist(c1, c2) )
        {
            return TeSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, ci);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcTST ************************/
    /**
     * @brief Judge TcTiST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTiST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return (_distance >= sqrt(pow(2 * c1.getRadius() * c1.getSinMu() + 2 *fabs(c1.getKappaInv()), 2) +
                                      pow(2 * c1.getRadius() * c1.getCosMu(), 2)));
        }
    }

    /**
     * @brief Judge TcTeST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTeST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * (fabs(c1.getKappaInv()) + c1.getRadius() * c1.getSinMu()));
        }
    }

    /**
     * @brief Judge TcTST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTST_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TcTiST_Exist(c1, c2) || TcTeST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TcTiST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent point betwen start circle and straight line
     * @param q2 : the tengent point betwen middle circle and straight line
     * @param q3 : the tengent point betwen middle circle and end circle
     * @param q4 : the configuration point in the end
     * @param c1 : the middle circle
     * @return the length of the TcTiST Path
     */
    double TcTiST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_y = (4 * c1.getRadius() * c1.getCosMu()) / (fabs(c1.getKappa()) * _distance);
        double delta_x = sqrt(pow(2 * c1.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_circle, q1);
        TiST_TangentCircles(middle_circle, c2, q2, q3);

        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(**q3, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q4 = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        *ci = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);

        return  (*cstart)->hc_turn_lenght(**q1) +
                (*ci)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTeST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the cross point betwen start circle and straight line
     * @param q2 : the tengent point betwen middle circle and straight line
     * @param q3 : the tengent point betwen middle circle and end circle
     * @param q4 : the tengent point betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TcTeST Path
     */
    double TcTeST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c2.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_circle, q1);
        TeST_TangentCircles(middle_circle, c2, q2, q3);


        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(**q3, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q4 = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        *ci = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);

        return  (*cstart)->hc_turn_lenght(**q1) +
                (*ci)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);

    }

    /**
     * @brief Computation of the length of TcTST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TcTST Path
     */
    double TcTST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2,
                       Configuration **q3, Configuration **q4,
                       HC_CC_Circle **ci) const
    {
        if( TcTiST_Exist(c1, c2) )
        {
            return TcTiST_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else if( TcTeST_Exist(c1, c2) )
        {
            return TcTeST_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcTSTcT ************************/
    /**
     * @brief Judge TcTiSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTiSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= sqrt(pow(2 * c1.getRadius(), 2) +
                                      16 * c1.getRadius() * c1.getSinMu() * fabs(c1.getKappaInv()) +
                                      pow(4 * c1.getKappaInv(), 2)));
        }
    }

    /**
     * @brief Judge TcTeSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTeSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
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
            return (_distance >= 4 * fabs(c1.getKappaInv()) + 2 * c1.getRadius() * c1.getSinMu() );
        }
    }

    /**
     * @brief Judge TcTSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTSTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TcTiSTcT_Exist(c1, c2) || TcTeSTcT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TcTiSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTiSTcT Path
     */
    double TcTiSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                          HC_CC_Circle **cstart, HC_CC_Circle **cend,
                          Configuration **q1, Configuration **q2,
                          Configuration **q3, Configuration **q4,
                          HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        double psi = _angle;
        double delta_y = (4 * c1.getRadius() * c1.getCosMu()) / (fabs(c1.getKappa()) * _distance);
        double delta_x = sqrt(pow(2 * c1.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle, q1);
        TiST_TangentCircles(left_middle_circle, right_middle_circle, q2, q3);
        TcT_TangentCircles(right_middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(c2);
        *ci1 = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        *ci2 = new HC_CC_Circle(**q3, !c2.getLeft(), c2.getForward(), true, _parent->_hc_cc_circle_param);

        return  (*cstart)->hc_turn_lenght(**q1) +
                (*ci1)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci2)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTeSTcT Path
     */
    double TcTeSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                          HC_CC_Circle **cstart, HC_CC_Circle **cend,
                          Configuration **q1, Configuration **q2,
                          Configuration **q3, Configuration **q4,
                          HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c1.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle, q1);
        TeST_TangentCircles(left_middle_circle, right_middle_circle, q2, q3);
        TcT_TangentCircles(right_middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(c2);
        *ci1 = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        *ci2 = new HC_CC_Circle(**q3, !c2.getLeft(), c2.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (*ci1)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci2)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);

    }

    /**
     * @brief Computation of the length of TcTSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTSTcT Path
     */
    double TcTSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                         HC_CC_Circle **cstart, HC_CC_Circle **cend,
                         Configuration **q1, Configuration **q2,
                         Configuration **q3, Configuration **q4,
                         HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        if( TcTiSTcT_Exist(c1, c2) )
        {
            return TcTiSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
        }
        else if( TcTeSTcT_Exist(c1, c2) )
        {
            return TcTeSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }

    /************************ TTcTT ************************/
    /**
     * @brief Judge TTcTT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TTcTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
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
            return (_distance <= 4 * c1.getRadius() + 2 * fabs(c1.getKappaInv()));
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :start circle
     * @param c2 :end circle
     * @param qa1 :the tangent point betwen start circle and left middle circle
     * @param qa2 :the inersection point betwen left middle circle and right middle circle
     * @param qa3 :the tangent point betwen right middle circle and end circle
     * @param qb1 :the tangent point betwen start circle and left middle circle
     * @param qb2 :the inersection point betwen left middle circle and right middle circle
     * @param qb3 :the tangent point betwen right middle circle and end circle
     */
    void TTcTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **qa1, Configuration **qa2, Configuration **qa3,
                              Configuration **qb1, Configuration **qb2, Configuration **qb3) const
    {
        double psi = _angle;
        double r1, r2, delta_x, delta_y, x, y;

        r1 = 2 * fabs(c1.getKappaInv());
        r2 = 2 * c1.getRadius();

        if( _distance < 4 * c1.getRadius() - 2 * fabs(c1.getKappaInv()) )
        {
            delta_x = (_distance + r1) / 2;
            delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
        }
        else
        {
            delta_x = (_distance - r1) / 2;
            delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
        }

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle1(x, y, !c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle left_middle_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle2(x, y, !c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, left_middle_circle1, qa1);
        TcT_TangentCircles(left_middle_circle1, right_middle_circle1, qa2);
        TT_TangentCircles(right_middle_circle1, c2, qa3);

        TT_TangentCircles(c1, left_middle_circle2, qb1);
        TcT_TangentCircles(left_middle_circle2, right_middle_circle2, qb2);
        TT_TangentCircles(right_middle_circle2, c2, qb3);
    }

    /**
     * @brief Computation of the TTcTT path
     * @param c1 :Configuration Start circle
     * @param c2 :Configuration end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 :the tangent point betwen start circle and left middle circle
     * @param q2 :the inersection point betwen left middle circle and right middle circle
     * @param q3 :the tangent point betwen right middle circle and end circle
     * @param ci1 :the left middle circle
     * @param ci2 :the right middle circle
     * @return the length of path
     */
    double TTcTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3,
                       HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        Configuration *qa1, *qa2, *qa3;
        Configuration *qb1, *qb2, *qb3;
        TTcTT_TangentCircles(c1, c2, &qa1, &qa2, &qa3, &qb1, &qb2, &qb3);

        HC_CC_Circle *left_middle_cricle1, *right_middle_circle1;
        HC_CC_Circle *left_middle_cricle2, *right_middle_circle2;
        HC_CC_Circle *end_circle1, *end_circle2;
        left_middle_cricle1  = new HC_CC_Circle(*qa1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle1 = new HC_CC_Circle(*qa3, !c2.getLeft(),  c2.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle1          = new HC_CC_Circle(*qa3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        left_middle_cricle2  = new HC_CC_Circle(*qb1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle2 = new HC_CC_Circle(*qb3, !c2.getLeft(),  c2.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle2          = new HC_CC_Circle(*qb3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *q3     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        // select shortest connection path
        double lenght1 = (*cstart)->cc_turn_lenght(*qa1) +
                         left_middle_cricle1->hc_turn_lenght(*qa2) +
                         right_middle_circle1->hc_turn_lenght(*qa2) +
                         end_circle1->hc_turn_lenght(**q3);

        double lenght2 = (*cstart)->cc_turn_lenght(*qb1) +
                         left_middle_cricle1->hc_turn_lenght(*qb2) +
                         right_middle_circle1->hc_turn_lenght(*qb2) +
                         end_circle2->hc_turn_lenght(**q3);

        if( lenght1 < lenght2 )
        {
            *cend = end_circle1;
            *q1 = qa1;
            *q2 = qa2;
            *q3 = qa3;
            *ci1 = left_middle_cricle1;
            *ci2 = right_middle_circle1;
            delete qb1;
            delete qb2;
            delete qb3;
            delete left_middle_cricle2;
            delete right_middle_circle2;
            return lenght1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *q3 = qb3;
            *ci1 = left_middle_cricle2;
            *ci2 = right_middle_circle2;
            delete qa1;
            delete qa2;
            delete qa3;
            delete left_middle_cricle1;
            delete right_middle_circle1;
            return lenght2;
        }
    }

    /************************ TcTTcT ************************/
    /**
     * @brief Judge TcTTcT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TcTTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
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
            return (_distance <= 4 * fabs(c1.getKappaInv()) + 2 * c1.getRadius()) &&
                   (_distance >= 4 * fabs(c1.getKappaInv()) - 2 * c1.getRadius());
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :start circle
     * @param c2 :end circle
     * @param qa1 :the inersection point betwen start circle and left middle circle
     * @param qa2 :the tangent point betwen left middle circle and right middle circle
     * @param qa3 :the inersection point betwen right middle circle and end circle
     * @param qb1 :the inersection point betwen start circle and left middle circle
     * @param qb2 :the tangent point betwen left middle circle and right middle circle
     * @param qb3 :the inersection point betwen right middle circle and end circle
     */
    void TcTTcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **qa1, Configuration **qa2, Configuration **qa3,
                              Configuration **qb1, Configuration **qb2, Configuration **qb3) const
    {
        double psi = _angle;
        double r1, r2, delta_x, delta_y, x, y;

        r1 = 2 * fabs(c1.getKappaInv());
        r2 = 2 * c1.getRadius();

        delta_x = (pow(r1, 2) + pow(_distance / 2, 2) - pow(r2, 2)) / _distance;
        delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));



        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle1(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle1(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle left_middle_circle2(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle2(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle1, qa1);
        TT_TangentCircles(left_middle_circle1, right_middle_circle1, qa2);
        TcT_TangentCircles(right_middle_circle1, c2, qa3);

        TcT_TangentCircles(c1, left_middle_circle2, qb1);
        TT_TangentCircles(left_middle_circle2, right_middle_circle2, qb2);
        TcT_TangentCircles(right_middle_circle2, c2, qb3);
    }

    /**
     * @brief Computation of the TcTTcT path
     * @param c1 :Configuration Start circle
     * @param c2 :Configuration end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 :the inersection point betwen start circle and left middle circle
     * @param q2 :the tangent point betwen left middle circle and right middle circle
     * @param q3 :the inersection point betwen right middle circle and end circle
     * @param ci1 :the left middle circle
     * @param ci2 :the right middle circle
     * @return the length of path
     */
    double TcTTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2, HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3,
                       HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        Configuration *qa1, *qa2, *qa3;
        Configuration *qb1, *qb2, *qb3;
        TcTTcT_TangentCircles(c1, c2, &qa1, &qa2, &qa3, &qb1, &qb2, &qb3);

        HC_CC_Circle *left_middle_cricle1, *right_middle_circle1;
        HC_CC_Circle *left_middle_cricle2, *right_middle_circle2;
        left_middle_cricle1  = new HC_CC_Circle(*qa2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle1 = new HC_CC_Circle(*qa2,  c1.getLeft(), !c1.getForward(), true, _parent->_hc_cc_circle_param);
        left_middle_cricle2  = new HC_CC_Circle(*qb2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle2 = new HC_CC_Circle(*qb2,  c1.getLeft(), !c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection path
        double lenght1 = (*cstart)->hc_turn_lenght(*qa1) +
                         left_middle_cricle1->hc_turn_lenght(*qa1) +
                         right_middle_circle1->hc_turn_lenght(*qa3) +
                         (*cend)->hc_turn_lenght(*qa3);

        double lenght2 = (*cstart)->hc_turn_lenght(*qb1) +
                         left_middle_cricle1->hc_turn_lenght(*qb1) +
                         right_middle_circle1->hc_turn_lenght(*qb3) +
                         (*cend)->hc_turn_lenght(*qb3);

        if( lenght1 < lenght2 )
        {
            *q1 = qa1;
            *q2 = qa2;
            *q3 = qa3;
            *ci1 = left_middle_cricle1;
            *ci2 = right_middle_circle1;
            delete qb1;
            delete qb2;
            delete qb3;
            delete left_middle_cricle2;
            delete right_middle_circle2;
            return lenght1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *q3 = qb3;
            *ci1 = left_middle_cricle2;
            *ci2 = right_middle_circle2;
            delete qa1;
            delete qa2;
            delete qa3;
            delete left_middle_cricle1;
            delete right_middle_circle1;
            return lenght2;
        }
    }
    /************************ Additional Families ************************/
    /************************ TTT ************************/
    /**
     * @brief Judge TTT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
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
            return _distance <= 4 * c1.getRadius();
        }
    }

    /**
     * @brief Computation of the tangent point of TTT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r = 2 * c1.getRadius();
        double delta_x = 0.5 * _distance;
        double delta_y = sqrt(pow(r, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TTT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @param ci : the middle tangent circle
     * @return the length of the TcTcT Path
     */
    double TTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     Configuration **q1, Configuration **q2,
                     HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TTT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2.getStart(), c2.getLeft(), c2.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);

        // select shortest connection
        double length1 = (*cstart)->cc_turn_lenght(*qa1)
                       + middle_tangent_circle1->cc_turn_lenght(*qa2)
                       + (*cend)->cc_turn_lenght(*qa2);

        double length2 = (*cstart)->cc_turn_lenght(*qb1)
                       + middle_tangent_circle1->cc_turn_lenght(*qb2)
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

    /************************ TcST ************************/
    /**
     * @brief Judge TciST path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TciST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(c1.getRadius() * c1.getSinMu(), 2) +
                                      pow(c1.getRadius() * c1.getCosMu() + fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TceST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TceST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return _distance >= sqrt( pow(c1.getRadius() * c1.getSinMu(), 2) +
                                      pow(c1.getRadius() * c1.getCosMu() - fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TcST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TciST_Exist(c1, c2) || TceST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TciST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TciST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (c1.getRadius() * c1.getCosMu() + fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = 0.0;
        double delta_y1 = fabs(c1.getKappaInv());
        double delta_x2 = c1.getRadius() * c1.getSinMu();
        double delta_y2 = c1.getRadius() * c1.getCosMu();
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TceST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TceST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (c1.getRadius() * c1.getCosMu() - fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = 0.0;
        double delta_y1 = fabs(c1.getKappaInv());
        double delta_x2 = c1.getRadius() * c1.getSinMu();
        double delta_y2 = c1.getRadius() * c1.getCosMu();
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TciST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TciST Path
     */
    double TciST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        TciST_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(c2.getStart(), c2.getLeft(), c2.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->cc_turn_lenght(**q2);

    }

    /**
     * @brief Computation of the length of TceST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeST Path
     */
    double TceST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        TceST_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2.getStart(), c2.getLeft(), c2.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->cc_turn_lenght(**q2);
    }

    /**
     * @brief Computation of the length of TcST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TST Path
     */
    double TcST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2) const
    {
        if(TciST_Exist(c1, c2))
        {
            return TciST_Path(c1, c2, cstart, cend, q1, q2);
        }
        else if(TceST_Exist(c1, c2))
        {
            return TceST_Path(c1,c2, cstart, cend, q1, q2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TScT ************************/
    /**
     * @brief Judge TiScT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TiScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(c1.getRadius() * c1.getSinMu(), 2) +
                                      pow(c1.getRadius() * c1.getCosMu() + fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TeScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return _distance >= sqrt( pow(c1.getRadius() * c1.getSinMu(), 2) +
                                      pow(c1.getRadius() * c1.getCosMu() - fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TiScT_Exist(c1, c2) || TeScT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TiScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TiScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (c1.getRadius() * c1.getCosMu() + fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = c1.getRadius() * c1.getSinMu();
        double delta_y1 = c1.getRadius() * c1.getCosMu();
        double delta_x2 = 0.0;
        double delta_y2 = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TeScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TeScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (c1.getRadius() * c1.getCosMu() - fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = c1.getRadius() * c1.getSinMu();
        double delta_y1 = c1.getRadius() * c1.getCosMu();
        double delta_x2 = 0.0;
        double delta_y2 = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TiScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TiScT Path
     */
    double TiScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        TiScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q2);

    }

    /**
     * @brief Computation of the length of TeScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeScT Path
     */
    double TeScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        TeScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->cc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q2);
    }

    /**
     * @brief Computation of the length of TScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TScT Path
     */
    double TScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2) const
    {
        if(TiScT_Exist(c1, c2))
        {
            return TiScT_Path(c1, c2, cstart, cend, q1, q2);
        }
        else if(TeScT_Exist(c1, c2))
        {
            return TeScT_Path(c1,c2, cstart, cend, q1, q2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcScT ************************/
    /**
     * @brief Judge TciScT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TciScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= 2 * fabs(c1.getKappaInv());
        }
    }

    /**
     * @brief Judge TceScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TceScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
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
            return _distance >= math::getEpsilon();
        }
    }

    /**
     * @brief Judge TcScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TciScT_Exist(c1, c2) || TceScT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TciScTs circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TciScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                                Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( 2 / (fabs(c1.getKappa()) * _distance));
        double delta_x = 0.0;
        double delta_y = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TceScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TceScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                                Configuration **q1, Configuration **q2) const
    {
        double psi = _angle;
        double delta_x = 0.0;
        double delta_y = fabs(c1.getKappaInv());
        double x, y;

        if(c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TciScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TciScT Path
     */
    double TciScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2) const
    {
        TciScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q2);

    }

    /**
     * @brief Computation of the length of TceScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TceScT Path
     */
    double TceScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2) const
    {
        TeScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q2);
    }

    /**
     * @brief Computation of the length of TcScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TcScT Path
     */
    double TcScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        if(TciScT_Exist(c1, c2))
        {
            return TciScT_Path(c1, c2, cstart, cend, q1, q2);
        }
        else if(TceScT_Exist(c1, c2))
        {
            return TceScT_Path(c1,c2, cstart, cend, q1, q2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ private variable interface ************************/
    void   setDistance(double value){ _distance = value; }
    double getDistance(void)        { return _distance; }

    void   setAngle(double value) { _angle = value; }
    double getAngle(void)         { return _angle; }
private:
    HC0PM_ReedsSheppStateSpace *_parent;

    double _distance = 0.0;
    double _angle = 0.0;
}; // end of HC00_ReedsShepp class function define


HC0PM_ReedsSheppStateSpace::HC0PM_ReedsSheppStateSpace(double kappa, double sigma, double discretization)
{

}
