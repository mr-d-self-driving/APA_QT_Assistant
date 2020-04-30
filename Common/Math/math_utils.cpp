#include "math_utils.h"

namespace math{

/**
 * @brief Normallize Angle to [-PI,PI)
 * @param angle the original value of the angle
 * @return The normalized value of the angle
 */
double NormallizeAngle(const double angle)
{
    double a = fmod(angle + M_PI,2.0 * M_PI);
    if(a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return  a - M_PI;
}

/**
 * @brief Normallize Angle to [0 , 2PI)
 * @param angle: the original value of the angle
 * @return The normalized value of the angle
 */
double TwoPiNormallizeAngle(const double angle)
{
    double a = fmod(angle,2.0 * M_PI);
    if(a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return  a;
}

/**
 * @brief 获取变量的符号
 * @param x: 求取符号的变量
 * @return 符号值: (-1.0, 1.0)
 */
double sgn(double x)
{
    return ((x < 0.0) ? -1.0 : 1.0);
}

/**
 * @brief Transformation of (local_x, local_y) from local coordinate system to global one
 * @param (x, y, psi): the local body point in global coordinate system
 * @param (local_x,local_y): the local coordinate system
 * @param (global_x,global_y): the global coordinate system
 */
void change_to_global_frame(double x, double y, double psi,
                            double   local_x, double   local_y,
                            double *global_x, double *global_y)
{
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    *global_x = x + local_x * cos_psi - local_y * sin_psi ;
    *global_y = y + local_x * sin_psi + local_y * cos_psi ;
}

/**
 * @brief Transformation of (global_x, global_y) from global coordinate system to local one
 * @param (x, y, psi): the local body point in global coordinate system
 * @param (global_x,global_y): the global coordinate system
 * @param (local_x,local_y): the local coordinate system
 */
void change_to_local_frame(double x, double y, double psi,
                           double global_x, double global_y,
                           double *local_x, double *local_y)
{
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    *local_x = (global_x - x) * cos_psi - (global_y - y) * sin_psi ;
    *local_y = (global_x - x) * sin_psi + (global_y - y) * cos_psi ;
}

/**
 * @brief get the epsilon value
 * @return  the epsilon value
 */
double getEpsilon(void){return _epsilon;}

}


