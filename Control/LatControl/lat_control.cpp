/*
 * lat_control.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#include "lat_control.h"‬

LatControl::LatControl() {
	// TODO Auto-generated constructor stub

}

LatControl::~LatControl() {
	// TODO Auto-generated destructor stub
}

void LatControl::Init()
{
	_lat_control_status = init_status;
}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->SteeringEnable)
	{
		ctl->SteeringAngle = 0;
	}
}

float LatControl::TargetLine(float x)
{
	return cosf(COEFFICIENT_TLS*x)-1;
}

float LatControl::TargetLineFirstDerivative(float x)
{
	return -COEFFICIENT_TLS*sinf(COEFFICIENT_TLS*x);
}

float LatControl::TargetLineSecondDerivative(float x)
{
	return -COEFFICIENT_TLS*COEFFICIENT_TLS*cosf(COEFFICIENT_TLS*x);
}

//
float LatControl::SatFunction(float x)
{
	if(x > COEFFICIENT_DELTA)
	{
		return  1.0f;
	}
	else if(x < -COEFFICIENT_DELTA)
	{
		return -1.0f;
	}
	else
	{
		return x/COEFFICIENT_DELTA;
	}
}

TargetTrack LatControl::TrackingCurve(float x)
{
	TargetTrack _temp_track;
	float _temp_cos_psi_r;

	_temp_track.point.setX(x);
	_temp_track.point.setY(TargetLine(x));
	_temp_track.yaw = atanf(TargetLineFirstDerivative(x)); // (-90,90)°
	_temp_cos_psi_r = cosf(_temp_track.yaw);
	_temp_track.curvature = _temp_cos_psi_r*_temp_cos_psi_r*_temp_cos_psi_r*TargetLineSecondDerivative(x);
	return _temp_track;
}

//psi in (-pi/2,pi/2)
void LatControl::Proc(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float delta_t;
	float cos_psi_a,cos_psi_t;
	float tan_psi_a,tan_psi_t;
	float temp_a,temp_b;
	float delta_ctl;

	cos_psi_t = cosf(t_track.yaw);
	tan_psi_t = tanf(t_track.yaw);

	cos_psi_a = cosf(a_track->getYaw());
	tan_psi_a = tanf(a_track->getYaw());

	delta_t = atanf(WHEEL_BASE*t_track.curvature);

	_x1 = t_track.point.getY() - a_track->getPosition().getY();

	if(Drive == msg->getGear())
	{
		_x2 = tan_psi_t - tan_psi_a;
	}
	else if(Reverse == msg->getGear())
	{
		_x2 = tan_psi_a - tan_psi_t;
	}
	else
	{

	}
	_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

	temp_a = WHEEL_BASE * cos_psi_a * cos_psi_a * cos_psi_a;
	temp_b = tanf(delta_t)/(WHEEL_BASE * cos_psi_t * cos_psi_t * cos_psi_t) +
			COEFFICIENT_SMV * _x2 +
			COEFFICIENT_RHO * SatFunction(_sliding_variable) +
			COEFFICIENT_K   * _sliding_variable;

	delta_ctl = atanf(temp_a * temp_b);

	delta_ctl = delta_ctl > 0.54 ? 0.54 : delta_ctl < -0.54 ? -0.54:delta_ctl;
	ctl->SteeringAngle 		= delta_ctl * 16 * 57.3f;
	ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
}


/*
 * t_track：目标路径参数变量
 * t_track.point:目标路径与车辆后轴最近的点
 * t_track.yaw:目标路径的航向角@(-pi,pi)
 * t_track.curvature: 目标路径处的曲率
 *
 * a_track：实际跟踪变量
 * a_track.point:实时跟踪坐标
 * a_track.yaw:实时航向角@(-pi,pi)
 */
void LatControl::ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float cos_psi_a,cos_psi_t;
	float tan_psi_a,tan_psi_t;
	float delta_t,rote_angle;
	float temp_a,temp_b;
	float delta_ctl;

	float yaw_a,yaw_t;
	Vector2d point_a,point_t;

	if((a_track->getYaw() >= -M_PI) && (a_track->getYaw() < -M_3PI4))
	{
		rote_angle = M_3PI4;
	}
	else if((a_track->getYaw() >= -M_3PI4) && (a_track->getYaw() < -M_PI2))
	{
		rote_angle = M_PI2;
	}
	else if((a_track->getYaw() >= -M_PI2) && (a_track->getYaw() < -M_PI4))
	{
		rote_angle = M_PI4;
	}
	else if((a_track->getYaw() >= -M_PI4) && (a_track->getYaw() < 0))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= 0) && (a_track->getYaw() < M_PI4))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= M_PI4) && (a_track->getYaw() < M_PI2))
	{
		rote_angle = -M_PI4;
	}
	else if((a_track->getYaw() >= M_PI2) && (a_track->getYaw() < M_3PI4))
	{
		rote_angle = -M_PI2;
	}
	else if((a_track->getYaw() >= M_3PI4) && (a_track->getYaw() < M_PI))
	{
		rote_angle = -M_3PI4;
	}
	else
	{

	}

	point_a = a_track->getPosition().rotate(rote_angle);
	point_t = t_track.point.rotate(rote_angle);

	yaw_a = a_track->getYaw() + rote_angle;
	yaw_t = t_track.yaw       + rote_angle;

	cos_psi_t = cosf(yaw_t);
	tan_psi_t = tanf(yaw_t);

	cos_psi_a = cosf(yaw_a);
	tan_psi_a = tanf(yaw_a);

	delta_t = atanf(WHEEL_BASE*t_track.curvature);

	_x1 = point_t.getY() - point_a.getY();

	if(Drive == msg->getGear())
	{
		_x2 = tan_psi_t - tan_psi_a;
	}
	else if(Reverse == msg->getGear())
	{
		_x2 = tan_psi_a - tan_psi_t;
	}
	else
	{

	}
	_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

	temp_a = WHEEL_BASE * cos_psi_a * cos_psi_a * cos_psi_a;
	temp_b = tanf(delta_t)/(WHEEL_BASE * cos_psi_t * cos_psi_t * cos_psi_t) +
			COEFFICIENT_SMV * _x2 +
			COEFFICIENT_RHO * SatFunction(_sliding_variable) +
			COEFFICIENT_K   * _sliding_variable;

	delta_ctl = atanf(temp_a * temp_b);

	delta_ctl = delta_ctl > 0.54 ? 0.54 : delta_ctl < -0.54 ? -0.54:delta_ctl;
	ctl->SteeringAngle 		= delta_ctl * 16 * 57.3f;
	ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
}

/*
 * t_track：目标路径参数变量
 * t_track.point:目标路径与车辆后轴最近的点
 * t_track.yaw:目标路径的航向角@(-pi,pi)
 * t_track.curvature: 目标路径处的曲率
 *
 * a_track：实际跟踪变量
 * a_track.point:实时跟踪坐标
 * a_track.yaw:实时航向角@(-pi,pi)
 */
void LatControl::RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float err_yaw,err_cro;
	Vector2d vec_d,vec_t;
	float psi_omega;
	float v_x;
	float delta_ctl;

	vec_d = a_track->getPosition() - t_track.point;
	vec_t = Vector2d(cosf(t_track.yaw),sinf(t_track.yaw));
	err_cro = vec_d.CrossProduct(vec_t);
	err_yaw = a_track->getYaw() - t_track.yaw;

	v_x = msg->getVehicleMiddleSpeed();

	psi_omega = v_x * t_track.curvature * cosf(err_yaw)/(1.0f - t_track.curvature * err_cro)
			  - COEFFICIENT_KE   * v_x * sinf(err_yaw) * err_cro / err_yaw
			  - COEFFICIENT_KPSI * fabs(v_x) * err_yaw ;

	if(fabs(psi_omega) < 1.0e-6f || fabs(err_yaw) < 1.0e-6f)
	{
		ctl->SteeringAngle 		= 0.0f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
	else
	{
		delta_ctl = atanf(psi_omega * WHEEL_BASE / v_x);
		delta_ctl = delta_ctl > 0.54 ? 0.54 : delta_ctl < -0.54 ? -0.54:delta_ctl;
		ctl->SteeringAngle 		= delta_ctl * 16 * 57.3f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
}

void LatControl::Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *track)
{
	switch(_lat_control_status)
	{
		case init_status:
			if(ctl->getAPAEnable() && (msg->getSteeringAngle() < 5.0f) && (msg->getSteeringAngle() > -5.0f))
			{
				track->Init(); // 跟踪位置初始化，从坐标零点开始控制
				_lat_control_status = process_status;
			}
			else
			{
				ctl->SteeringAngle 		= 0;
				ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
			}
			break;

		case process_status:
			if(ctl->getAPAEnable())
			{
				_target_track = TrackingCurve(track->getPosition().getX());
//				ProcV1_0(msg,ctl,track,_target_track);
				RearWheelFeedback(msg,ctl,track,_target_track);
			}
			else
			{
				_lat_control_status = init_status;
			}
			break;

		default:
			break;
	}
}

TargetTrack LatControl::getTargetTrack()                  { return  _target_track;}
void        LatControl::setTargetTrack(TargetTrack value) { _target_track = value;}

float LatControl::getX1()            { return  _x1;}
void  LatControl::setX1(float value) { _x1 = value;}

float LatControl::getX2()            { return  _x2;}
void  LatControl::setX2(float value) { _x2 = value;}

float LatControl::getSlidingVariable()            { return  _sliding_variable;}
void  LatControl::setSlidingVariable(float value) { _sliding_variable = value;}
