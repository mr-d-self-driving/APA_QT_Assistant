/*
 * geometric_track.cpp
 *
 *  Created on: January 9 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: geometric_track.cpp                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: track the vehilce position        					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/
#include "../GeometricTrack/geometric_track.h"

VehilceConfig m_GeometricVehicleConfig;

GeometricTrack::GeometricTrack() {

	SumRearLeftPulse.setContainer(this);
	SumRearLeftPulse.getter(&GeometricTrack::getSumRearLeftPulse);
	SumRearLeftPulse.setter(&GeometricTrack::setSumRearLeftPulse);

	SumRearRightPulse.setContainer(this);
	SumRearRightPulse.getter(&GeometricTrack::getSumRearRightPulse);
	SumRearRightPulse.setter(&GeometricTrack::setSumRearRightPulse);

	Init();
}

GeometricTrack::~GeometricTrack() {

}

void GeometricTrack::Init(void)
{
	_position.X = 0;
	_position.Y = 0;
	Yaw = 0.0f;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;

	_last_rear_left_pulse   = 0;
	_last_rear_right_pulse  = 0;
	_sum_rear_left_pulse    = 0;
	_sum_rear_right_pulse   = 0;
	_delta_rear_left_pulse  = 0;
	_delta_rear_right_pulse = 0;
}

void GeometricTrack::Init(float x,float y,float yaw)
{
	_position.setX(x);
	_position.Y = y;
	Yaw = yaw;
	_last_yaw = Yaw;
	LinearVelocity = 0.0f;

	_last_rear_left_pulse   = 0;
	_last_rear_right_pulse  = 0;
	_sum_rear_left_pulse    = 0;
	_sum_rear_right_pulse   = 0;
	_delta_rear_left_pulse  = 0;
	_delta_rear_right_pulse = 0;
}

void GeometricTrack::VelocityUpdate(MessageManager *msg,float dt)
{
	float radius;
	LinearRate = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft)*0.5;
	LinearVelocity = msg->WheelSpeedDirection == Forward ?  LinearRate :
					 msg->WheelSpeedDirection == Backward ? -LinearRate : 0;

	if( ((int16_t)fabs(msg->SteeringAngle) != 0) && ((int16_t)fabs(msg->SteeringAngle) < 520))
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg->SteeringAngle);
		Yaw  = _last_yaw + LinearVelocity * dt / radius;
		_position.X = _position.X + radius * (sinf(Yaw) - sinf(_last_yaw));
		_position.Y = _position.Y + radius * (cosf(_last_yaw) - cosf(Yaw));
	}
	else
	{
		_position.X = _position.X + LinearVelocity * cosf(Yaw) * dt;
		_position.Y = _position.Y + LinearVelocity * sinf(Yaw) * dt;
	}
	_last_yaw = Yaw;
}

void GeometricTrack::PulseUpdate(MessageManager *msg)
{
	float displacement,radius;

	if( (0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse) )
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =  msg->WheelSpeedDirection == Forward ?
								((msg->WheelPulseRearLeft >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg->WheelPulseRearLeft  - _last_rear_left_pulse) :
								  msg->WheelSpeedDirection == Backward ?
							   -((msg->WheelPulseRearLeft >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg->WheelPulseRearLeft  - _last_rear_left_pulse) : 0;
	}

	if( (0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse) )
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse =  msg->WheelSpeedDirection == Forward ?
								 ((msg->WheelPulseRearRight >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg->WheelPulseRearRight  - _last_rear_right_pulse) :
								   msg->WheelSpeedDirection == Backward ?
								-((msg->WheelPulseRearRight >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg->WheelPulseRearRight  - _last_rear_right_pulse) : 0;
	}
	displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;

	// 速度判定
	_velocity_line_rate = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft) * 0.5f;


	if(_velocity_line_rate < 0.2f)//低速情形
	{
		// 累积脉冲达到指定
		if(msg->WheelSpeedDirection == Forward)
		{

		}
		else if(msg->WheelSpeedDirection == Backward)
		{

		}
		else if(msg->WheelSpeedDirection == StandStill)
		{

		}
		else
		{

		}
		 _cumulation_rear_left_pulse  += _delta_rear_left_pulse;
		 _cumulation_rear_right_pulse += _delta_rear_right_pulse;
		 _cumulation_middle_displacement = (_cumulation_rear_left_pulse + _cumulation_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
		 if(_cumulation_middle_displacement >= 4.0f)
		 {
			 LinearVelocity = _cumulation_middle_displacement  * 50 /_wait_time_cnt;
		 }
		 else
		 {
			 _wait_time_cnt++;
		 }
	}
	else// 高速情形
	{
		LinearRate = _velocity_line_rate;
	}




	LinearVelocity = msg->WheelSpeedDirection == Forward ?  LinearRate :
					 msg->WheelSpeedDirection == Backward ? -LinearRate : 0;

	if( ((int16_t)fabs(msg->SteeringAngle) != 0) && ((uint16_t)fabs(msg->SteeringAngle) < 520))
	{
		radius = m_GeometricVehicleConfig.TurnRadiusCalculate(msg->SteeringAngle);
		Yaw  = _last_yaw + displacement / radius;
		_position.X = _position.X + radius * (sinf(Yaw) - sinf(_last_yaw));
		_position.Y = _position.Y + radius * (cosf(_last_yaw) - cosf(Yaw));
	}
	else
	{
		_position.X = _position.X + displacement * cosf(Yaw);
		_position.Y = _position.Y + displacement * sinf(Yaw);
	}

	_sum_rear_left_pulse  += _delta_rear_left_pulse;
	_sum_rear_right_pulse += _delta_rear_right_pulse;


	_last_rear_left_pulse  = msg->WheelPulseRearLeft;
	_last_rear_right_pulse = msg->WheelPulseRearRight;
	_last_yaw = Yaw;
}

void GeometricTrack::PulseTrackUpdate(MessageManager *msg)
{
	float displacement;

	if( (0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse) )
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =  msg->WheelSpeedDirection == Forward ?
								((msg->WheelPulseRearLeft >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg->WheelPulseRearLeft  - _last_rear_left_pulse) :
								  msg->WheelSpeedDirection == Backward ?
							   -((msg->WheelPulseRearLeft >= _last_rear_left_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								  msg->WheelPulseRearLeft  - _last_rear_left_pulse) : 0;
	}

	if( (0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse) )
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse =  msg->WheelSpeedDirection == Forward ?
								 ((msg->WheelPulseRearRight >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg->WheelPulseRearRight  - _last_rear_right_pulse) :
								   msg->WheelSpeedDirection == Backward ?
								-((msg->WheelPulseRearRight >= _last_rear_right_pulse  ? 0 : WHEEL_PUSLE_MAX) +
								   msg->WheelPulseRearRight  - _last_rear_right_pulse) : 0;
	}

	displacement = (_delta_rear_left_pulse + _delta_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	if(_delta_rear_right_pulse == _delta_rear_left_pulse)
	{
		_position.X = _position.X + displacement * cosf(Yaw);
		_position.Y = _position.Y + displacement * sinf(Yaw);
	}
	else
	{
		_delta_yaw = (_delta_rear_right_pulse - _delta_rear_left_pulse)/WIDTH;
		Yaw  = _last_yaw + _delta_yaw;
		_position.X = _position.X + displacement * cosf(_last_yaw + _delta_yaw*0.5);
		_position.Y = _position.Y + displacement * sinf(_last_yaw + _delta_yaw*0.5);
	}
	_last_yaw = Yaw;
}

void GeometricTrack::VelocityPulseUpdate(MessageManager *msg)
{
	if( (0 == _delta_rear_left_pulse) && (0 == _last_rear_left_pulse) )
	{
		_delta_rear_left_pulse = 0;
	}
	else
	{
		_delta_rear_left_pulse =(msg->WheelPulseRearLeft >= _last_rear_left_pulse ? 0 : WHEEL_PUSLE_MAX) +
								 msg->WheelPulseRearLeft  - _last_rear_left_pulse ;
	}

	if( (0 == _delta_rear_right_pulse) && (0 == _last_rear_right_pulse) )
	{
		_delta_rear_right_pulse = 0;
	}
	else
	{
		_delta_rear_right_pulse = (msg->WheelPulseRearRight >= _last_rear_right_pulse ? 0 : WHEEL_PUSLE_MAX) +
								   msg->WheelPulseRearRight  - _last_rear_right_pulse ;
	}
	// 速度判定
	if((msg->WheelSpeedRearRight > 0.2f) && (msg->WheelSpeedRearLeft > 0.2f))
	{
		msg->VehicleMiddleSpeed = (msg->WheelSpeedRearRight + msg->WheelSpeedRearLeft) * 0.5f;
	}
	else
	{
		msg->VehicleMiddleSpeed = 0;
	}
	/*************************************************脉冲计算速度********************************************************/
	 if((_delta_rear_left_pulse > 0) || (_delta_rear_right_pulse > 0))//脉冲更新解锁
	 {
		 _velocity_lock = 0;
	 }
	// 累积脉冲达到指定
	 _cumulation_rear_left_pulse  += _delta_rear_left_pulse;
	 _cumulation_rear_right_pulse += _delta_rear_right_pulse;
	 _cumulation_middle_displacement = (_cumulation_rear_left_pulse + _cumulation_rear_right_pulse) * 0.5f * WHEEL_PUSLE_RATIO;
	 if(_cumulation_middle_displacement >= 0.04f)
	 {
		 _velocity_lock = 0;
		 _wait_time_cnt++;
		 PulseUpdateVelocity = _cumulation_middle_displacement  * 50 /_wait_time_cnt;
		 _cumulation_rear_left_pulse  = 0;
		 _cumulation_rear_right_pulse = 0;
		 _wait_time_cnt               = 0;
	 }
	 else if(_wait_time_cnt >= 25)
	 {
		 if((msg->WheelSpeedRearRight < 1.0e-6) && (msg->WheelSpeedRearLeft < 1.0e-6))
		 {
			 _velocity_lock               = 0xff;
			 PulseUpdateVelocity          = 0;
			 _cumulation_rear_left_pulse  = 0;
			 _cumulation_rear_right_pulse = 0;
			 _wait_time_cnt               = 0;
		 }
	 }
	 else
	 {
		 if(0xff != _velocity_lock)
		 {
			 _wait_time_cnt++;
		 }
	 }
	 /**************************************************脉冲计算结束****************************************/
	 if(msg->VehicleMiddleSpeed < 1.0e-6)
	 {
		 if( PulseUpdateVelocity < 0.5f )
		 {
			 msg->VehicleMiddleSpeed = PulseUpdateVelocity;
		 }
	 }

	_sum_rear_left_pulse  += _delta_rear_left_pulse;
	_sum_rear_right_pulse += _delta_rear_right_pulse;

	_last_rear_left_pulse  = msg->WheelPulseRearLeft;
	_last_rear_right_pulse = msg->WheelPulseRearRight;
}
/**************************************************************************************/
int32_t GeometricTrack::getSumRearLeftPulse()             { return _sum_rear_left_pulse;}
void    GeometricTrack::setSumRearLeftPulse(int32_t value){_sum_rear_left_pulse = value;}

int32_t GeometricTrack::getSumRearRightPulse()             { return _sum_rear_right_pulse;}
void    GeometricTrack::setSumRearRightPulse(int32_t value){_sum_rear_right_pulse = value;}
