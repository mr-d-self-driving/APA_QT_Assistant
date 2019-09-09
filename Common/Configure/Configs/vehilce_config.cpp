/*
 * vehilce_config.cpp
 *
 *  Created on: 2019年1月10日
 *      Author: zhuguohua
 */
/*****************************************************************************/
/* FILE NAME: vehilce_config.cpp                  COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: configure the vehile information and the steering and radius */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 10 2019     Initial Version                  */
/* 1.1	 Guohua Zhu     May     13 2019     Steering angle calculatge        */
/*****************************************************************************/
#include "./Common/Configure/Configs/vehilce_config.h"

VehilceConfig::VehilceConfig() {
	FrontDiagonalAxis.setContainer(this);
	FrontDiagonalAxis.getter(&VehilceConfig::getFrontDiagonalAxis);
	FrontDiagonalAxis.setter(&VehilceConfig::setFrontDiagonalAxis);

	FrontDiagonalAngle.setContainer(this);
	FrontDiagonalAngle.getter(&VehilceConfig::getFrontDiagonalAngle);
	FrontDiagonalAngle.setter(&VehilceConfig::setFrontDiagonalAngle);

	RearDiagonalAxis.setContainer(this);
	RearDiagonalAxis.getter(&VehilceConfig::getRearDiagonalAxis);
	RearDiagonalAxis.setter(&VehilceConfig::setRearDiagonalAxis);

	RearDiagonalAngle.setContainer(this);
	RearDiagonalAngle.getter(&VehilceConfig::getRearDiagonalAngle);
	RearDiagonalAngle.setter(&VehilceConfig::setRearDiagonalAngle);

	RadiusFrontLeft.setContainer(this);
	RadiusFrontLeft.getter(&VehilceConfig::getRadiusFrontLeft);
	RadiusFrontLeft.setter(&VehilceConfig::setRadiusFrontLeft);

	RadiusFrontRight.setContainer(this);
	RadiusFrontRight.getter(&VehilceConfig::getRadiusFrontRight);
	RadiusFrontRight.setter(&VehilceConfig::setRadiusFrontRight);

	RadiusRearLeft.setContainer(this);
	RadiusRearLeft.getter(&VehilceConfig::getRadiusRearLeft);
	RadiusRearLeft.setter(&VehilceConfig::setRadiusRearLeft);

	RadiusRearRight.setContainer(this);
	RadiusRearRight.getter(&VehilceConfig::getRadiusRearRight);
	RadiusRearRight.setter(&VehilceConfig::setRadiusRearRight);

	UltrasonicLocationArray.setContainer(this);
	UltrasonicLocationArray.getter(&VehilceConfig::getUltrasonicLocationArray);

	AccelerateTable.setContainer(this);
	AccelerateTable.getter(&VehilceConfig::getAccelerateTable);

	VelocityTable.setContainer(this);
	VelocityTable.getter(&VehilceConfig::getVelocityTable);

	TorqueTable.setContainer(this);
	TorqueTable.getter(&VehilceConfig::getTorqueTable);

	AccNum.setContainer(this);
	AccNum.getter(&VehilceConfig::getAccNum);

	VlcNum.setContainer(this);
	VlcNum.getter(&VehilceConfig::getVlcNum);

	Init();
}

VehilceConfig::~VehilceConfig() {

}

void VehilceConfig::Init()
{
	uint8_t i;

	FrontDiagonalAxis = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(FRONT_EDGE_TO_CENTER,2));
	RearDiagonalAxis  = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(REAR_EDGE_TO_CENTER,2));

	FrontDiagonalAngle = atanf(LEFT_EDGE_TO_CENTER/FRONT_EDGE_TO_CENTER);
	RearDiagonalAngle  = atanf(LEFT_EDGE_TO_CENTER/REAR_EDGE_TO_CENTER);

	_ultrasonic_location_array[0].Point.X = SENSOR1_X;
	_ultrasonic_location_array[0].Point.Y = SENSOR1_Y;
	_ultrasonic_location_array[0].Angle   = SENSOR1_ANGLE;

	_ultrasonic_location_array[1].Point.X = SENSOR2_X;
	_ultrasonic_location_array[1].Point.Y = SENSOR2_Y;
	_ultrasonic_location_array[1].Angle   = SENSOR2_ANGLE;

	_ultrasonic_location_array[2].Point.X = SENSOR3_X;
	_ultrasonic_location_array[2].Point.Y = SENSOR3_Y;
	_ultrasonic_location_array[2].Angle   = SENSOR3_ANGLE;

	_ultrasonic_location_array[3].Point.X = SENSOR4_X;
	_ultrasonic_location_array[3].Point.Y = SENSOR4_Y;
	_ultrasonic_location_array[3].Angle   = SENSOR4_ANGLE;

	_ultrasonic_location_array[4].Point.X = SENSOR5_X;
	_ultrasonic_location_array[4].Point.Y = SENSOR5_Y;
	_ultrasonic_location_array[4].Angle   = SENSOR5_ANGLE;

	_ultrasonic_location_array[5].Point.X = SENSOR6_X;
	_ultrasonic_location_array[5].Point.Y = SENSOR6_Y;
	_ultrasonic_location_array[5].Angle   = SENSOR6_ANGLE;

	_ultrasonic_location_array[6].Point.X = SENSOR7_X;
	_ultrasonic_location_array[6].Point.Y = SENSOR7_Y;
	_ultrasonic_location_array[6].Angle   = SENSOR7_ANGLE;

	_ultrasonic_location_array[7].Point.X = SENSOR8_X;
	_ultrasonic_location_array[7].Point.Y = SENSOR8_Y;
	_ultrasonic_location_array[7].Angle   = SENSOR8_ANGLE;

	_ultrasonic_location_array[8].Point.X = SENSOR9_X;
	_ultrasonic_location_array[8].Point.Y = SENSOR9_Y;
	_ultrasonic_location_array[8].Angle   = SENSOR9_ANGLE;

	_ultrasonic_location_array[9].Point.X = SENSOR10_X;
	_ultrasonic_location_array[9].Point.Y = SENSOR10_Y;
	_ultrasonic_location_array[9].Angle   = SENSOR10_ANGLE;

	_ultrasonic_location_array[10].Point.X = SENSOR11_X;
	_ultrasonic_location_array[10].Point.Y = SENSOR11_Y;
	_ultrasonic_location_array[10].Angle   = SENSOR11_ANGLE;

	_ultrasonic_location_array[11].Point.X = SENSOR12_X;
	_ultrasonic_location_array[11].Point.Y = SENSOR12_Y;
	_ultrasonic_location_array[11].Angle   = SENSOR12_ANGLE;

	/*
	 * 初始化扭矩标定表
	 * */

	_acc_num = ACC_ARRAY_NUM;
	_vlc_num  = VELOCITY_ARRAY_NUM;
	for(i=0;i<ACC_ARRAY_NUM;i++)
	{
		_accelerate_table[i] = acc_table[i];
	}
	for(i=0;i<VELOCITY_ARRAY_NUM;i++)
	{
		_velocity_table[i] = velocity_table[i];
	}
	for(i=0;i< (ACC_ARRAY_NUM * VELOCITY_ARRAY_NUM);i++)
	{
		_torque_table[i] = torque_table[i/VELOCITY_ARRAY_NUM][i%VELOCITY_ARRAY_NUM];
	}
}

// r is + and -
void VehilceConfig::EdgeRadius(float r)
{
	RadiusFrontLeft = sqrtf( powf( r - LEFT_EDGE_TO_CENTER , 2 ) +
			                 powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusFrontRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                              powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusRearLeft = sqrtf( powf( r -  LEFT_EDGE_TO_CENTER , 2 ) +
			                powf( REAR_EDGE_TO_CENTER , 2 ) );

	RadiusRearRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                             powf( REAR_EDGE_TO_CENTER , 2 ) );
}

float VehilceConfig::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
	return steer < 0 ? -WHEEL_BASE / tanf((a * -steer + b) * PI / 180) :
			            WHEEL_BASE / tanf((a *  steer + b) * PI / 180) ;
}

float VehilceConfig::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return steer < 0 ? -a * expf(b * -steer) : a * expf(b * steer);
}

float VehilceConfig::TurnningRadius2SteeringAngleExp(float radius,float a,float b)
{
	return radius < 0 ? -logf(-radius/a)/b : logf(radius/a)/b;
}

float VehilceConfig::TurnningRadius2SteeringAngle(float radius,float a,float b)
{
	return radius < 0 ? -(atanf(-WHEEL_BASE/radius) * 180 / PI - b)/a :
			             (atanf( WHEEL_BASE/radius) * 180 / PI - b)/a ;
}

float VehilceConfig::TurnRadiusCurveFitting(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle >  -50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle > -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle > -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle > -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle > -400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
									SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12);
}

float VehilceConfig::TurnRadiusFindingTable(float steering_angle)
{
    return steering_angle >=  0 ? SteerAngle2Radius[static_cast<uint16_t>(steering_angle)][0] : -SteerAngle2Radius[static_cast<uint16_t>(-steering_angle)][1];
}

float VehilceConfig::SteeringAngleCurveFitting(float radius)
{
	return
           radius >   48.308f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A6,FIT_RADIUS_B6) :
           radius >   24.018f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A5,FIT_RADIUS_B5) :
           radius >   11.910f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A4,FIT_RADIUS_B4) :
           radius >   7.6736f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A3,FIT_RADIUS_B3) :
           radius >    5.463f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A2,FIT_RADIUS_B2) :
           radius >      0.0f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A1,FIT_RADIUS_B1) :
           radius > - 5.4745f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A12,FIT_RADIUS_B12) :
           radius > - 7.7214f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A11,FIT_RADIUS_B11):
           radius > - 11.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A10,FIT_RADIUS_B10):
           radius > - 24.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A9,FIT_RADIUS_B9):
           radius > - 49.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A8,FIT_RADIUS_B8):
							   TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A7,FIT_RADIUS_B7);
}

float VehilceConfig::SteeringAngleFindingCallback(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	middle_id = (left_id + right_id) * 0.5;
	if(radius > 0)
	{
		if( (right_id - left_id) > 1)
		{
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return middle_id;
			}
		}
		else
		{
			return (left_id + right_id) * 0.5;
		}
	}
	else
	{
		if( (right_id - left_id) > 1)
		{
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return -middle_id;
			}
		}
		else
		{
			return -(left_id + right_id) * 0.5;
		}
	}
}

float VehilceConfig::SteeringAngleFindingLoop(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	if(radius > 0)
	{
		while( (right_id - left_id) > 1)
		{
			middle_id = (uint16_t)((left_id + right_id) * 0.5);
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				left_id = middle_id;
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				right_id = middle_id;
			}
			else
			{
				return middle_id;
			}
		}
		return (left_id + right_id) * 0.5;
	}
	else
	{
		while( (right_id - left_id) > 1)
		{
			middle_id = (uint16_t)((left_id + right_id) * 0.5);
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				left_id = middle_id;
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				right_id = middle_id;
			}
			else
			{
				return -middle_id;
			}
		}
		return -(left_id + right_id) * 0.5;
	}
}

float VehilceConfig::TurnRadiusCalculate(float steering_angle)
{
	return TurnRadiusFindingTable(steering_angle);
}

float VehilceConfig::SteeringAngleCalculate(float radius)
{
	return SteeringAngleFindingCallback(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
//	return SteeringAngleFindingLoop(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
}

float VehilceConfig::getRadiusFrontLeft()           { return  _radius_front_left;}
void  VehilceConfig::setRadiusFrontLeft(float value){ _radius_front_left = value;}
float VehilceConfig::getRadiusFrontRight()           { return  _radius_front_right;}
void  VehilceConfig::setRadiusFrontRight(float value){ _radius_front_right = value;}
float VehilceConfig::getRadiusRearLeft()           { return  _radius_rear_left;}
void  VehilceConfig::setRadiusRearLeft(float value){ _radius_rear_left = value;}
float VehilceConfig::getRadiusRearRight()           { return  _radius_rear_right;}
void  VehilceConfig::setRadiusRearRight(float value){ _radius_rear_right = value;}

float VehilceConfig::getFrontDiagonalAxis()           { return  _front_diagonal_axis;}
void  VehilceConfig::setFrontDiagonalAxis(float value){ _front_diagonal_axis = value;}
float VehilceConfig::getFrontDiagonalAngle()           { return  _front_diagonal_angle;}
void  VehilceConfig::setFrontDiagonalAngle(float value){ _front_diagonal_angle = value;}
float VehilceConfig::getRearDiagonalAxis()           { return  _rear_diagonal_axis;}
void  VehilceConfig::setRearDiagonalAxis(float value){ _rear_diagonal_axis = value;}
float VehilceConfig::getRearDiagonalAngle()           { return  _rear_diagonal_angle;}
void  VehilceConfig::setRearDiagonalAngle(float value){ _rear_diagonal_angle = value;}

Location* VehilceConfig::getUltrasonicLocationArray() { return  _ultrasonic_location_array;}

float* VehilceConfig::getAccelerateTable() { return  _accelerate_table;}
float* VehilceConfig::getVelocityTable() { return  _velocity_table;}
float* VehilceConfig::getTorqueTable() { return  _torque_table;}

uint16_t VehilceConfig::getAccNum() { return  _acc_num;}
uint16_t VehilceConfig::getVlcNum() { return  _vlc_num;}

