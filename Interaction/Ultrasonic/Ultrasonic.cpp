/*
 * ultrasonic.cpp
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: ultrasonic.cpp                      COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this module process the ultrasonic data				         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	Init();

	ScheduleTimeCnt.setContainer(this);
	ScheduleTimeCnt.getter(&Ultrasonic::getScheduleTimeCnt);
	ScheduleTimeCnt.setter(&Ultrasonic::setScheduleTimeCnt);

	ReadStage.setContainer(this);
	ReadStage.getter(&Ultrasonic::getReadStage);
	ReadStage.setter(&Ultrasonic::setReadStage);

	SystemTime.setContainer(this);
	SystemTime.getter(&Ultrasonic::getSystemTime);
	SystemTime.setter(&Ultrasonic::setSystemTime);

	UltrasonicPacket.setContainer(this);
	UltrasonicPacket.getter(&Ultrasonic::getUltrasonicPacket);

	UltrasonicLocationPacket.setContainer(this);
	UltrasonicLocationPacket.getter(&Ultrasonic::getUltrasonicLocationPacket);

	AbstacleBodyPositionDirect.setContainer(this);
	AbstacleBodyPositionDirect.getter(&Ultrasonic::getAbstacleBodyPositionDirect);

	AbstacleBodyPositionTriangle.setContainer(this);
	AbstacleBodyPositionTriangle.getter(&Ultrasonic::getAbstacleBodyPositionTriangle);

	AbstacleGroundPositionTriangle.setContainer(this);
	AbstacleGroundPositionTriangle.getter(&Ultrasonic::getAbstacleGroundPositionTriangle);
}

Ultrasonic::~Ultrasonic() {

}

void Ultrasonic::Init(void)
{
	_system_time = 0;
	_schedule_time_cnt = 0;
	_read_stage = 0;
}

uint8_t Ultrasonic::getScheduleTimeCnt()             {return _schedule_time_cnt ;}
void    Ultrasonic::setScheduleTimeCnt(uint8_t value){_schedule_time_cnt = value;}

uint8_t Ultrasonic::getReadStage()             {return _read_stage ;}
void    Ultrasonic::setReadStage(uint8_t value){_read_stage = value;}

uint32_t Ultrasonic::getSystemTime()              {return _system_time ;}
void     Ultrasonic::setSystemTime(uint32_t value){_system_time = value;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicPacket(){return _ultrasonic_packet;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicLocationPacket(){return _ultrasonic_location_packet;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionDirect(){return _abstacle_body_position_direct;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionTriangle(){return _abstacle_body_position_triangle;}

ObstacleLocationPacket* Ultrasonic::getAbstacleGroundPositionTriangle(){return _abstacle_ground_position_triangle;}

void Ultrasonic::setUltrasonicPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_packet[n] = p;
}

void Ultrasonic::setUltrasonicLocationPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_location_packet[n] = p;
}

void Ultrasonic::setAbstacleGroundPositionTriangle(uint8_t n,ObstacleLocationPacket p)
{
	_abstacle_ground_position_triangle[n] = p;
}

/*
 * 直接测量数据的传感器坐标系与载体坐标系的转换
 * position: 传感器的安装坐标
 * data    : 超声波数据
 * location: 障碍物相对车体的位置
 * */
void Ultrasonic::BodyDirectCalculate(Location position,Ultrasonic_Packet data,ObstacleLocationPacket *location)
{
	Vector2d temp_angle;
	if(0 == data.status)
	{
		if(0 == data.Distance1)
		{
			location->Position = position.Point;
			location->Status = OverDetection;
		}
		else
		{
			temp_angle = Vector2d(data.Distance1,0);
			location->Position = position.Point + temp_angle.rotate(position.Angle);
			location->Status = Normal;
		}
	}
	else
	{
		location->Position = position.Point;
		if(16 == data.status)
		{
			location->Status = BlindZone;
		}
		else if(2 == data.status)
		{
			location->Status = Noise;
		}
	}
}

/*
 * 三角定位测量值 由传感器坐标系转到载体坐标系
 * type:三角顶点朝向；1 -> 朝上   0 -> 朝下
 * position_a:传感器安装位置a
 * position_b:传感器安装位置b
 * data_ul:三角定位测量的左边长值
 * data_ur:三角定位测量的右边长值
 * location:障碍物定位坐标
 * */
void Ultrasonic::BodyTriangleCalculate(int8_t type,Location position_a,Location position_b,
										Ultrasonic_Packet data_ul,Ultrasonic_Packet data_ur,
										ObstacleLocationPacket *location)
{
	float bottom_edge;
	float alpha,beta;
	Vector2d temp_angle;

	if( (0 == data_ul.status) && (0 == data_ur.status))
	{
		if( (0 == data_ul.Distance1) || (0 == data_ur.Distance1))
		{
			location->Position = position_a.Point;// Vector2d(0,0);
			location->Status   = OverDetection;
		}
		else
		{
			bottom_edge = ( position_b.Point - position_a.Point ).Length();
			if( (     (data_ul.Distance1 + data_ur.Distance1) >  bottom_edge      ) &&
				( fabs(data_ul.Distance1 - data_ur.Distance1) <  bottom_edge      ) &&
				( fabs(data_ul.Distance1 - data_ur.Distance1) < (bottom_edge * 0.8) )
			)
			{
				alpha = ( position_b.Point - position_a.Point ).Angle();
				if(1 == type)
				{
					beta =  acosf( (bottom_edge * bottom_edge + data_ul.Distance1 * data_ul.Distance1 - data_ur.Distance1 * data_ur.Distance1) / (2 * data_ul.Distance1 * bottom_edge));
				}
				else if(-1 == type)
				{
					beta = -acosf( (bottom_edge * bottom_edge + data_ul.Distance1 * data_ul.Distance1 - data_ur.Distance1 * data_ur.Distance1) / (2 * data_ul.Distance1 * bottom_edge));
				}
				temp_angle = Vector2d(data_ul.Distance1,0);
				location->Position = position_a.Point + temp_angle.rotate(alpha + beta);
				location->Status   = Normal;
			}
			else
			{
				location->Position = position_a.Point;
				location->Status   = InvalidPoint;
			}
		}
	}
	else
	{
		location->Position = position_a.Point;
		if( (16 == data_ul.status) || (16 == data_ur.status))
		{
			location->Status = BlindZone;
		}
		else if((2 == data_ul.status) || (2 == data_ur.status))
		{
			location->Status = Noise;
		}
		else
		{
			location->Status = InvalidPoint;
		}
	}
}

/*
 * 三角定位地面坐标系的转换
 * vehicle:车辆状态信息
 * body   :障碍物相对于载体坐标系的坐标
 * ground :障碍物相对于地面坐标系的坐标
 * */
void Ultrasonic::GroundTriangleCalculate(VehicleState *vehicle,ObstacleLocationPacket body,ObstacleLocationPacket *ground)
{
	ground->Position = vehicle->getPosition() + body.Position.rotate(vehicle->getYaw());
	ground->Status   = body.Status;
}

/*
 * 基于直接测量值的载体坐标系转换
 * */
void Ultrasonic::BodyDirectLocation()
{
	switch(ScheduleTimeCnt)
	{
		case 0:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[0],UltrasonicPacket[0],&AbstacleBodyPositionDirect[0]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[7],UltrasonicPacket[7],&AbstacleBodyPositionDirect[7]);
			break;


		case 8:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[1],UltrasonicPacket[1],&AbstacleBodyPositionDirect[1]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[6],UltrasonicPacket[6],&AbstacleBodyPositionDirect[6]);
			break;

		case 11:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);
			break;

		case 13:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		case 14:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[3],UltrasonicPacket[3],&AbstacleBodyPositionDirect[3]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[4],UltrasonicPacket[4],&AbstacleBodyPositionDirect[4]);
			break;

		case 22:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[2],UltrasonicPacket[2],&AbstacleBodyPositionDirect[2]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[5],UltrasonicPacket[5],&AbstacleBodyPositionDirect[5]);
			break;


		case 25:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[8],UltrasonicPacket[8],&AbstacleBodyPositionDirect[8]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[10],UltrasonicPacket[10],&AbstacleBodyPositionDirect[10]);

			break;

		case 27:
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[9],UltrasonicPacket[9],&AbstacleBodyPositionDirect[9]);
			BodyDirectCalculate( _abstacle_config.UltrasonicLocationArray[11],UltrasonicPacket[11],&AbstacleBodyPositionDirect[11]);
			break;

		default:
			break;
	}
}

void Ultrasonic::BodyTriangleLocation()
{
	switch(ScheduleTimeCnt)
	{
		case 8:
			BodyTriangleCalculate(1,_abstacle_config.UltrasonicLocationArray[0],_abstacle_config.UltrasonicLocationArray[1],
									UltrasonicLocationPacket[0],UltrasonicLocationPacket[1],
									&AbstacleBodyPositionTriangle[0]);

			BodyTriangleCalculate(1,_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
								    UltrasonicLocationPacket[1],UltrasonicLocationPacket[2],
									&AbstacleBodyPositionTriangle[1]);

			BodyTriangleCalculate(1,_abstacle_config.UltrasonicLocationArray[1],_abstacle_config.UltrasonicLocationArray[2],
									UltrasonicLocationPacket[3],UltrasonicLocationPacket[4],
									&AbstacleBodyPositionTriangle[2]);

			BodyTriangleCalculate(1,_abstacle_config.UltrasonicLocationArray[2],_abstacle_config.UltrasonicLocationArray[3],
									UltrasonicLocationPacket[4],UltrasonicLocationPacket[5],
									&AbstacleBodyPositionTriangle[3]);
			break;

		case 22:
			BodyTriangleCalculate(-1,_abstacle_config.UltrasonicLocationArray[4],_abstacle_config.UltrasonicLocationArray[5],
									 UltrasonicLocationPacket[6],UltrasonicLocationPacket[7],
									 &AbstacleBodyPositionTriangle[4]);

			BodyTriangleCalculate(-1,_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
									 UltrasonicLocationPacket[7],UltrasonicLocationPacket[8],
									 &AbstacleBodyPositionTriangle[5]);

			BodyTriangleCalculate(-1,_abstacle_config.UltrasonicLocationArray[5],_abstacle_config.UltrasonicLocationArray[6],
									 UltrasonicLocationPacket[9],UltrasonicLocationPacket[10],
									 &AbstacleBodyPositionTriangle[6]);

			BodyTriangleCalculate(-1,_abstacle_config.UltrasonicLocationArray[6],_abstacle_config.UltrasonicLocationArray[7],
									 UltrasonicLocationPacket[10],UltrasonicLocationPacket[11],
									 &AbstacleBodyPositionTriangle[7]);
			break;

		default:
			break;
	}

}

void Ultrasonic::GroundTriangleLocation(VehicleState *vehicle_state)
{
	switch(ScheduleTimeCnt)
	{
		case 8:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[0],&AbstacleGroundPositionTriangle[0]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[1],&AbstacleGroundPositionTriangle[1]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[2],&AbstacleGroundPositionTriangle[2]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[3],&AbstacleGroundPositionTriangle[3]);
			break;

		case 11:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[8],&AbstacleGroundPositionTriangle[8]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[10],&AbstacleGroundPositionTriangle[10]);
			break;

		case 13:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[9],&AbstacleGroundPositionTriangle[9]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[11],&AbstacleGroundPositionTriangle[11]);
			break;

		case 22:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[4],&AbstacleGroundPositionTriangle[4]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[5],&AbstacleGroundPositionTriangle[5]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[6],&AbstacleGroundPositionTriangle[6]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionTriangle[7],&AbstacleGroundPositionTriangle[7]);
			break;

		case 25:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[8],&AbstacleGroundPositionTriangle[8]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[10],&AbstacleGroundPositionTriangle[10]);
			break;

		case 27:
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[9],&AbstacleGroundPositionTriangle[9]);
			GroundTriangleCalculate(vehicle_state,AbstacleBodyPositionDirect[11],&AbstacleGroundPositionTriangle[11]);
			break;

		default:
			break;
	}
}
