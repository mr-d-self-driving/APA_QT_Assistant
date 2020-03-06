/*
 * lat_control.h
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#ifndef LATCONTROL_LAT_CONTROL_H_
#define LATCONTROL_LAT_CONTROL_H_

//#include "property.h"
#include "Control/Interface/controller.h"

#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

#define COEFFICIENT_TLS 	( 0.4f  )	// 目标曲线曲率因子
#define COEFFICIENT_SMV 	( 12.0f )	// 滑模变量系数
#define COEFFICIENT_RHO 	( 0.02f )	// 补偿噪声系数
#define COEFFICIENT_K   	( 1.1f  )	// 指数趋近率系数
#define COEFFICIENT_DELTA	( 0.1f  )   // Sat函数系数

#define COEFFICIENT_KPSI	( 1.0f  )   // K_psi
#define COEFFICIENT_KE		( 0.5f  )   // K_e

typedef enum _LatControlStatus
{
	init_status = 0,
	process_status
}LatControlStatus;


typedef struct _TargetTrack
{
	Vector2d point;
	float    yaw;
	float    curvature;
}TargetTrack;

/**
 * @brief The TrackLinkList class：曲线跟踪目标曲线链表
 */
class TrackLinkList : public LinkList<TargetTrack>{};

class LatControl :public Controller
{
public:
	LatControl();
	virtual ~LatControl();

	void Init() override;

	float TargetLine(float x);
	float TargetLineFirstDerivative(float x);
	float TargetLineSecondDerivative(float x);

	TargetTrack TrackingCurve(float x);

    void GenerateCurvatureSets(TrackLinkList *list);

    TargetTrack CalculateNearestPoint(TrackLinkList *list,GeometricTrack *a_track);

    float pi2pi(float angle);

	float SatFunction(float x);

	void Proc(MessageManager *msg,VehicleController *ctl,PID *pid) override;
	void ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track);

	void RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track);
    void Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track);

	TargetTrack getTargetTrack();
	void setTargetTrack(TargetTrack value);

	float getX1();
	void  setX1(float value);

	float getX2();
	void  setX2(float value);

	float getSlidingVariable();
	void  setSlidingVariable(float value);

private:
	TargetTrack _target_track;
	float _x1;
	float _x2;
	float _sliding_variable;

	LatControlStatus _lat_control_status;

//    TrackLinkList *_target_curvature_data_sets;

};

#endif /* LATCONTROL_LAT_CONTROL_H_ */
