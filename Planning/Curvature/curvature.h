#ifndef CURVATURE_H
#define CURVATURE_H


#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

#include "Control/Common/trajectory_analyzer.h"

#define COEFFICIENT_TLS 	( 1.2f  )	// 目标曲线曲率因子


class Curvature
{
public:
    Curvature();

    void Init();

    float TargetLine(float x);
    float TargetLineFirstDerivative(float x);
    float TargetLineSecondDerivative(float x);

    TargetTrack TrackingCurve(float x);

    /**
     * @brief GenerateCurvatureSets：生成目标曲线数据集
     * @param list：目标曲线数据集
     */
    void GenerateCurvatureSets(TrackLinkList *list,uint16_t type);



private:
    TargetTrack _target_track;
    Node<TargetTrack>* track_node;
    TrackLinkList    * track_list;
};

#endif // CURVATURE_H
